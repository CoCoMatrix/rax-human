"""Restart one allowlisted personal PiPER deployment and verify it becomes ready.

This module never sends Prepare or Start.  It only replaces the one formal
``launch_robot.sh`` process that owns the current graph, and is deliberately
unavailable outside the explicit personal deployment catalogue.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any, Mapping
from urllib.error import URLError
from urllib.request import urlopen


REPO = Path(__file__).resolve().parents[3]
CATALOG_PATH = REPO / "policy/Pi_05/junqi_personal_web_deployments.json"
ARTIFACT_DIR = Path("/home/ubuntu/code_junqi/migration/piper-deployment-web-switch")
STATUS_PATH = ARTIFACT_DIR / "status.json"
LOCK_PATH = ARTIFACT_DIR / "active.lock"
LAUNCHER = REPO / "scripts/launch_robot.sh"
REGISTRY = REPO / "policy/Pi_05/piper_deployments.py"
READY_TIMEOUT_SECONDS = 300.0
REQUIRED_CAMERAS = ("left", "head", "right")


class DeploymentSwitchError(ValueError):
    """A rejected switch request leaves the existing graph untouched."""


@dataclass(frozen=True)
class DeploymentOption:
    name: str
    label: str


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise DeploymentSwitchError(f"invalid JSON object: {path}")
    return value


def load_catalog(path: Path = CATALOG_PATH) -> tuple[DeploymentOption, ...]:
    value = _read_json(path)
    if value.get("schema") != "xpolicylab.piper-web-deployment-switch/v1":
        raise DeploymentSwitchError("personal deployment catalogue schema is invalid")
    rows = value.get("deployments")
    if not isinstance(rows, list) or not rows:
        raise DeploymentSwitchError("personal deployment catalogue is empty")
    if path == CATALOG_PATH:
        from XPolicyLab.policy.Pi_05.web_deployment_intake import registered_options
        rows = [*rows, *registered_options()]
    result: list[DeploymentOption] = []
    names: set[str] = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"name", "label"}:
            raise DeploymentSwitchError("invalid personal deployment entry")
        name = str(row["name"]).strip()
        label = str(row["label"]).strip()
        if (
            not name
            or name != Path(name).name
            or not all(character.isalnum() or character in {"-", "_"} for character in name)
            or not label
            or len(label) > 120
            or name in names
        ):
            raise DeploymentSwitchError("invalid personal deployment entry")
        descriptor = REPO / "policy/Pi_05/deployments" / f"{name}.yml"
        if not descriptor.is_file():
            raise DeploymentSwitchError(f"personal deployment descriptor is missing: {name}")
        names.add(name)
        result.append(DeploymentOption(name, label))
    return tuple(result)


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def read_status() -> dict[str, Any]:
    try:
        return _read_json(STATUS_PATH)
    except (OSError, json.JSONDecodeError, DeploymentSwitchError):
        return {"phase": "idle"}


def _process_arguments(pid: int) -> list[str]:
    try:
        values = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    except OSError as exc:
        raise DeploymentSwitchError("current formal launcher is unavailable") from exc
    result = [value.decode("utf-8") for value in values if value]
    if not result:
        raise DeploymentSwitchError("current formal launcher has no command line")
    return result


def _process_is_live(pid: int) -> bool:
    try:
        state = Path(f"/proc/{pid}/status").read_text(encoding="utf-8")
    except OSError:
        return False
    return "\nState:\tZ" not in state


def _canonical_launcher(args: list[str]) -> list[str]:
    # launch_robot.sh changes cwd to the graph directory after startup, so
    # /proc/PID/cwd cannot resolve its original relative command line.
    result = list(args)
    for index, argument in enumerate(result):
        if argument in {str(LAUNCHER), "scripts/launch_robot.sh", "./scripts/launch_robot.sh"}:
            result[index] = str(LAUNCHER)
            return result
    raise DeploymentSwitchError("unsupported launcher command")


def _launcher_environment(pid: int) -> dict[str, str]:
    # Use the launcher's initial environment, never the UI node's expanded
    # graph environment (old checkpoint, prompt, and Dora session).
    values = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
    return dict(value.decode().split("=", 1) for value in values if b"=" in value)


def find_formal_launcher(*, port: int, current_deployment: str) -> tuple[int, list[str]]:
    pid = os.getpid()
    visited: set[int] = set()
    while pid > 1 and pid not in visited:
        visited.add(pid)
        args = _process_arguments(pid)
        if any(arg in {str(LAUNCHER), "scripts/launch_robot.sh", "./scripts/launch_robot.sh"} for arg in args) and "piper_x_dual" in args:
            cwd = Path(f"/proc/{pid}/cwd").resolve()
            if not cwd.is_relative_to(REPO):
                raise DeploymentSwitchError("formal launcher is outside the personal repository")
            if "--deployment" not in args or args[args.index("--deployment") + 1] != current_deployment:
                raise DeploymentSwitchError("formal launcher deployment does not match the active UI")
            if "--operator-ui-port" not in args or int(args[args.index("--operator-ui-port") + 1]) != port:
                raise DeploymentSwitchError("formal launcher UI port does not match the active UI")
            return pid, _canonical_launcher(args)
        try:
            pid = int(Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[3])
        except (OSError, IndexError, ValueError) as exc:
            raise DeploymentSwitchError("cannot locate the formal launcher ancestor") from exc
    raise DeploymentSwitchError("operator UI was not started by a formal PiPER launcher")


def replacement_command(original: list[str], target: str, execute_steps: int | None = None) -> list[str]:
    """Keep only known live-launch controls; never replay task or run controls."""
    original = _canonical_launcher(original)
    if "piper_x_dual" not in original:
        raise DeploymentSwitchError("unsupported launcher command")
    allowed_flags = {"--enable-write", "--no-write", "--no-model-filter", "--model-filter", "--open-ui", "--no-open-ui"}
    allowed_values = {"--control-source", "--operator-ui-port", "--model-port", "--model-device", "--model-ready-timeout", "--execute-steps"}
    result = ["bash", str(LAUNCHER), "piper_x_dual", "--deployment", target]
    position = original.index("piper_x_dual") + 1
    seen: set[str] = set()
    while position < len(original):
        item = original[position]
        if item == "--deployment":
            position += 2
            continue
        if item in allowed_flags:
            if item in seen:
                raise DeploymentSwitchError("ambiguous formal launcher flag")
            seen.add(item)
            result.append(item)
            position += 1
            continue
        if item in allowed_values:
            if position + 1 >= len(original) or item in seen:
                raise DeploymentSwitchError("ambiguous formal launcher setting")
            seen.add(item)
            result.extend((item, original[position + 1]))
            position += 2
            continue
        raise DeploymentSwitchError(f"unsupported active launcher option: {item}")
    for required in ("--control-source", "--operator-ui-port", "--model-port"):
        if required not in seen:
            raise DeploymentSwitchError(f"formal launcher is missing {required}")
    if not ({"--enable-write", "--no-write"} & seen):
        raise DeploymentSwitchError("formal launcher write mode is ambiguous")
    if not ({"--no-model-filter", "--model-filter"} & seen):
        raise DeploymentSwitchError("formal launcher model-filter mode is ambiguous")
    if not ({"--open-ui", "--no-open-ui"} & seen):
        raise DeploymentSwitchError("formal launcher UI-open mode is ambiguous")
    for pair in ({"--enable-write", "--no-write"}, {"--model-filter", "--no-model-filter"}, {"--open-ui", "--no-open-ui"}):
        if pair <= seen:
            raise DeploymentSwitchError("conflicting formal launcher flags")
    if result[result.index("--control-source") + 1] != "keyboard":
        raise DeploymentSwitchError("deployment switching requires keyboard control")
    if execute_steps is not None:
        if type(execute_steps) is not int or execute_steps not in (10, 15, 20, 25):
            raise DeploymentSwitchError("执行步数必须为 10、15、20、25")
        if "--execute-steps" in result:
            result[result.index("--execute-steps") + 1] = str(execute_steps)
        else:
            result.extend(("--execute-steps", str(execute_steps)))
    return result


def _eligible(snapshot: Mapping[str, Any]) -> None:
    robot = snapshot.get("robot_status", {})
    if robot.get("takeover") or robot.get("recording") or robot.get("boundary_event"):
        raise DeploymentSwitchError("结束接管、录制或当前动作后再重连/切换")
    if snapshot.get("command_pending") or snapshot.get("pigey", {}).get("active"):
        raise DeploymentSwitchError("wait for the current operator command or supervisor to finish")
    if str(snapshot.get("policy_status", {}).get("status", "")).lower() != "paused":
        raise DeploymentSwitchError("pause the policy before switching deployment")
    lifecycle = str(snapshot.get("robot_status", {}).get("lifecycle_state", "")).lower()
    if lifecycle not in {"idle", "prepared", "stopped"}:
        raise DeploymentSwitchError("stop or finish the current rollout before switching deployment")
    if bool(snapshot.get("operator_recording", {}).get("active")) or bool(snapshot.get("human_demo", {}).get("active")):
        raise DeploymentSwitchError("stop the active recording before switching deployment")


class DeploymentSwitcher:
    def __init__(self, *, deployment: str, port: int, catalog: tuple[DeploymentOption, ...]) -> None:
        self.deployment = deployment
        self.port = int(port)
        self.catalog = catalog
        self.options = {item.name: item for item in catalog}
        self.dynamic_catalog = False

    @classmethod
    def from_environment(cls) -> DeploymentSwitcher | None:
        deployment = os.getenv("PIPER_DEPLOYMENT", "").strip()
        try:
            catalog = load_catalog()
        except (OSError, ValueError) as exc:
            print(f"PIPER_DEPLOYMENT_SWITCH_DISABLED {exc}", flush=True)
            return None
        if deployment not in {item.name for item in catalog}:
            return None
        try:
            instance = cls(deployment=deployment, port=int(os.getenv("PIPER_OPERATOR_UI_PORT", "8091")), catalog=catalog)
            instance.dynamic_catalog = True
            return instance
        except ValueError:
            return None

    def snapshot(self, runtime: Mapping[str, Any]) -> dict[str, Any]:
        status = read_status()
        reason = ""
        try:
            self.refresh_catalog()
            _eligible(runtime)
        except (ValueError, OSError) as exc:
            reason = str(exc)
        from XPolicyLab.policy.Pi_05.execution_selection import execution_options
        try:
            steps = list(execution_options(self.deployment))
        except ValueError as exc:
            steps = []
            reason = str(exc)
        return {"enabled": True, "current": self.deployment,
                "execution_options": steps, "execution_steps": runtime.get("execution", {}).get("fixed_steps"),
                "deployments": [{"name": item.name, "label": item.label} for item in self.catalog],
                "eligible": not reason and not self.busy(), "reason": reason, "status": status}

    def refresh_catalog(self) -> None:
        if self.dynamic_catalog:
            catalog = load_catalog()
            self.catalog = catalog
            self.options = {item.name: item for item in catalog}

    def busy(self) -> bool:
        return LOCK_PATH.exists()

    @staticmethod
    def _is_eligible(runtime: Mapping[str, Any]) -> bool:
        try:
            _eligible(runtime)
            return True
        except DeploymentSwitchError:
            return False

    def request(self, target: str, runtime: Mapping[str, Any], execute_steps: int | None = None,
                *, reconnect: bool = False) -> dict[str, Any]:
        _eligible(runtime)
        self.refresh_catalog()
        target = str(target).strip()
        if reconnect and target != self.deployment:
            raise DeploymentSwitchError("硬件重连必须保留当前模型")
        if target not in self.options:
            raise DeploymentSwitchError("deployment is not in the personal switch list")
        current_steps = runtime.get("execution", {}).get("fixed_steps")
        # Preserve the active interval when changing models. Legacy deployments
        # without the release extension continue using the existing fixed path.
        from XPolicyLab.policy.Pi_05.execution_selection import execution_options, validate_execution_steps
        if (execute_steps is None and current_steps is not None
                and execution_options(self.deployment) and execution_options(target)):
            execute_steps = current_steps
        if execute_steps is not None:
            validate_execution_steps(target, execute_steps)
        if not reconnect and target == self.deployment and (execute_steps is None or execute_steps == current_steps):
            raise DeploymentSwitchError("selected deployment is already active")
        if LOCK_PATH.exists():
            raise DeploymentSwitchError("another deployment switch is already in progress")
        launcher_pid, original = find_formal_launcher(port=self.port, current_deployment=self.deployment)
        replacement_command(original, target, execute_steps)
        environment = _launcher_environment(launcher_pid)
        ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
        try:
            descriptor = os.open(LOCK_PATH, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        except FileExistsError as exc:
            raise DeploymentSwitchError("another deployment switch is already in progress") from exc
        os.close(descriptor)
        request_id = f"{int(time.time())}-{os.getpid()}"
        log = ARTIFACT_DIR / f"{request_id}.log"
        try:
            _atomic_json(STATUS_PATH, {"phase": "queued", "request_id": request_id,
                "previous": self.deployment, "target": target, "time_ns": time.time_ns(),
                "operation": "reconnect" if reconnect else "switch"})
            with log.open("xb") as stream:
                subprocess.Popen([sys.executable, "-m", "XPolicyLab.real_world.robots.piper_x_dual.deployment_switch",
                              "--request-id", request_id, "--target", target, "--previous", self.deployment,
                              "--launcher-pid", str(launcher_pid), "--launcher-command", json.dumps(original),
                              "--port", str(self.port),
                              *(["--reconnect"] if reconnect else []),
                              *(["--execute-steps", str(execute_steps)] if execute_steps is not None else [])], cwd=REPO, stdin=subprocess.DEVNULL,
                             stdout=stream, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        except Exception as exc:
            LOCK_PATH.unlink(missing_ok=True)
            _write_status(phase="failed", request_id=request_id, target=target, error=str(exc))
            raise DeploymentSwitchError(f"cannot start deployment switch: {exc}") from exc
        return {"ok": True, "request_id": request_id, "target": target, "log": str(log)}


def _write_status(**value: Any) -> None:
    previous = read_status()
    if previous.get("request_id") == value.get("request_id"):
        value.setdefault("operation", previous.get("operation", "switch"))
    value["time_ns"] = time.time_ns()
    _atomic_json(STATUS_PATH, value)


ENVIRONMENT_KEYS = {
    'PIPER_TRAINING_RTC_ENABLED', 'PIPER_NATIVE_RTC_FUTURE_STEPS',
    'PIPER_NATIVE_RTC_MAX_PREFIX_STEPS', 'PIPER_NATIVE_RTC_WALLCLOCK_DEADLINE',
    'PIPER_NATIVE_TRAJECTORY_PROFILE', 'PIPER_NATIVE_VELOCITY_SEAM_PROFILE',
    'PIPER_NATIVE_ONE_EURO_PROFILE', 'PIPER_NATIVE_JOINT_CLIP_PROFILE',
    'PIPER_NATIVE_PRECOMMIT_PROFILE',
    'PIPER_PROFILE', 'PIPER_MOTION_TRACE_PATH', 'ACTION_TRAJECTORY_PROFILE',
    'ACTION_TRAJECTORY_TIME_SCALE', 'ACTION_RTC_MODE',
    'ACTION_RTC_REQUEST_REMAINING_STEPS', 'ACTION_RTC_TRANSITION_STEPS',
    'ACTION_RTC_PRESERVE_STEPS', 'PIPER_MODEL_FILTER_ENABLED',
}


def configuration_environment(original, settings):
    """Apply only release-declared runtime settings, excluding credentials."""
    if not isinstance(settings, dict) or set(settings) - ENVIRONMENT_KEYS:
        raise DeploymentSwitchError('invalid release switch environment')
    if not all(isinstance(value, str) for value in settings.values()):
        raise DeploymentSwitchError('switch environment values must be strings')
    return {**{k: v for k, v in original.items() if k not in ENVIRONMENT_KEYS}, **settings}


@contextmanager
def _deployment_environment(name):
    from XPolicyLab.policy.Pi_05.piper_deployments import load_deployment
    deployment = load_deployment(name)
    evidence = _read_json(REPO / deployment['release']['evidence'])
    settings = evidence.get('web_switch_environment')
    original = dict(os.environ)
    # Older deployments retain their historical behavior until explicitly reviewed.
    selected = original if settings is None else configuration_environment(original, settings)
    os.environ.clear()
    os.environ.update(selected)
    try:
        yield
    finally:
        os.environ.clear()
        os.environ.update(original)


def _validate_target(target: str) -> None:
    from XPolicyLab.policy.Pi_05.web_deployment_intake import verify_registered
    verify_registered(target)
    result = subprocess.run(["/usr/bin/python3", str(REGISTRY), "check", target], cwd=REPO,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=120)
    if result.returncode:
        raise RuntimeError("formal deployment validation failed: " + result.stdout[-1000:])


def _wait_for_exit(pid: int, timeout: float = 45.0) -> None:
    deadline = time.monotonic() + timeout
    while _process_is_live(pid):
        if time.monotonic() >= deadline:
            raise RuntimeError("previous formal launcher did not exit")
        time.sleep(0.1)


def _start(command: list[str]) -> subprocess.Popen[Any]:
    return subprocess.Popen(command, cwd=REPO, stdin=subprocess.DEVNULL,
                            start_new_session=True)


def _stop_started_launcher(process: subprocess.Popen[Any], timeout: float = 45.0) -> None:
    """Stop a failed replacement before releasing the port to rollback."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while process.poll() is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("failed replacement launcher did not exit")
        time.sleep(0.1)


def _wait_ready(*, target: str, port: int, write_enabled: bool, timeout: float = READY_TIMEOUT_SECONDS,
                execute_steps: int | None = None) -> None:
    deadline = time.monotonic() + timeout
    expected_status = "READY_LIVE_TAKEOVER" if write_enabled else "READY_NO_WRITE"
    while time.monotonic() < deadline:
        try:
            with urlopen(f"http://127.0.0.1:{port}/api/state", timeout=2) as response:
                state = json.loads(response.read().decode("utf-8"))
        except (URLError, OSError, json.JSONDecodeError):
            time.sleep(1.0)
            continue
        cameras = state.get("cameras", {})
        if (state.get("deployment", {}).get("id") == target
                and (execute_steps is None or (
                    state.get("execution", {}).get("fixed_steps") == execute_steps
                    and state.get("policy_status", {}).get("training_time_rtc", {}).get("commit_steps") == execute_steps))
                and state.get("write_enabled", write_enabled) == write_enabled
                and state.get("policy_status", {}).get("status") == "paused"
                and state.get("robot_status", {}).get("status") == expected_status
                and all(
                    bool(cameras.get(name, {}).get("available"))
                    for name in REQUIRED_CAMERAS
                )):
            return
        time.sleep(1.0)
    raise RuntimeError("new deployment did not become ready with the requested identity")


def run_switch(args: argparse.Namespace) -> int:
    reconnect = getattr(args, "reconnect", False)
    if reconnect and args.target != args.previous:
        raise RuntimeError("reconnect cannot change deployment")
    catalog = {item.name for item in load_catalog()}
    if args.target not in catalog or args.previous not in catalog:
        raise RuntimeError("switch target is outside the personal catalogue")
    original = json.loads(args.launcher_command)
    if not isinstance(original, list) or not all(isinstance(item, str) for item in original):
        raise RuntimeError("invalid original launcher command")
    execute_steps = getattr(args, "execute_steps", None)
    replacement = replacement_command(original, args.target, execute_steps)
    from XPolicyLab.policy.Pi_05.execution_selection import execution_options
    if execute_steps is None and not execution_options(args.target) and '--execute-steps' in replacement:
        index = replacement.index('--execute-steps')
        del replacement[index:index + 2]
    rollback = replacement_command(original, args.previous)
    rollback_steps = int(rollback[rollback.index("--execute-steps") + 1]) if "--execute-steps" in rollback else None
    write_enabled = "--enable-write" in replacement
    original_stopped = False
    replacement_process: subprocess.Popen[Any] | None = None
    _write_status(phase="validating", request_id=args.request_id, previous=args.previous, target=args.target)
    try:
        with _deployment_environment(args.target):
            _validate_target(args.target)
        if execute_steps is not None:
            from XPolicyLab.policy.Pi_05.execution_selection import validate_execution_steps
            validate_execution_steps(args.target, execute_steps)
        if _canonical_launcher(_process_arguments(args.launcher_pid)) != original:
            raise RuntimeError("original launcher identity changed")
        with urlopen(f"http://127.0.0.1:{args.port}/api/state", timeout=3) as response:
            current = json.loads(response.read())
        if current.get("deployment", {}).get("id") != args.previous:
            raise RuntimeError("active deployment identity changed")
        _eligible(current)
        _write_status(phase="stopping", request_id=args.request_id, previous=args.previous, target=args.target)
        os.kill(args.launcher_pid, signal.SIGTERM)
        _wait_for_exit(args.launcher_pid)
        original_stopped = True
        _write_status(phase="starting", request_id=args.request_id, previous=args.previous, target=args.target)
        with _deployment_environment(args.target):
            replacement_process = _start(replacement)
        _wait_ready(target=args.target, port=args.port, write_enabled=write_enabled, execute_steps=execute_steps)
        _write_status(phase="ready", request_id=args.request_id, previous=args.previous, target=args.target,
                      launcher_pid=replacement_process.pid, execute_steps=execute_steps, verified=True)
        return 0
    except Exception as exc:
        if reconnect and original_stopped and replacement_process is not None and replacement_process.poll() is None:
            # Retain the paused UI and its component diagnostics. Restarting
            # the same failing hardware again would hide the useful error.
            _write_status(phase="failed", request_id=args.request_id,
                          previous=args.previous, target=args.target,
                          launcher_pid=replacement_process.pid, error=str(exc), verified=False)
            return 1
        if not original_stopped:
            _write_status(
                phase="failed",
                request_id=args.request_id,
                previous=args.previous,
                target=args.target,
                error=str(exc),
                rollback_error="original runtime was not stopped; rollback was not started",
                verified=False,
            )
            return 1
        _write_status(phase="rollback_starting", request_id=args.request_id, previous=args.previous,
                      target=args.target, error=str(exc))
        try:
            if replacement_process is not None:
                _stop_started_launcher(replacement_process)
            child = _start(rollback)
            _wait_ready(target=args.previous, port=args.port, write_enabled=write_enabled, execute_steps=rollback_steps)
            _write_status(phase="rolled_back", request_id=args.request_id, previous=args.previous,
                          target=args.target, launcher_pid=child.pid, error=str(exc), verified=True)
        except Exception as rollback_exc:
            if 'child' in locals():
                _stop_started_launcher(child)
            _write_status(phase="failed", request_id=args.request_id, previous=args.previous, target=args.target,
                          error=str(exc), rollback_error=str(rollback_exc), verified=False)
        return 1
    finally:
        try:
            LOCK_PATH.unlink()
        except FileNotFoundError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--previous", required=True)
    parser.add_argument("--launcher-pid", required=True, type=int)
    parser.add_argument("--launcher-command", required=True)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--execute-steps", type=int, choices=(10, 15, 20, 25))
    parser.add_argument("--reconnect", action="store_true")
    return run_switch(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
