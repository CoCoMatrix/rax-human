"""Validated deployment descriptors for dual-PiPER policy bundles.

The descriptor is the single launcher-facing recipe. Model restore details
remain in the referenced server/deploy adapter; robot and camera identity stay
in the physical profile and instance files.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import secrets
import shlex
import signal
import socket
import subprocess
import sys
import time
from typing import Any, Callable, Mapping
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTOR_ROOT = Path(__file__).resolve().parent / "deployments"
SCHEMA = "xpolicylab.piper-deployment/v1"
PIGEY_ROOT = Path(
    os.environ.get("XPOLICY_PIGEY_ROOT", "/home/ubuntu/code_junqi/pigey_piper")
).expanduser()
PIGEY_PYTHON = Path(
    os.environ.get(
        "XPOLICY_PIGEY_PYTHON",
        "/home/ubuntu/miniconda3/envs/infra/bin/python",
    )
).expanduser()
PIGEY_QWEN_PORT = 8002
PIGEY_OPERATOR_PORT = 8091
PIGEY_QWEN_MODEL = "Qwen/Qwen3-VL-8B-Instruct-FP8"
PIGEY_QWEN_TIMEOUT_SECONDS = 420.0
PIGEY_OPERATOR_TIMEOUT_SECONDS = 420.0
PIGEY_DEMO_RECORD_TIMEOUT_SECONDS = 900.0
PIGEY_OPERATOR_RESTART_TIMEOUT_SECONDS = 30.0
# Direct invocation (``python policy/Pi_05/piper_deployments.py``) otherwise
# exposes only policy/Pi_05 on sys.path, while the qualification module imports
# the repository's XPolicyLab namespace.
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SHELL_FIELDS = (
    "server.default_mode",
    "operator.default_control_source",
    "operator.keyboard_graph",
    "operator.dagger_graph",
    "label",
    "tasks.default",
    "bundle.default_path",
    "policy.action_frequency_hz",
    "policy.action_horizon",
    "policy.default_execute_chunk",
    "policy.default_execute_mode",
    "server.script",
    "operator.ui_enabled",
    "tasks.catalog_path",
    "bundle.kind",
    "bundle.verifier",
    "setup.script",
    "qualification.smoke_script",
    "model.backend",
    "observation.camera_profile",
    "release.status",
    "robot.device_instance",
    "policy.trajectory_profile",
)


class DeploymentError(ValueError):
    pass


@dataclass(frozen=True)
class PigeyRunOptions:
    """Pigey-only options removed before invoking the robot launcher."""

    enabled: bool = False
    demo: Path | None = None
    record_demo: bool = False
    preset_ui: bool = False
    expected_tasks: int | None = None
    check_steps: int | None = None
    reinitialize_steps: int | None = None
    max_task_steps: int | None = None
    # Zero means keep the warm Pigey session alive until Ctrl+C.
    rounds: int = 0
    demo_sample_fps: float = 1.0
    demo_max_frames: int = 40


class _SupervisorSignal(RuntimeError):
    def __init__(self, signum: int):
        self.signum = signum
        super().__init__(f"received signal {signum}")


def parse_pigey_run_arguments(
    arguments: list[str] | tuple[str, ...],
) -> tuple[list[str], PigeyRunOptions]:
    """Split supervisor flags from the existing launch_robot.sh arguments."""

    forwarded: list[str] = []
    enabled: bool | None = None
    demo: Path | None = None
    record_demo = False
    preset_ui = False
    expected_tasks: int | None = None
    check_steps: int | None = None
    reinitialize_steps: int | None = None
    max_task_steps: int | None = None
    rounds = 0
    demo_sample_fps = 1.0
    demo_max_frames = 40
    demo_sampling_option_seen = False
    pigey_option_seen = False
    index = 0
    values = list(arguments)
    while index < len(values):
        option = values[index]
        if option in {"--pigey", "--no-pigey"}:
            selected = option == "--pigey"
            if enabled is not None and enabled != selected:
                raise DeploymentError("--pigey and --no-pigey are mutually exclusive")
            enabled = selected
            index += 1
            continue
        if option == "--pigey-record-demo":
            pigey_option_seen = True
            record_demo = True
            index += 1
            continue
        if option == "--pigey-preset-ui":
            pigey_option_seen = True
            preset_ui = True
            index += 1
            continue
        if option in {
            "--pigey-demo",
            "--pigey-expected-tasks",
            "--pigey-check-steps",
            "--pigey-reinitialize-steps",
            "--pigey-max-task-steps",
            "--pigey-rounds",
            "--pigey-demo-sample-fps",
            "--pigey-demo-max-frames",
        }:
            pigey_option_seen = True
            if index + 1 >= len(values):
                raise DeploymentError(f"{option} requires a value")
            raw = values[index + 1]
            if option == "--pigey-demo":
                demo = Path(raw).expanduser().resolve()
            elif option == "--pigey-demo-sample-fps":
                demo_sampling_option_seen = True
                try:
                    demo_sample_fps = float(raw)
                except ValueError as exc:
                    raise DeploymentError(
                        "--pigey-demo-sample-fps must be a number in (0, 30]"
                    ) from exc
                if not 0 < demo_sample_fps <= 30:
                    raise DeploymentError(
                        "--pigey-demo-sample-fps must be in the range (0, 30]"
                    )
            else:
                try:
                    number = int(raw)
                except ValueError as exc:
                    qualifier = (
                        "a non-negative integer"
                        if option == "--pigey-reinitialize-steps"
                        else "a positive integer"
                    )
                    raise DeploymentError(f"{option} must be {qualifier}") from exc
                if number < 0 or (
                    option not in {"--pigey-reinitialize-steps", "--pigey-rounds"}
                    and number < 1
                ):
                    qualifier = (
                        "a non-negative integer"
                        if option in {"--pigey-reinitialize-steps", "--pigey-rounds"}
                        else "a positive integer"
                    )
                    raise DeploymentError(f"{option} must be {qualifier}")
                if option == "--pigey-check-steps" and number > 10_000:
                    raise DeploymentError(
                        "--pigey-check-steps must be in the range [1, 10000]"
                    )
                if option == "--pigey-reinitialize-steps" and number > 100_000:
                    raise DeploymentError(
                        "--pigey-reinitialize-steps must be in the range [0, 100000]"
                    )
                if option == "--pigey-max-task-steps" and number > 100_000:
                    raise DeploymentError(
                        "--pigey-max-task-steps must be in the range [1, 100000]"
                    )
                if option == "--pigey-rounds" and number > 100:
                    raise DeploymentError(
                        "--pigey-rounds must be in the range [0, 100]"
                    )
                if option == "--pigey-demo-max-frames":
                    demo_sampling_option_seen = True
                    if not 2 <= number <= 40:
                        raise DeploymentError(
                            "--pigey-demo-max-frames must be in the range [2, 40]"
                        )
                    demo_max_frames = number
                elif option == "--pigey-expected-tasks":
                    expected_tasks = number
                elif option == "--pigey-reinitialize-steps":
                    reinitialize_steps = number
                elif option == "--pigey-max-task-steps":
                    max_task_steps = number
                elif option == "--pigey-rounds":
                    rounds = number
                else:
                    check_steps = number
            index += 2
            continue
        forwarded.append(option)
        index += 1

    if pigey_option_seen and enabled is not True:
        raise DeploymentError("Pigey-specific options require --pigey")
    options = PigeyRunOptions(
        enabled=enabled is True,
        demo=demo,
        record_demo=record_demo,
        preset_ui=preset_ui,
        expected_tasks=expected_tasks,
        check_steps=check_steps,
        reinitialize_steps=reinitialize_steps,
        max_task_steps=max_task_steps,
        rounds=rounds,
        demo_sample_fps=demo_sample_fps,
        demo_max_frames=demo_max_frames,
    )
    if not options.enabled:
        return forwarded, options
    selected_inputs = sum(
        (options.demo is not None, options.record_demo, options.preset_ui)
    )
    if selected_inputs > 1:
        raise DeploymentError(
            "--pigey-demo, --pigey-record-demo, and --pigey-preset-ui are "
            "mutually exclusive"
        )
    has_demo_input = options.demo is not None or options.record_demo
    if not has_demo_input and options.expected_tasks is not None:
        raise DeploymentError(
            "--pigey-expected-tasks requires --pigey-demo or --pigey-record-demo"
        )
    if not has_demo_input and demo_sampling_option_seen:
        raise DeploymentError(
            "Pigey demo sampling options require --pigey-demo or "
            "--pigey-record-demo"
        )
    return forwarded, options


def _last_argument(arguments: list[str], option: str) -> str | None:
    result: str | None = None
    for index, value in enumerate(arguments[:-1]):
        if value == option:
            result = arguments[index + 1]
    return result


def _last_boolean_option(
    arguments: list[str], positive: str, negative: str, *, default: bool
) -> bool:
    result = default
    for value in arguments:
        if value == positive:
            result = True
        elif value == negative:
            result = False
    return result


def _pigey_artifact_dir() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return PIGEY_ROOT / "artifacts" / "real" / f"pigey-{stamp}"


def _pigey_harness_config(deployment: Mapping[str, Any]) -> Path:
    raw = str(_lookup(deployment, "pigey.harness_config") or "").strip()
    if not raw:
        raw = "configs/real/harness_xpolicy_local_qwen.json"
    path = Path(raw).expanduser()
    return path.resolve() if path.is_absolute() else (PIGEY_ROOT / path).resolve()


def _transition_pose_library(
    deployment: Mapping[str, Any],
) -> tuple[Path, str] | None:
    raw_path = str(
        _lookup(deployment, "pigey.transition_pose_library.path") or ""
    ).strip()
    raw_digest = str(
        _lookup(deployment, "pigey.transition_pose_library.sha256") or ""
    ).strip().lower()
    if not raw_path and not raw_digest:
        return None
    if not raw_path or len(raw_digest) != 64 or any(
        character not in "0123456789abcdef" for character in raw_digest
    ):
        raise DeploymentError(
            "pigey.transition_pose_library requires path and SHA-256"
        )
    path = Path(raw_path).expanduser()
    path = path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()
    if path.is_file():
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != raw_digest:
            raise DeploymentError(
                "Pigey transition-pose library does not match its descriptor SHA-256"
            )
    return path, raw_digest


def _continuous_policy_prompts(
    deployment: Mapping[str, Any],
) -> tuple[str, ...]:
    raw = _lookup(deployment, "pigey.continuous_policy_prompts")
    if not raw:
        return ()
    if not isinstance(raw, list) or not all(
        isinstance(item, str) and item.strip() for item in raw
    ):
        raise DeploymentError(
            "pigey.continuous_policy_prompts must contain non-empty strings"
        )
    prompts = tuple(item.strip() for item in raw)
    configured = tuple(_lookup(deployment, "tasks.inline") or ())
    if any(prompt not in configured for prompt in prompts):
        raise DeploymentError(
            "continuous Pigey prompts must belong to the deployment task catalog"
        )
    return prompts


def _default_reinitialize_steps(deployment: Mapping[str, Any]) -> int:
    raw = _lookup(deployment, "pigey.reinitialize_after_steps")
    if raw in ("", None):
        return 0
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise DeploymentError(
            "pigey.reinitialize_after_steps must be an integer"
        ) from exc
    if not 0 <= value <= 100_000:
        raise DeploymentError(
            "pigey.reinitialize_after_steps must be in [0, 100000]"
        )
    return value


def _effective_check_steps(
    deployment: Mapping[str, Any], options: PigeyRunOptions
) -> int:
    raw = (
        options.check_steps
        if options.check_steps is not None
        else _lookup(deployment, "pigey.check_steps")
    )
    if raw in ("", None):
        return 300
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise DeploymentError("pigey.check_steps must be an integer") from exc
    if not 1 <= value <= 10_000:
        raise DeploymentError("pigey.check_steps must be in [1, 10000]")
    return value


def _default_max_task_steps(deployment: Mapping[str, Any]) -> int:
    raw = _lookup(deployment, "pigey.max_task_control_steps")
    if raw in ("", None):
        return 0
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise DeploymentError(
            "pigey.max_task_control_steps must be an integer"
        ) from exc
    if not 0 <= value <= 100_000:
        raise DeploymentError(
            "pigey.max_task_control_steps must be in [0, 100000]"
        )
    return value


def _pigey_initialization_mode(deployment: Mapping[str, Any]) -> str:
    value = str(
        _lookup(deployment, "pigey.initialization_mode") or "dataset_pose"
    ).strip()
    if value not in {"dataset_pose", "profile_home"}:
        raise DeploymentError(
            "pigey.initialization_mode must be dataset_pose or profile_home"
        )
    return value


def _pigey_task_transition_mode(deployment: Mapping[str, Any]) -> str:
    value = str(
        _lookup(deployment, "pigey.task_transition_mode") or "initialize"
    ).strip()
    if value not in {
        "initialize",
        "hold_current",
        "dataset_pose_or_hold_current",
    }:
        raise DeploymentError(
            "pigey.task_transition_mode must be initialize, hold_current, or "
            "dataset_pose_or_hold_current"
        )
    return value


def _pigey_task_step_limit_mode(deployment: Mapping[str, Any]) -> str:
    value = str(
        _lookup(deployment, "pigey.task_step_limit_mode")
        or "require_vlm_done"
    ).strip()
    if value not in {"require_vlm_done", "force_done"}:
        raise DeploymentError(
            "pigey.task_step_limit_mode must be require_vlm_done or force_done"
        )
    return value


def _pigey_sequence_presets(
    deployment: Mapping[str, Any],
) -> tuple[dict[str, Any], ...]:
    raw = _lookup(deployment, "pigey.sequence_presets")
    if raw in (None, ""):
        return ()
    if not isinstance(raw, list) or not raw:
        raise DeploymentError("pigey.sequence_presets must be a non-empty list")
    allowed_prompts = set(_lookup(deployment, "tasks.inline") or ())
    presets: list[dict[str, Any]] = []
    ids: set[str] = set()
    sequences: set[tuple[str, ...]] = set()
    for row in raw:
        if not isinstance(row, Mapping) or set(row) != {"id", "label", "prompts"}:
            raise DeploymentError(
                "each Pigey sequence preset requires only id, label, and prompts"
            )
        preset_id = str(row["id"]).strip()
        label = str(row["label"]).strip()
        prompts = row["prompts"]
        if (
            not preset_id
            or len(preset_id) > 64
            or any(not (char.isalnum() or char in {"_", "-"}) for char in preset_id)
        ):
            raise DeploymentError("Pigey sequence preset id is invalid")
        if preset_id in ids:
            raise DeploymentError("Pigey sequence preset ids must be unique")
        if not label or len(label) > 160:
            raise DeploymentError("Pigey sequence preset label is invalid")
        if (
            not isinstance(prompts, list)
            or not prompts
            or any(not isinstance(prompt, str) or not prompt.strip() for prompt in prompts)
        ):
            raise DeploymentError("Pigey sequence preset prompts must be strings")
        normalized = tuple(prompt.strip() for prompt in prompts)
        if len(normalized) != len(set(normalized)):
            raise DeploymentError("Pigey sequence preset prompts must be unique")
        if any(prompt not in allowed_prompts for prompt in normalized):
            raise DeploymentError(
                "Pigey sequence preset contains a prompt outside tasks.inline"
            )
        if normalized in sequences:
            raise DeploymentError("Pigey sequence preset sequences must be unique")
        ids.add(preset_id)
        sequences.add(normalized)
        presets.append(
            {"id": preset_id, "label": label, "prompts": list(normalized)}
        )
    return tuple(presets)


def build_pigey_command(
    deployment: Mapping[str, Any],
    launch_arguments: list[str],
    options: PigeyRunOptions,
    artifacts: Path,
) -> list[str]:
    """Build the Pigey command without starting robot or model processes."""

    task = _last_argument(launch_arguments, "--task") or str(
        _lookup(deployment, "tasks.default")
    )
    check_steps = _effective_check_steps(deployment, options)
    command = [
        str(PIGEY_PYTHON),
        str(PIGEY_ROOT / "scripts/run_piper_fsm.py"),
        "--robot-config",
        str(PIGEY_ROOT / "configs/real/xpolicy_dual_piper_site.json"),
        "--harness-config",
        str(_pigey_harness_config(deployment)),
        "--artifacts",
        str(artifacts),
        "--max-control-steps",
        str(check_steps),
        "--execute",
        "--policy-deployment",
        str(deployment["name"]),
        "--policy-checkpoint",
        str(_lookup(deployment, "bundle.default_path")),
        "--policy-control-hz",
        str(_lookup(deployment, "policy.action_frequency_hz")),
        "--policy-action-chunk",
        str(_lookup(deployment, "policy.action_horizon")),
        "--policy-execute-chunk",
        str(_lookup(deployment, "policy.default_execute_chunk")),
        "--initialization-mode",
        _pigey_initialization_mode(deployment),
        "--task-transition-mode",
        _pigey_task_transition_mode(deployment),
        "--task-step-limit-mode",
        _pigey_task_step_limit_mode(deployment),
    ]
    transition_library = _transition_pose_library(deployment)
    if transition_library is not None:
        command.extend(
            (
                "--transition-pose-library",
                str(transition_library[0]),
                "--transition-pose-library-sha256",
                transition_library[1],
            )
        )
    configured_tasks = tuple(_lookup(deployment, "tasks.inline") or ())
    for prompt in configured_tasks:
        command.extend(("--policy-prompt", str(prompt)))
    for prompt in _continuous_policy_prompts(deployment):
        command.extend(("--continuous-policy-prompt", prompt))
    reinitialize_steps = (
        options.reinitialize_steps
        if options.reinitialize_steps is not None
        else _default_reinitialize_steps(deployment)
    )
    command.extend(("--reinitialize-after-steps", str(reinitialize_steps)))
    max_task_steps = (
        options.max_task_steps
        if options.max_task_steps is not None
        else _default_max_task_steps(deployment)
    )
    if max_task_steps > 0:
        command.extend(("--max-task-control-steps", str(max_task_steps)))
    command.extend(("--rounds", str(options.rounds)))
    if options.preset_ui:
        presets = _pigey_sequence_presets(deployment)
        if not presets:
            raise DeploymentError(
                "--pigey-preset-ui requires deployment Pigey sequence presets"
            )
        command.extend(
            (
                "--sequence-presets-json",
                json.dumps(presets, separators=(",", ":")),
            )
        )
    elif options.demo is None:
        command.extend(("--task", task))
    else:
        command.extend(
            (
                "--demo",
                str(options.demo),
                "--demo-sample-fps",
                str(options.demo_sample_fps),
                "--demo-max-frames",
                str(options.demo_max_frames),
            )
        )
        if options.expected_tasks is not None:
            command.extend(("--expected-rule-count", str(options.expected_tasks)))
    return command


def build_pigey_validation_command(pigey_command: list[str]) -> list[str]:
    """Convert the live supervisor command into a hardware-free input check."""

    command = [value for value in pigey_command if value != "--execute"]
    command.append("--validate-input-only")
    return command


def _qwen_command() -> list[str]:
    return [
        str(PIGEY_PYTHON),
        str(PIGEY_ROOT / "scripts/run_local_qwen3_vl.py"),
        "--port",
        str(PIGEY_QWEN_PORT),
    ]


def _load_secret_environment(path: Path) -> dict[str, str]:
    """Parse a small shell-style export file without executing it."""

    if not path.is_file():
        raise DeploymentError(f"Pigey VLM secret file is missing: {path}")
    result: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        name, separator, raw_value = line.partition("=")
        if not separator or not name.isidentifier():
            raise DeploymentError(
                f"invalid Pigey VLM secret entry on line {line_number}"
            )
        values = shlex.split(raw_value, comments=False, posix=True)
        if len(values) != 1:
            raise DeploymentError(
                f"invalid Pigey VLM secret value on line {line_number}"
            )
        result[name] = values[0]
    return result


def _remote_vlm_environment(deployment: Mapping[str, Any]) -> dict[str, str]:
    raw_secret_path = str(
        _lookup(deployment, "pigey.vlm.secret_file") or ""
    ).strip()
    if not raw_secret_path:
        raise DeploymentError("remote Pigey VLM requires pigey.vlm.secret_file")
    secret_path = Path(raw_secret_path).expanduser()
    environment = _load_secret_environment(secret_path)
    base_url = str(_lookup(deployment, "pigey.vlm.base_url") or "").strip()
    live_model = str(_lookup(deployment, "pigey.vlm.live_model") or "").strip()
    demo_model = str(_lookup(deployment, "pigey.vlm.demo_model") or "").strip()
    if not base_url or not live_model or not demo_model:
        raise DeploymentError(
            "remote Pigey VLM requires base_url, live_model, and demo_model"
        )
    environment.update(
        {
            "VLM_BASE_URL": base_url.rstrip("/"),
            "VLM_LIVE_MODEL": live_model,
            "VLM_DEMO_MODEL": demo_model,
        }
    )
    if not any(
        environment.get(name)
        for name in (
            "DASHSCOPE_API_KEY",
            "QWEN_API_KEY",
            "VLM_API_KEY",
            "OPENAI_API_KEY",
        )
    ):
        raise DeploymentError("remote Pigey VLM secret file has no API key")
    return environment


def _remote_qwen_ready(environment: Mapping[str, str]) -> bool:
    base_url = str(environment.get("VLM_BASE_URL", "")).rstrip("/")
    key = next(
        (
            environment.get(name, "")
            for name in (
                "DASHSCOPE_API_KEY",
                "QWEN_API_KEY",
                "VLM_API_KEY",
                "OPENAI_API_KEY",
            )
            if environment.get(name)
        ),
        "",
    )
    required = {
        str(environment.get("VLM_LIVE_MODEL", "")),
        str(environment.get("VLM_DEMO_MODEL", "")),
    }
    if not base_url or not key or "" in required:
        return False
    request = Request(
        f"{base_url}/models",
        headers={"Authorization": f"Bearer {key}"},
    )
    try:
        with urlopen(request, timeout=10.0) as response:
            value = json.loads(response.read().decode("utf-8"))
    except (
        HTTPError,
        URLError,
        OSError,
        UnicodeDecodeError,
        json.JSONDecodeError,
    ):
        return False
    available = {
        str(model.get("id", ""))
        for model in value.get("data", ())
        if isinstance(model, dict)
    }
    return required.issubset(available)


def _graph_command(deployment: Mapping[str, Any], arguments: list[str]) -> list[str]:
    return [
        "bash",
        str(REPO_ROOT / "scripts/launch_robot.sh"),
        "piper_x_dual",
        "--deployment",
        str(deployment["name"]),
        *arguments,
    ]


def _replace_boolean_option(
    arguments: list[str], positive: str, negative: str, *, selected: bool
) -> list[str]:
    """Return arguments with one unambiguous final boolean selection."""

    return [
        value for value in arguments if value not in {positive, negative}
    ] + [positive if selected else negative]


def _capture_graph_arguments(arguments: list[str]) -> list[str]:
    """Force the human-demonstration phase to be physically read-only."""

    return _replace_boolean_option(
        arguments, "--enable-write", "--no-write", selected=False
    )


def _live_graph_arguments(arguments: list[str], *, recording_mode: bool) -> list[str]:
    """Reuse the capture browser tab instead of opening a duplicate live tab."""

    if not recording_mode:
        return list(arguments)
    return _replace_boolean_option(
        arguments, "--open-ui", "--no-open-ui", selected=False
    )


def _tcp_port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.25):
            return True
    except OSError:
        return False


def _qwen_ready() -> bool:
    try:
        with urlopen(
            f"http://127.0.0.1:{PIGEY_QWEN_PORT}/v1/models", timeout=1.0
        ) as response:
            value = json.loads(response.read().decode("utf-8"))
    except (HTTPError, URLError, OSError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    models = value.get("data", ()) if isinstance(value, dict) else ()
    return any(
        isinstance(model, dict) and str(model.get("id", "")) == PIGEY_QWEN_MODEL
        for model in models
    )


def _operator_state() -> dict[str, Any] | None:
    try:
        with urlopen(
            f"http://127.0.0.1:{PIGEY_OPERATOR_PORT}/api/state", timeout=1.0
        ) as response:
            value = json.loads(response.read().decode("utf-8"))
    except (HTTPError, URLError, OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict):
        return None
    return value


def _operator_ready() -> bool:
    value = _operator_state()
    if value is None:
        return False
    robot = value.get("robot_status", {})
    cameras = value.get("cameras", {})
    robot_state = value.get("robot_state", {})
    policy = value.get("policy_status", {})
    status = str(robot.get("status", "")) if isinstance(robot, dict) else ""
    lifecycle = (
        str(robot.get("lifecycle_state", "")).lower()
        if isinstance(robot, dict)
        else ""
    )
    camera_ready = isinstance(cameras, dict) and all(
        isinstance(cameras.get(name), dict) and cameras[name].get("available")
        for name in ("left", "head", "right")
    )
    state_ready = (
        isinstance(robot_state, dict)
        and robot_state.get("available") is True
        and robot_state.get("schema_id") == "xpolicy.piper_x_dual.state.v1"
        and len(robot_state.get("values", ())) == 14
        and len(robot_state.get("names", ())) == 14
    )
    policy_status = (
        str(policy.get("status", "")).lower() if isinstance(policy, dict) else "error"
    )
    return bool(
        status.startswith("READY_")
        and lifecycle == "idle"
        and value.get("write_enabled") is True
        and camera_ready
        and state_ready
        and policy_status in {"paused", "ready"}
    )


def _operator_capture_ready() -> bool:
    """Require the top-camera UI while proving robot writes are disabled."""

    value = _operator_state()
    if value is None:
        return False
    robot = value.get("robot_status", {})
    cameras = value.get("cameras", {})
    pigey = value.get("pigey", {})
    return bool(
        isinstance(robot, dict)
        and str(robot.get("status", "")).startswith("READY_")
        and str(robot.get("lifecycle_state", "")).lower() == "idle"
        and value.get("write_enabled") is False
        and isinstance(cameras, dict)
        and isinstance(cameras.get("head"), dict)
        and cameras["head"].get("available") is True
        and isinstance(pigey, dict)
        and str(pigey.get("phase", "")).upper() == "WAITING_FOR_HUMAN_DEMO"
    )


def _operator_recorded_demo_path() -> Path | None:
    """Return the UI-finalized Pigey input, or fail on recorder errors."""

    value = _operator_state()
    if value is None:
        return None
    demo = value.get("human_demo", {})
    if not isinstance(demo, dict):
        return None
    status = str(demo.get("status", "")).lower()
    if status == "error":
        raise RuntimeError(
            "human-demo recording failed: "
            + str(demo.get("message", "unknown recorder error"))
        )
    if status != "ready":
        return None
    raw_path = str(demo.get("pigey_input_path", "")).strip()
    if not raw_path:
        raise RuntimeError("human-demo recorder reported ready without an input path")
    path = Path(raw_path).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"human-demo recorder output is missing: {path}")
    return path


def _wait_for(
    predicate: Callable[[], bool],
    *,
    timeout: float,
    description: str,
    children: tuple[tuple[str, subprocess.Popen[Any]], ...],
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        for name, child in children:
            status = child.poll()
            if status is not None:
                raise RuntimeError(
                    f"{name} exited with status {status} while waiting for {description}"
                )
        time.sleep(0.25)
    raise RuntimeError(f"timed out waiting for {description} after {int(timeout)}s")


def _stop_process_group(child: subprocess.Popen[Any], name: str) -> None:
    if child.poll() is not None:
        return
    print(f"Pigey launcher: stopping owned {name} (PID {child.pid})", flush=True)
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        # Pigey first sends an authenticated stop and waits for XPolicy's idle
        # acknowledgement. Give that safety unwind longer than model services.
        child.wait(timeout=30 if name == "Pigey" else 10)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    child.wait(timeout=5)


def _print_pigey_plan(
    validation_command: list[str],
    qwen_command: list[str] | None,
    graph_command: list[str],
    pigey_command: list[str],
    qwen_log: Path,
) -> None:
    print(f"VALIDATE: {shlex.join(validation_command)}")
    if qwen_command is None:
        print("QWEN: authenticated remote provider readiness check (credential redacted)")
    else:
        print(
            f"QWEN: {shlex.join(qwen_command)} > "
            f"{shlex.quote(str(qwen_log))} 2>&1"
        )
    print(f"XPOLICY: {shlex.join(graph_command)}")
    print(
        "PIGEY: "
        + shlex.join(["env", "PIPER_OPERATOR_ARMED=YES", *pigey_command])
    )


def _print_pigey_record_plan(
    qwen_command: list[str] | None,
    capture_graph_command: list[str],
    live_graph_command: list[str],
    check_steps: int,
    qwen_log: Path,
) -> None:
    """Describe the UI-triggered handoff without inventing an output path."""

    if qwen_command is None:
        print("QWEN: authenticated remote provider readiness check (credential redacted)")
    else:
        print(
            f"QWEN: {shlex.join(qwen_command)} > "
            f"{shlex.quote(str(qwen_log))} 2>&1"
        )
    print(
        "CAPTURE_XPOLICY: "
        + shlex.join(
            [
                "env",
                "PIPER_PIGEY_EXPECTED=1",
                "PIPER_PIGEY_RECORD_DEMO=1",
                *capture_graph_command,
            ]
        )
    )
    print(
        "WAIT_FOR_UI_DEMO: click Start recording, demonstrate the task sequence, "
        "then click Stop & save"
    )
    print(
        "VALIDATE_RECORDED_DEMO: Pigey validates the UI-produced MP4/manifest "
        "without robot access"
    )
    print(f"LIVE_XPOLICY: {shlex.join(live_graph_command)}")
    print(
        "PIGEY: infer the UI-recorded sequence and execute it with "
        f"{check_steps} control steps per VLM check"
    )


def run_with_pigey(
    deployment: Mapping[str, Any],
    launch_arguments: list[str],
    options: PigeyRunOptions,
) -> int:
    """Own Qwen, XPolicy, and Pigey as one fail-closed process tree."""

    pigey_contract = deployment.get("pigey")
    if not isinstance(pigey_contract, Mapping) or pigey_contract.get(
        "supported"
    ) is not True:
        raise DeploymentError(
            "--pigey requires an explicit supported Pigey deployment contract"
        )
    requested_mode = (
        "record_demo"
        if options.record_demo
        else "preset_ui"
        if options.preset_ui
        else "demo"
        if options.demo is not None
        else "fixed_task"
    )
    effective_check_steps = _effective_check_steps(deployment, options)
    supported_modes = pigey_contract.get(
        "supported_modes", ["fixed_task", "demo", "record_demo"]
    )
    if (
        not isinstance(supported_modes, list)
        or not supported_modes
        or any(
            mode not in {"fixed_task", "demo", "record_demo", "preset_ui"}
            for mode in supported_modes
        )
        or len(supported_modes) != len(set(supported_modes))
    ):
        raise DeploymentError(
            "pigey.supported_modes must be a unique non-empty subset of "
            "fixed_task, demo, record_demo, preset_ui"
        )
    if requested_mode not in supported_modes:
        raise DeploymentError(
            f"{deployment['name']} Pigey supports only: "
            + ", ".join(supported_modes)
        )
    transition_library = _transition_pose_library(deployment)
    transition_environment = (
        {
            "PIPER_TRANSITION_POSE_LIBRARY_PATH": str(transition_library[0]),
            "PIPER_TRANSITION_POSE_LIBRARY_SHA256": transition_library[1],
        }
        if transition_library is not None
        else {}
    )
    sequence_environment = (
        {
            "PIPER_PIGEY_SEQUENCE_PRESETS_JSON": json.dumps(
                _pigey_sequence_presets(deployment), separators=(",", ":")
            )
        }
        if options.preset_ui
        else {}
    )
    raw_operator_port = (
        _last_argument(launch_arguments, "--operator-ui-port")
        or str(PIGEY_OPERATOR_PORT)
    )
    try:
        operator_port = int(raw_operator_port)
    except ValueError as exc:
        raise DeploymentError("--operator-ui-port must be an integer") from exc
    if operator_port != PIGEY_OPERATOR_PORT:
        raise DeploymentError(
            f"--pigey currently requires --operator-ui-port {PIGEY_OPERATOR_PORT}"
        )
    configured_tasks = _lookup(deployment, "tasks.inline")
    if (
        options.expected_tasks is not None
        and isinstance(configured_tasks, list)
        and options.expected_tasks > len(configured_tasks)
    ):
        raise DeploymentError(
            "--pigey-expected-tasks cannot exceed the configured "
            f"{len(configured_tasks)}-task catalog"
        )
    dry_run = "--dry-run" in launch_arguments
    if not dry_run and not _last_boolean_option(
        launch_arguments, "--enable-write", "--no-write", default=False
    ):
        raise DeploymentError("live --pigey execution requires --enable-write")

    artifacts = _pigey_artifact_dir()
    qwen_log = artifacts / "qwen.log"
    vlm_mode = str(_lookup(deployment, "pigey.vlm.mode") or "local").strip()
    if vlm_mode not in {"local", "remote"}:
        raise DeploymentError("pigey.vlm.mode must be local or remote")
    qwen_command = _qwen_command() if vlm_mode == "local" else None
    remote_environment: dict[str, str] = {}
    graph_command = _graph_command(
        deployment,
        _live_graph_arguments(launch_arguments, recording_mode=options.record_demo),
    )
    capture_graph_command = _graph_command(
        deployment, _capture_graph_arguments(launch_arguments)
    )
    pigey_command: list[str] | None = None
    validation_command: list[str] | None = None
    if not options.record_demo:
        pigey_command = build_pigey_command(
            deployment, launch_arguments, options, artifacts
        )
        validation_command = build_pigey_validation_command(pigey_command)
    if dry_run:
        if options.record_demo:
            _print_pigey_record_plan(
                qwen_command,
                capture_graph_command,
                graph_command,
                effective_check_steps,
                qwen_log,
            )
        else:
            assert validation_command is not None and pigey_command is not None
            _print_pigey_plan(
                validation_command,
                qwen_command,
                graph_command,
                pigey_command,
                qwen_log,
            )
        return 0

    required = [
        PIGEY_PYTHON,
        PIGEY_ROOT / "scripts/run_piper_fsm.py",
        PIGEY_ROOT / "configs/real/xpolicy_dual_piper_site.json",
        _pigey_harness_config(deployment),
    ]
    if transition_library is not None:
        required.append(transition_library[0])
    if vlm_mode == "local":
        required.append(PIGEY_ROOT / "scripts/run_local_qwen3_vl.py")
    missing = [str(path) for path in required if not path.is_file()]
    if options.demo is not None and not options.demo.exists():
        missing.append(str(options.demo))
    if missing:
        raise DeploymentError(f"Pigey input is missing: {', '.join(missing)}")

    if vlm_mode == "remote":
        remote_environment = _remote_vlm_environment(deployment)

    if validation_command is not None:
        print("Pigey launcher: validating task/demo input before startup", flush=True)
        validation = subprocess.run(
            validation_command,
            cwd=PIGEY_ROOT,
            check=False,
        )
        if validation.returncode:
            raise DeploymentError(
                "Pigey input validation failed with status "
                f"{validation.returncode}; Qwen and XPolicy were not started"
            )

    if _tcp_port_open("127.0.0.1", PIGEY_OPERATOR_PORT):
        raise DeploymentError(
            f"operator port {PIGEY_OPERATOR_PORT} is already occupied; stop the "
            "existing XPolicy graph before using the combined --pigey launcher"
        )

    artifacts.mkdir(parents=True, exist_ok=False)
    owned: list[tuple[str, subprocess.Popen[Any]]] = []
    qwen_log_handle: Any = None
    previous_handlers: dict[int, Any] = {}
    # The record-demo flow replaces the read-only capture graph with the live
    # graph while deliberately reusing the same browser tab.  Keep one random
    # CSRF token for both operator-server instances so that tab does not retain
    # an invalid capture-phase token when live Pigey starts.
    operator_ui_token = secrets.token_urlsafe(32)

    def handle_signal(signum: int, _frame: Any) -> None:
        raise _SupervisorSignal(signum)

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, handle_signal)

        if vlm_mode == "remote":
            if not _remote_qwen_ready(remote_environment):
                raise RuntimeError(
                    "remote Qwen endpoint is unavailable or does not advertise "
                    "both configured Pigey models"
                )
            print(
                "Pigey launcher: authenticated remote Qwen ready "
                f"(live={remote_environment['VLM_LIVE_MODEL']}, "
                f"demo={remote_environment['VLM_DEMO_MODEL']})",
                flush=True,
            )
        elif _qwen_ready():
            print(
                f"Pigey launcher: reusing qualified Qwen at 127.0.0.1:{PIGEY_QWEN_PORT}",
                flush=True,
            )
        else:
            if _tcp_port_open("127.0.0.1", PIGEY_QWEN_PORT):
                raise RuntimeError(
                    f"port {PIGEY_QWEN_PORT} is occupied by a service that is not "
                    f"{PIGEY_QWEN_MODEL}; refusing to replace it"
                )
            qwen_log_handle = qwen_log.open("wb")
            assert qwen_command is not None
            qwen = subprocess.Popen(
                qwen_command,
                cwd=PIGEY_ROOT,
                stdout=qwen_log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            owned.append(("Qwen", qwen))
            print(
                f"Pigey launcher: starting owned Qwen (PID {qwen.pid}); log: {qwen_log}",
                flush=True,
            )
            _wait_for(
                _qwen_ready,
                timeout=PIGEY_QWEN_TIMEOUT_SECONDS,
                description="local Qwen /v1/models readiness",
                children=tuple(owned),
            )

        if options.record_demo:
            capture = subprocess.Popen(
                capture_graph_command,
                cwd=REPO_ROOT,
                env={
                    **os.environ,
                    "PIPER_PIGEY_EXPECTED": "1",
                    "PIPER_PIGEY_RECORD_DEMO": "1",
                    "PIPER_OPERATOR_UI_TOKEN": operator_ui_token,
                },
                start_new_session=True,
            )
            owned.append(("XPolicy capture", capture))
            print(
                "Pigey launcher: starting read-only top-camera capture graph "
                f"(PID {capture.pid})",
                flush=True,
            )
            _wait_for(
                _operator_capture_ready,
                timeout=PIGEY_OPERATOR_TIMEOUT_SECONDS,
                description="read-only human-demo recorder and head camera",
                children=tuple(owned),
            )
            print(
                "Pigey launcher: recording UI ready at "
                f"http://127.0.0.1:{PIGEY_OPERATOR_PORT}; click Start recording, "
                "demonstrate the ordered tasks, then click Stop & save",
                flush=True,
            )
            recorded_demo: Path | None = None

            def demo_is_ready() -> bool:
                nonlocal recorded_demo
                recorded_demo = _operator_recorded_demo_path()
                return recorded_demo is not None

            _wait_for(
                demo_is_ready,
                timeout=PIGEY_DEMO_RECORD_TIMEOUT_SECONDS,
                description="operator Stop & save for the human demonstration",
                children=tuple(owned),
            )
            assert recorded_demo is not None
            print(
                f"Pigey launcher: received finalized UI demo: {recorded_demo}",
                flush=True,
            )
            recorded_options = replace(
                options, demo=recorded_demo, record_demo=False
            )
            pigey_command = build_pigey_command(
                deployment, launch_arguments, recorded_options, artifacts
            )
            validation_command = build_pigey_validation_command(pigey_command)
            print(
                "Pigey launcher: validating the recorded demo before enabling writes",
                flush=True,
            )
            validation = subprocess.run(
                validation_command,
                cwd=PIGEY_ROOT,
                check=False,
            )
            if validation.returncode:
                raise DeploymentError(
                    "recorded Pigey demo validation failed with status "
                    f"{validation.returncode}; live robot control was not started"
                )
            _stop_process_group(capture, "XPolicy capture")
            owned = [item for item in owned if item[1] is not capture]
            _wait_for(
                lambda: not _tcp_port_open("127.0.0.1", PIGEY_OPERATOR_PORT),
                timeout=PIGEY_OPERATOR_RESTART_TIMEOUT_SECONDS,
                description="capture operator API shutdown",
                children=tuple(owned),
            )

        assert pigey_command is not None
        graph = subprocess.Popen(
            graph_command,
            cwd=REPO_ROOT,
            env={
                **os.environ,
                **transition_environment,
                **sequence_environment,
                "PIPER_PIGEY_EXPECTED": "1",
                "PIPER_PIGEY_RECORD_DEMO": "0",
                "PIPER_OPERATOR_UI_TOKEN": operator_ui_token,
            },
            start_new_session=True,
        )
        owned.append(("XPolicy", graph))
        print(
            f"Pigey launcher: starting authoritative XPolicy graph (PID {graph.pid})",
            flush=True,
        )
        _wait_for(
            _operator_ready,
            timeout=PIGEY_OPERATOR_TIMEOUT_SECONDS,
            description="READY XPolicy operator API and head camera",
            children=tuple(owned),
        )

        pigey_environment = {
            **os.environ,
            **remote_environment,
            "PIPER_OPERATOR_ARMED": "YES",
        }
        pigey = subprocess.Popen(
            pigey_command,
            cwd=PIGEY_ROOT,
            env=pigey_environment,
            start_new_session=True,
        )
        owned.append(("Pigey", pigey))
        mode = (
            "recorded demo sequence"
            if options.record_demo
            else "operator-selected preset sequence"
            if options.preset_ui
            else "demo sequence"
            if options.demo is not None
            else "fixed task"
        )
        print(
            "Pigey launcher: supervisor active "
            f"({mode}, {effective_check_steps} control steps per VLM check)",
            flush=True,
        )
        next_remote_readiness_check = time.monotonic() + 30.0
        while True:
            for name, child in owned:
                status = child.poll()
                if status is None:
                    continue
                if name == "Pigey":
                    print(
                        f"Pigey launcher: Pigey exited with status {status}",
                        flush=True,
                    )
                    return int(status)
                print(
                    f"Pigey launcher: owned {name} exited with status {status}",
                    file=sys.stderr,
                    flush=True,
                )
                return int(status) if status else 1
            if vlm_mode == "remote":
                now = time.monotonic()
                if now >= next_remote_readiness_check:
                    if not _remote_qwen_ready(remote_environment):
                        raise RuntimeError("remote Qwen endpoint became unavailable")
                    next_remote_readiness_check = now + 30.0
            elif not _qwen_ready():
                raise RuntimeError("reused Qwen server became unavailable")
            time.sleep(0.25)
    except _SupervisorSignal as exc:
        return 128 + exc.signum
    finally:
        for name, child in reversed(owned):
            _stop_process_group(child, name)
        if qwen_log_handle is not None:
            qwen_log_handle.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def _lookup(value: Mapping[str, Any], dotted: str) -> Any:
    current: Any = value
    for part in dotted.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return ""
        current = current[part]
    return current


def _path(value: Any) -> str:
    raw = str(value or "").strip()
    if not raw:
        return ""
    raw = raw.replace("{repo_root}", str(REPO_ROOT))
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return str(path.resolve())


def descriptor_paths() -> tuple[Path, ...]:
    return tuple(sorted(DESCRIPTOR_ROOT.glob("*.yml")))


def deployment_names() -> tuple[str, ...]:
    return tuple(path.stem for path in descriptor_paths())


def load_deployment(name: str) -> dict[str, Any]:
    selected = str(name).strip()
    if not selected or selected != Path(selected).name:
        raise DeploymentError("invalid Piper deployment name")
    path = DESCRIPTOR_ROOT / f"{selected}.yml"
    if not path.is_file():
        available = ", ".join(deployment_names()) or "none"
        raise DeploymentError(
            f"unknown Piper deployment {selected!r}; available: {available}"
        )
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise DeploymentError(f"Piper descriptor must be a mapping: {path}")
    if value.get("schema") != SCHEMA or value.get("name") != selected:
        raise DeploymentError(f"Piper descriptor identity mismatch: {path}")
    for section in ("bundle", "policy", "server", "operator", "tasks"):
        if not isinstance(value.get(section), dict):
            raise DeploymentError(f"Piper descriptor is missing {section}: {path}")
    frequency = float(_lookup(value, "policy.action_frequency_hz"))
    horizon = int(_lookup(value, "policy.action_horizon"))
    execute = int(_lookup(value, "policy.default_execute_chunk"))
    mode = str(_lookup(value, "policy.default_execute_mode"))
    if frequency <= 0 or horizon < 1 or not 1 <= execute <= horizon:
        raise DeploymentError(f"invalid policy cadence in {path}")
    if mode not in {"auto", "fixed"}:
        raise DeploymentError(f"invalid execution mode in {path}")
    if _lookup(value, "server.default_mode") not in {"managed", "external"}:
        raise DeploymentError(f"invalid model-server mode in {path}")
    if _lookup(value, "operator.default_control_source") not in {
        "keyboard",
        "qnexo",
        "both",
    }:
        raise DeploymentError(f"invalid control source in {path}")
    release = value.get("release")
    robot = value.get("robot", {})
    trajectory_profile = _lookup(value, "policy.trajectory_profile")
    if trajectory_profile and not Path(_path(trajectory_profile)).is_file():
        raise DeploymentError(f"trajectory profile is missing in {path}")
    if not isinstance(robot, Mapping):
        raise DeploymentError(f"invalid robot binding in {path}")
    if "device_instance" in robot:
        instance = _path(robot["device_instance"])
        if not instance or not Path(instance).is_file():
            raise DeploymentError(f"robot device instance is missing in {path}")
    if release is not None:
        if not isinstance(release, Mapping):
            raise DeploymentError(f"invalid release contract in {path}")
        release_status = str(release.get("status") or "").strip()
        if release_status not in {"hardware_free_candidate", "contract_complete"}:
            raise DeploymentError(f"invalid release status in {path}")
        if release_status == "contract_complete":
            evidence = _path(release.get("evidence"))
            expected_digest = str(release.get("evidence_sha256") or "").strip()
            if not evidence or not Path(evidence).is_file():
                raise DeploymentError(f"release evidence is missing in {path}")
            if len(expected_digest) != 64 or any(
                character not in "0123456789abcdef" for character in expected_digest
            ):
                raise DeploymentError(f"release evidence SHA-256 is invalid in {path}")
            actual_digest = hashlib.sha256(Path(evidence).read_bytes()).hexdigest()
            if actual_digest != expected_digest:
                raise DeploymentError(f"release evidence checksum mismatch in {path}")
    default_task = str(_lookup(value, "tasks.default")).strip()
    inline = _lookup(value, "tasks.inline")
    if not default_task or (inline and default_task not in inline):
        raise DeploymentError(f"invalid default task in {path}")
    qualification = value.get("qualification", {})
    if not isinstance(qualification, dict):
        raise DeploymentError(f"invalid qualification section in {path}")
    if qualification.get("supported") is True:
        tests = qualification.get("tests")
        if not isinstance(tests, list) or not tests or not all(
            isinstance(item, str) and item.strip() for item in tests
        ):
            raise DeploymentError(f"qualification tests are missing in {path}")
        if not (
            str(qualification.get("verifier") or "").strip()
            or str(_lookup(value, "bundle.verifier") or "").strip()
        ):
            raise DeploymentError(f"qualification verifier is missing in {path}")
        width = int(qualification.get("image_width", 0))
        height = int(qualification.get("image_height", 0))
        if width < 1 or height < 1:
            raise DeploymentError(
                f"qualification image dimensions are missing in {path}"
            )
    return value


def resolved_field(deployment: Mapping[str, Any], field: str) -> str:
    value = _lookup(deployment, field)
    if field == "tasks.catalog_path":
        raw = str(value or "").strip()
        if raw.startswith("{repo_root}") or Path(raw).expanduser().is_absolute():
            return _path(raw)
        return raw
    if field in {
        "bundle.default_path",
        "server.script",
        "operator.keyboard_graph",
        "operator.dagger_graph",
        "bundle.verifier",
        "setup.script",
        "qualification.smoke_script",
        "qualification.verifier",
        "observation.camera_profile",
        "robot.device_instance",
        "policy.trajectory_profile",
    }:
        return _path(value)
    if isinstance(value, bool):
        return "1" if value else "0"
    return str(value or "")


def task_catalog_path(deployment: Mapping[str, Any], bundle: Path) -> Path | None:
    relative = str(_lookup(deployment, "tasks.catalog_path") or "").strip()
    if not relative:
        return None
    relative = relative.replace("{repo_root}", str(REPO_ROOT))
    path = Path(relative).expanduser()
    return path.resolve() if path.is_absolute() else bundle / path


def allowed_tasks(deployment: Mapping[str, Any], bundle: Path) -> tuple[str, ...]:
    inline = _lookup(deployment, "tasks.inline")
    if inline:
        if not isinstance(inline, list) or not all(
            isinstance(item, str) and item.strip() for item in inline
        ):
            raise DeploymentError("inline Piper tasks must be non-empty strings")
        return tuple(item.strip() for item in inline)
    catalog_path = task_catalog_path(deployment, bundle)
    if catalog_path is None or not catalog_path.is_file():
        raise DeploymentError(f"Piper task catalog is missing: {catalog_path}")
    value = json.loads(catalog_path.read_text(encoding="utf-8"))
    rows = value.get("tasks") if isinstance(value, dict) else None
    if value.get("exact_match_required") is not True or not isinstance(rows, list):
        raise DeploymentError("Piper task catalog must require exact matches")
    tasks = tuple(
        str(row.get("task", "")).strip() for row in rows if isinstance(row, dict)
    )
    if not tasks or len(tasks) != len(rows) or len(tasks) != len(set(tasks)):
        raise DeploymentError("Piper task catalog is malformed")
    return tasks


def validate_launch(
    deployment: Mapping[str, Any],
    bundle: Path,
    task: str,
    *,
    run_verifier: bool = True,
) -> Path | None:
    if not bundle.is_dir():
        raise DeploymentError(f"Piper bundle directory is missing: {bundle}")
    missing = [
        item
        for item in _lookup(deployment, "bundle.required")
        if not (bundle / str(item)).exists()
    ]
    if missing:
        raise DeploymentError(
            f"Piper bundle is incomplete ({', '.join(map(str, missing))}): {bundle}"
        )
    tasks = allowed_tasks(deployment, bundle)
    if task not in tasks:
        raise DeploymentError(
            f"task must exactly match the {deployment['name']} task catalog"
        )
    verifier = _path(_lookup(deployment, "bundle.verifier"))
    if run_verifier and verifier:
        result = subprocess.run(
            [sys.executable, verifier, str(bundle)],
            cwd=REPO_ROOT,
            env={**os.environ, "PYTHONPATH": f"{REPO_ROOT}:{os.environ.get('PYTHONPATH', '')}"},
            check=False,
        )
        if result.returncode:
            raise DeploymentError(
                f"Piper bundle identity check failed for {deployment['name']}"
            )
    return task_catalog_path(deployment, bundle)


def _shell_fields(deployment: Mapping[str, Any]) -> bytes:
    rows = [resolved_field(deployment, field) for field in SHELL_FIELDS]
    return b"\0".join(row.encode() for row in rows) + b"\0"


def _run_setup(deployment: Mapping[str, Any], bundle: Path) -> int:
    script = _path(_lookup(deployment, "setup.script"))
    if not script:
        raise DeploymentError(f"{deployment['name']} has no automated setup script")
    return subprocess.call(
        ["bash", script],
        cwd=REPO_ROOT,
        env={**os.environ, "PI05_BUNDLE_ROOT": str(bundle)},
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list")
    show = subparsers.add_parser("show")
    show.add_argument("deployment")
    fields = subparsers.add_parser("shell-fields")
    fields.add_argument("deployment")
    validate = subparsers.add_parser("validate")
    validate.add_argument("deployment")
    validate.add_argument("--bundle", required=True, type=Path)
    validate.add_argument("--task", required=True)
    validate.add_argument("--skip-verifier", action="store_true")
    setup = subparsers.add_parser("setup")
    setup.add_argument("deployment")
    setup.add_argument("--bundle", type=Path)
    check = subparsers.add_parser("check")
    check.add_argument("deployment")
    check.add_argument("--bundle", type=Path)
    check.add_argument("--task")
    for command in ("qualify", "onboard"):
        qualification = subparsers.add_parser(command)
        qualification.add_argument("deployment")
        qualification.add_argument("--bundle", type=Path)
        qualification.add_argument("--task")
        qualification.add_argument("--device", default="cuda")
        qualification.add_argument(
            "--with-setup",
            action="store_true",
            help="prepare/update the model runtime before qualification",
        )
        qualification.add_argument(
            "--quick",
            action="store_true",
            help="skip model-server startup and synthetic inference",
        )
        qualification.add_argument(
            "--dry-run",
            action="store_true",
            help="write a report containing the planned commands only",
        )
        qualification.add_argument("--report-dir", type=Path)
        qualification.add_argument("--server-timeout", type=int)
    run = subparsers.add_parser("run")
    run.add_argument("deployment")
    run.add_argument(
        "arguments",
        nargs=argparse.REMAINDER,
        help=(
            "launch_robot.sh options, plus --pigey/--no-pigey, "
            "--pigey-check-steps N, --pigey-demo PATH, "
            "--pigey-reinitialize-steps N, "
            "--pigey-max-task-steps N, "
            "--pigey-rounds N, "
            "--pigey-record-demo, --pigey-preset-ui, and "
            "--pigey-expected-tasks N, --pigey-demo-sample-fps FPS, "
            "--pigey-demo-max-frames N"
        ),
    )
    args = parser.parse_args(argv)
    if args.command == "list":
        for name in deployment_names():
            item = load_deployment(name)
            print(f"piper_x_dual {name:28} {item['label']}")
        return 0
    deployment = load_deployment(args.deployment)
    if args.command == "show":
        print(yaml.safe_dump(deployment, sort_keys=False).rstrip())
        return 0
    if args.command == "shell-fields":
        sys.stdout.buffer.write(_shell_fields(deployment))
        return 0
    if args.command == "validate":
        validate_launch(
            deployment,
            args.bundle.expanduser().resolve(),
            args.task,
            run_verifier=not args.skip_verifier,
        )
        return 0
    if args.command == "setup":
        bundle = args.bundle or Path(resolved_field(deployment, "bundle.default_path"))
        return _run_setup(deployment, bundle.expanduser().resolve())
    if args.command == "check":
        bundle = args.bundle or Path(resolved_field(deployment, "bundle.default_path"))
        task = args.task or str(_lookup(deployment, "tasks.default"))
        bundle = bundle.expanduser().resolve()
        validate_launch(deployment, bundle, task)
        return subprocess.call(
            [
                "bash",
                str(REPO_ROOT / "scripts/launch_robot.sh"),
                "piper_x_dual",
                "--deployment",
                deployment["name"],
                "--ckpt",
                str(bundle),
                "--task",
                task,
                "--no-write",
                "--dry-run",
            ],
            cwd=REPO_ROOT,
        )
    if args.command in {"qualify", "onboard"}:
        from XPolicyLab.policy.Pi_05.piper_qualification import PiperQualification

        bundle = args.bundle or Path(resolved_field(deployment, "bundle.default_path"))
        task = args.task or str(_lookup(deployment, "tasks.default"))
        report = PiperQualification(
            deployment,
            bundle,
            task=task,
            device=args.device,
            onboard=args.command == "onboard",
            with_setup=args.with_setup,
            quick=args.quick,
            dry_run=args.dry_run,
            report_dir=args.report_dir,
            server_timeout_seconds=args.server_timeout,
        ).run()
        print(report)
        return 0
    if args.command == "run":
        launch_arguments, pigey = parse_pigey_run_arguments(args.arguments)
        if pigey.enabled:
            return run_with_pigey(deployment, launch_arguments, pigey)
        command = _graph_command(deployment, launch_arguments)
        os.execv("/usr/bin/bash", command)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DeploymentError, RuntimeError, OSError, json.JSONDecodeError) as exc:
        print(f"piper_deployments.py: {exc}", file=sys.stderr)
        raise SystemExit(2)
