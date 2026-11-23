"""Fail-fast, hardware-free onboarding and qualification for dual-PiPER policies."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

from .piper_deployments import (
    DeploymentError,
    REPO_ROOT,
    _lookup,
    resolved_field,
    validate_launch,
)


REPORT_SCHEMA = "xpolicylab.piper-qualification/v1"
TEST_ENV = REPO_ROOT / ".xpolicy/envs/piper_qualification"
DEFAULT_REPORT_ROOT = REPO_ROOT / ".xpolicy/reports/piper"
ACTIVE_RUNTIME_MARKERS = (
    "launch_robot.sh\x00piper_x_dual",
    "XPolicyLab.real_world.robots.piper_x_dual.dora_owner_node",
    "piper-x-dual-desktop-cleanup-policy",
    "deploy_piper_",
)


class QualificationError(RuntimeError):
    pass


@dataclass(slots=True)
class StageResult:
    name: str
    status: str
    started_at: str
    duration_seconds: float = 0.0
    command: list[str] | None = None
    log: str | None = None
    detail: dict[str, Any] | None = None
    error: str | None = None


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def active_piper_processes() -> list[dict[str, Any]]:
    current = os.getpid()
    matches: list[dict[str, Any]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == current:
            continue
        try:
            command = (entry / "cmdline").read_bytes().decode(errors="replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(marker in command for marker in ACTIVE_RUNTIME_MARKERS):
            matches.append(
                {
                    "pid": int(entry.name),
                    "command": command.replace("\x00", " ").strip(),
                }
            )
    return sorted(matches, key=lambda item: item["pid"])


def free_loopback_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind(("127.0.0.1", 0))
        return int(server.getsockname()[1])


def _tail(path: Path, lines: int = 30) -> str:
    try:
        values = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except FileNotFoundError:
        return ""
    return "\n".join(values[-lines:])


def subprocess_environment(
    overrides: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Return the shared environment for qualification subprocesses."""

    current_pythonpath = os.environ.get("PYTHONPATH", "").strip()
    pythonpath = str(REPO_ROOT)
    if current_pythonpath:
        pythonpath = f"{pythonpath}{os.pathsep}{current_pythonpath}"
    return {
        **os.environ,
        "PYTHONPATH": pythonpath,
        **dict(overrides or {}),
    }


class PiperQualification:
    def __init__(
        self,
        deployment: Mapping[str, Any],
        bundle: Path,
        *,
        task: str,
        device: str = "cuda",
        onboard: bool = False,
        with_setup: bool = False,
        quick: bool = False,
        dry_run: bool = False,
        report_dir: Path | None = None,
        server_timeout_seconds: int | None = None,
    ) -> None:
        self.deployment = dict(deployment)
        self.bundle = bundle.expanduser().resolve()
        self.task = str(task)
        self.device = str(device)
        self.onboard = bool(onboard)
        self.with_setup = bool(with_setup or onboard)
        self.quick = bool(quick)
        self.dry_run = bool(dry_run)
        configured_timeout = int(
            _lookup(deployment, "qualification.server_timeout_seconds") or 300
        )
        self.server_timeout_seconds = int(
            server_timeout_seconds or configured_timeout
        )
        if self.server_timeout_seconds < 1:
            raise QualificationError("server timeout must be positive")
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.report_dir = (
            report_dir.expanduser().resolve()
            if report_dir is not None
            else DEFAULT_REPORT_ROOT / f"{timestamp}-{deployment['name']}"
        )
        self.report_path = self.report_dir / "qualification.json"
        self.stages: list[StageResult] = []
        self.started_at = utc_now()
        self.status = "running"

    def _report(self) -> dict[str, Any]:
        return {
            "schema": REPORT_SCHEMA,
            "status": self.status,
            "started_at": self.started_at,
            "updated_at": utc_now(),
            "deployment": self.deployment["name"],
            "descriptor_schema": self.deployment["schema"],
            "bundle": str(self.bundle),
            "task": self.task,
            "device": self.device,
            "mode": "onboard" if self.onboard else "qualify",
            "quick": self.quick,
            "dry_run": self.dry_run,
            "hardware_access": False,
            "stages": [asdict(stage) for stage in self.stages],
        }

    def _write_report(self) -> None:
        self.report_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.report_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self._report(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.report_path)

    def _record(
        self,
        name: str,
        status: str,
        *,
        started: float,
        command: Sequence[str] | None = None,
        log: Path | None = None,
        detail: dict[str, Any] | None = None,
        error: str | None = None,
    ) -> StageResult:
        stage = StageResult(
            name=name,
            status=status,
            started_at=utc_now(),
            duration_seconds=time.monotonic() - started,
            command=list(command) if command is not None else None,
            log=str(log) if log is not None else None,
            detail=detail,
            error=error,
        )
        self.stages.append(stage)
        self._write_report()
        return stage

    def _command_stage(
        self,
        name: str,
        command: Sequence[str],
        *,
        env: Mapping[str, str] | None = None,
    ) -> None:
        started = time.monotonic()
        log = self.report_dir / f"{len(self.stages) + 1:02d}-{name}.log"
        printable = " ".join(command)
        print(f"[{name}] {printable}", flush=True)
        if self.dry_run:
            self._record(
                name,
                "planned",
                started=started,
                command=command,
                log=log,
            )
            return
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open("w", encoding="utf-8") as stream:
            process = subprocess.Popen(
                list(command),
                cwd=REPO_ROOT,
                env=subprocess_environment(env),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert process.stdout is not None
            for line in process.stdout:
                stream.write(line)
                stream.flush()
                print(line, end="", flush=True)
            return_code = process.wait()
        if return_code:
            error = f"command exited with status {return_code}"
            self._record(
                name,
                "failed",
                started=started,
                command=command,
                log=log,
                error=error,
            )
            raise QualificationError(f"{name}: {error}\n{_tail(log)}")
        self._record(
            name,
            "passed",
            started=started,
            command=command,
            log=log,
        )

    def _intake_command(self) -> list[str] | None:
        intake = _lookup(self.deployment, "intake")
        if not isinstance(intake, Mapping) or not intake:
            return None
        provider = str(intake.get("provider", ""))
        if provider != "aliyun_oss":
            raise QualificationError(f"unsupported intake provider: {provider!r}")
        command = [
            str(intake.get("python") or sys.executable),
            str(REPO_ROOT / "scripts/sync_aliyun_oss_bundle.py"),
            "--bucket",
            str(intake["bucket"]),
            "--endpoint",
            str(intake["endpoint"]),
            "--prefix",
            str(intake["prefix"]),
            "--destination",
            str(self.bundle),
            "--expected-bytes",
            str(int(intake["expected_bytes"])),
            "--threads",
            str(int(intake.get("threads", 8))),
            "--request-timeout-seconds",
            str(int(intake.get("request_timeout_seconds", 120))),
            "--attempts",
            str(int(intake.get("attempts", 5))),
        ]
        metadata = intake.get("metadata", {})
        if not isinstance(metadata, Mapping):
            raise QualificationError("intake metadata must be a mapping")
        for key, value in sorted(metadata.items()):
            command.extend(("--metadata", f"{key}={value}"))
        return command

    def _intake(self) -> None:
        started = time.monotonic()
        if self.bundle.exists():
            self._record(
                "intake",
                "skipped",
                started=started,
                detail={"reason": "immutable destination already exists"},
            )
            return
        command = self._intake_command()
        if command is None:
            raise QualificationError(
                "bundle is missing and the descriptor has no automated intake source"
            )
        self._command_stage("intake", command)

    def _preflight(self) -> None:
        started = time.monotonic()
        active = active_piper_processes()
        if active and not self.dry_run:
            self._record(
                "preflight",
                "failed",
                started=started,
                detail={"active_processes": active},
                error="a live Piper/model runtime is active",
            )
            raise QualificationError(
                "stop the active Piper launch before running GPU/server qualification"
            )
        self._record(
            "preflight",
            "planned" if self.dry_run else "passed",
            started=started,
            detail={"active_processes": active, "hardware_access": False},
        )

    def _validate_descriptor(self) -> None:
        started = time.monotonic()
        if self.dry_run and not self.bundle.exists():
            status = "planned"
            detail = {"reason": "bundle will be validated after intake"}
        else:
            validate_launch(
                self.deployment,
                self.bundle,
                self.task,
                run_verifier=False,
            )
            status = "passed"
            detail = {
                "action_horizon": int(
                    _lookup(self.deployment, "policy.action_horizon")
                ),
                "execute_prefix": int(
                    _lookup(self.deployment, "policy.default_execute_chunk")
                ),
            }
        self._record(
            "descriptor_contract",
            status,
            started=started,
            detail=detail,
        )

    def _full_verifier_command(self) -> list[str]:
        verifier = resolved_field(self.deployment, "qualification.verifier")
        if not verifier:
            verifier = resolved_field(self.deployment, "bundle.verifier")
        if not verifier:
            raise QualificationError(
                f"{self.deployment['name']} has no qualification verifier"
            )
        args = _lookup(self.deployment, "qualification.verifier_args")
        if args and not isinstance(args, list):
            raise QualificationError("qualification.verifier_args must be a list")
        return [sys.executable, verifier, str(self.bundle), *(str(v) for v in args or [])]

    def _setup_or_verify(self) -> None:
        if self.with_setup:
            setup = resolved_field(self.deployment, "setup.script")
            if not setup:
                raise QualificationError(
                    f"{self.deployment['name']} has no setup script"
                )
            self._command_stage(
                "verify_and_setup",
                ["bash", setup],
                env={
                    "MODEL_BUNDLE_ROOT": str(self.bundle),
                    "MODEL_DEVICE": self.device,
                    "PI05_BUNDLE_ROOT": str(self.bundle),
                    "PI05_DEVICE": self.device,
                },
            )
            return
        self._command_stage("bundle_identity", self._full_verifier_command())

    def _test_environment(self) -> None:
        uv = str(
            _lookup(self.deployment, "qualification.uv")
            or "/home/ubuntu/miniconda3/envs/infra/bin/uv"
        )
        self._command_stage(
            "test_environment",
            [
                uv,
                "sync",
                "--project",
                str(REPO_ROOT),
                "--frozen",
                "--group",
                "dev",
                "--no-install-project",
            ],
            env={"UV_PROJECT_ENVIRONMENT": str(TEST_ENV)},
        )

    def _adapter_tests(self) -> None:
        tests = _lookup(self.deployment, "qualification.tests")
        if not isinstance(tests, list) or not tests:
            raise QualificationError("descriptor has no focused qualification tests")
        command = [str(TEST_ENV / "bin/python"), "-m", "pytest", "-q"]
        command.extend(str((REPO_ROOT / str(path)).resolve()) for path in tests)
        self._command_stage(
            "adapter_tests",
            command,
            env={"PYTHONPATH": str(REPO_ROOT)},
        )

    def _wait_for_server(self, process: subprocess.Popen[Any], port: int) -> None:
        deadline = time.monotonic() + self.server_timeout_seconds
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise QualificationError(
                    f"policy server exited before readiness (status {process.returncode})"
                )
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                    return
            except OSError:
                time.sleep(0.25)
        raise QualificationError(
            f"policy server was not ready after {self.server_timeout_seconds}s"
        )

    @staticmethod
    def _stop_server(process: subprocess.Popen[Any]) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)

    def _service_parity(self) -> None:
        name = "websocket_service"
        started = time.monotonic()
        server_script = resolved_field(self.deployment, "server.script")
        if not server_script:
            raise QualificationError("descriptor has no managed model server")
        port = free_loopback_port()
        url = f"ws://127.0.0.1:{port}"
        server_command = ["bash", server_script]
        probe_command = [
            str(TEST_ENV / "bin/python"),
            str(REPO_ROOT / "policy/Pi_05/probe_piper_policy_server.py"),
            "--url",
            url,
            "--task",
            self.task,
            "--action-horizon",
            str(int(_lookup(self.deployment, "policy.action_horizon"))),
            "--frequency-hz",
            str(float(_lookup(self.deployment, "policy.action_frequency_hz"))),
            "--execute-prefix",
            str(int(_lookup(self.deployment, "policy.default_execute_chunk"))),
            "--image-width",
            str(int(_lookup(self.deployment, "qualification.image_width"))),
            "--image-height",
            str(int(_lookup(self.deployment, "qualification.image_height"))),
            "--output",
            str(self.report_dir / "service-inference.json"),
        ]
        server_log = self.report_dir / f"{len(self.stages) + 1:02d}-{name}.log"
        if self.dry_run:
            self._record(
                name,
                "planned",
                started=started,
                command=server_command,
                log=server_log,
                detail={"probe_command": probe_command, "url": url},
            )
            return
        self.report_dir.mkdir(parents=True, exist_ok=True)
        environment = {
            **os.environ,
            "MODEL_BUNDLE_ROOT": str(self.bundle),
            "MODEL_HOST": "127.0.0.1",
            "MODEL_PORT": str(port),
            "MODEL_DEVICE": self.device,
            "PI05_BUNDLE_ROOT": str(self.bundle),
            "PI05_HOST": "127.0.0.1",
            "PI05_PORT": str(port),
            "PI05_DEVICE": self.device,
        }
        with server_log.open("w", encoding="utf-8") as stream:
            process = subprocess.Popen(
                server_command,
                cwd=REPO_ROOT,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            try:
                print(f"[{name}] waiting for {url}", flush=True)
                self._wait_for_server(process, port)
                probe_log = self.report_dir / "service-probe.log"
                with probe_log.open("w", encoding="utf-8") as probe_stream:
                    result = subprocess.run(
                        probe_command,
                        cwd=REPO_ROOT,
                        env={**os.environ, "PYTHONPATH": str(REPO_ROOT)},
                        stdout=probe_stream,
                        stderr=subprocess.STDOUT,
                        text=True,
                        check=False,
                        timeout=self.server_timeout_seconds,
                    )
                if result.returncode:
                    raise QualificationError(
                        f"WebSocket inference probe failed\n{_tail(probe_log)}"
                    )
            except Exception as exc:
                self._record(
                    name,
                    "failed",
                    started=started,
                    command=server_command,
                    log=server_log,
                    detail={"probe_command": probe_command, "url": url},
                    error=str(exc),
                )
                raise
            finally:
                self._stop_server(process)
        evidence = json.loads(
            (self.report_dir / "service-inference.json").read_text(encoding="utf-8")
        )
        self._record(
            name,
            "passed",
            started=started,
            command=server_command,
            log=server_log,
            detail={"probe_command": probe_command, "evidence": evidence},
        )

    def _launcher_dry_run(self) -> None:
        self._command_stage(
            "launcher_no_write",
            [
                "bash",
                str(REPO_ROOT / "scripts/launch_robot.sh"),
                "piper_x_dual",
                "--deployment",
                str(self.deployment["name"]),
                "--ckpt",
                str(self.bundle),
                "--task",
                self.task,
                "--control-source",
                "keyboard",
                "--no-write",
                "--dry-run",
            ],
        )

    def run(self) -> Path:
        self._write_report()
        try:
            self._preflight()
            if _lookup(self.deployment, "qualification.supported") is False:
                raise QualificationError(
                    f"{self.deployment['name']} does not provide automated qualification"
                )
            if self.onboard:
                self._intake()
            self._validate_descriptor()
            self._setup_or_verify()
            self._test_environment()
            self._adapter_tests()
            if not self.quick:
                self._service_parity()
            self._launcher_dry_run()
        except Exception as exc:
            self.status = "failed"
            if not self.stages or self.stages[-1].status != "failed":
                self._record(
                    "qualification",
                    "failed",
                    started=time.monotonic(),
                    error=f"{type(exc).__name__}: {exc}",
                )
            self._write_report()
            raise
        self.status = "planned" if self.dry_run else "passed"
        self._write_report()
        return self.report_path
