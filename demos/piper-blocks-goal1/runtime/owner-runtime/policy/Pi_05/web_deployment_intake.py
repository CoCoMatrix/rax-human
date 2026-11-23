"""Personal Web enrolment: inspect -> official qualification -> atomic registration.

Inspection never executes code from a bundle. New model adapters and descriptors
must be authored in the repository; directory names cannot establish a contract.
No command in this module starts a robot graph or switches the active deployment.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time
import uuid

from .piper_deployments import REPO_ROOT, load_deployment

WORKSPACE = REPO_ROOT.parent
BUNDLE_ROOT = WORKSPACE / "model-store"
INTAKE_ROOT = REPO_ROOT / ".xpolicy/web-deployments"
BASE_CATALOG = REPO_ROOT / "policy/Pi_05/junqi_personal_web_deployments.json"
SWITCH_LOCK = WORKSPACE / "migration/piper-deployment-web-switch/active.lock"
BASELINE = "junqi-bat23-mixed2500-native-rtc5-instance01"
SCHEMA = "xpolicylab.web-deployment-enrolment/v1"
NATIVE_ENV = {
    "PIPER_TRAINING_RTC_ENABLED": "1", "PIPER_NATIVE_RTC_FUTURE_STEPS": "5",
    "ACTION_RTC_MODE": "disabled", "ACTION_RTC_TRANSITION_STEPS": "0",
    "PIPER_MODEL_FILTER_ENABLED": "0",
}


def _json(path: Path) -> dict:
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError(f"元数据文件过大：{path.name}")
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"元数据必须是 JSON 对象：{path.name}")
    return value


def _name(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,119}", value):
        raise ValueError("部署名只能包含英文字母、数字、下划线或连字符")
    return value


def _inside(value: str | Path, root: Path) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.is_relative_to(root.resolve()) or path == root.resolve():
        raise ValueError(f"路径必须位于个人目录 {root} 内")
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _atomic(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def _locked():
    SWITCH_LOCK.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(SWITCH_LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as exc:
        raise ValueError("部署切换或接入正在进行，请等待完成") from exc
    os.close(fd)
    try:
        yield
    finally:
        SWITCH_LOCK.unlink(missing_ok=True)


def _environment() -> dict[str, str]:
    # A Web request runs inside a Dora node: never inherit its old prompt,
    # checkpoint, profile overrides, write mode or graph-session variables.
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(("DORA", "PIPER", "MODEL", "POLICY", "ACTION", "ENABLE", "RECAP", "QNEXO"))
           and key not in {"TASK", "ROBOT_ID"}}
    env.update(NATIVE_ENV)
    env["PYTHONPATH"] = str(REPO_ROOT)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return env


def _evidence(deployment: dict) -> dict:
    if deployment.get("release", {}).get("status") != "contract_complete":
        raise ValueError("部署合同尚未完整，不能加入真机切换列表")
    return _json(_inside(REPO_ROOT / deployment["release"]["evidence"], REPO_ROOT))


def _compatible(deployment: dict, bundle: Path) -> dict:
    if Path(deployment["bundle"]["default_path"]).resolve() != bundle:
        raise ValueError("所选目录与部署配置绑定的模型路径不一致")
    baseline = load_deployment(BASELINE)
    evidence, reference = _evidence(deployment), _evidence(baseline)
    for section, key in (("robot", "device_instance"), ("observation", "camera_profile"),
                         ("operator", "keyboard_graph"), ("server", "default_mode"),
                         ("policy", "action_frequency_hz"), ("policy", "action_horizon"),
                         ("policy", "default_execute_chunk"), ("policy", "default_execute_mode")):
        if deployment.get(section, {}).get(key) != baseline.get(section, {}).get(key):
            raise ValueError(f"与当前机器人切换环境不兼容：{section}.{key}，需要单独适配")
    if deployment.get("policy", {}).get("trajectory_profile") or deployment.get("pigey", {}).get("supported"):
        raise ValueError("当前入口不接入轨迹实验或 Pigey 部署")
    if evidence.get("native_contract") != reference.get("native_contract"):
        raise ValueError("Native RTC 合同不匹配，需要单独适配")
    if deployment.get("qualification", {}).get("supported") is not True:
        raise ValueError("部署尚未提供完整验证流程")
    for section, key in (("server", "script"), ("bundle", "verifier")):
        if not _inside(REPO_ROOT / deployment[section][key], REPO_ROOT).is_file():
            raise ValueError(f"缺少仓库适配文件：{section}.{key}")
    for item in evidence.get("runtime_bindings", []):
        path = _inside(REPO_ROOT / item["path"], REPO_ROOT)
        if _sha(path) != item["sha256"]:
            raise ValueError(f"运行代码与发布记录不一致：{item['path']}")
    if not evidence.get("runtime_bindings"):
        raise ValueError("缺少运行代码绑定记录")
    return evidence


def _fingerprint(deployment: dict, bundle: Path) -> str:
    evidence = _compatible(deployment, bundle)
    value = {"descriptor": deployment, "release": evidence,
             "bundle": str(bundle), "metadata": {name: _sha(bundle / name)
             for name in ("contract.json", "deploy_input.json", "package_manifest.json")}}
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def _report_valid(report: dict, deployment: str, bundle: Path) -> None:
    if (report.get("schema") != "xpolicylab.piper-qualification/v1"
            or report.get("status") != "passed" or report.get("quick") is not False
            or report.get("dry_run") is not False or report.get("hardware_access") is not False
            or report.get("deployment") != deployment or report.get("bundle") != str(bundle)):
        raise ValueError("需要当前模型的完整验证报告；quick、计划或失败报告不能用于接入")
    stages = {row["name"]: row["status"] for row in report.get("stages", [])}
    required = {"preflight", "descriptor_contract", "adapter_tests", "websocket_service", "launcher_no_write"}
    if not required <= stages.keys() or any(stages[name] != "passed" for name in required):
        raise ValueError("验证报告缺少模型推理、适配测试或启动预检")
    if "failed" in stages.values():
        raise ValueError("验证报告包含失败步骤")


def _verify_receipt(receipt: dict, deployment: dict, bundle: Path) -> None:
    if receipt.get("schema") != SCHEMA or receipt.get("deployment") != deployment["name"]:
        raise ValueError("接入验证记录的部署身份不匹配")
    report = _inside(receipt["report"], INTAKE_ROOT)
    if receipt.get("report_sha256") != _sha(report):
        raise ValueError("完整验证报告发生变化，请重新验证")
    _report_valid(_json(report), deployment["name"], bundle)
    if receipt.get("fingerprint") != _fingerprint(deployment, bundle):
        raise ValueError("模型或部署代码已变化，请重新运行完整验证")


def registered_options() -> list[dict[str, str]]:
    rows = []
    for path in sorted((INTAKE_ROOT / "registered").glob("*.json")):
        value = _json(path)
        name = _name(value.get("deployment", ""))
        label = value.get("label", "")
        if (path.stem != name or value.get("schema") != SCHEMA
                or not isinstance(label, str) or not 1 <= len(label.strip()) <= 120):
            raise ValueError("新增部署登记记录无效")
        rows.append({"name": name, "label": label})
    return rows


def verify_registered(name: str) -> None:
    path = INTAKE_ROOT / "registered" / f"{_name(name)}.json"
    if path.exists():
        deployment = load_deployment(name)
        bundle = _inside(deployment["bundle"]["default_path"], BUNDLE_ROOT)
        _verify_receipt(_json(path), deployment, bundle)


def inspect_bundle(bundle_path: str, deployment_name: str = "") -> dict:
    bundle = _inside(bundle_path, BUNDLE_ROOT)
    if not bundle.is_dir() or bundle.name.endswith(".partial"):
        raise ValueError("请选择已完整下载的部署包目录，不能使用 .partial 目录")
    missing = [name for name in ("contract.json", "deploy_input.json", "package_manifest.json") if not (bundle / name).is_file()]
    result = {"bundle": str(bundle), "can_register": False, "deployment": "", "label": "", "commands": []}
    if missing:
        return {**result, "stage": "incomplete", "message": "缺少包元数据：" + "、".join(missing)}
    for name in ("contract.json", "deploy_input.json", "package_manifest.json"):
        _json(bundle / name)
    candidates = []
    for path in sorted((REPO_ROOT / "policy/Pi_05/deployments").glob("*.yml")):
        if deployment_name and path.stem != _name(deployment_name):
            continue
        try:
            deployment = load_deployment(path.stem)
        except ValueError:
            if deployment_name:
                raise
            continue
        if Path(deployment["bundle"]["default_path"]).resolve() == bundle:
            candidates.append(deployment)
    if len(candidates) != 1:
        return {**result, "stage": "needs_adapter", "candidates": [d["name"] for d in candidates],
                "message": "请填写部署名以选择适配配置。" if candidates else
                "尚无匹配的仓库适配配置；需先注册部署 YAML、模型启动脚本和合同校验器。不会自动套用旧模型配置。"}
    deployment = candidates[0]
    name = deployment["name"]
    result.update(deployment=name, label=deployment["label"][:120])
    _compatible(deployment, bundle)
    base_names = {row["name"] for row in _json(BASE_CATALOG)["deployments"]}
    if name in base_names or (INTAKE_ROOT / "registered" / f"{name}.json").exists():
        verify_registered(name)
        return {**result, "stage": "registered", "message": "已在切换列表中，无需重复接入。"}
    receipt = INTAKE_ROOT / "qualified" / f"{name}.json"
    command = [str(WORKSPACE / "xpolicy"), "web", "qualify", "--bundle", str(bundle), "--deployment", name]
    result["commands"] = [shlex.join(command)]
    if receipt.exists():
        _verify_receipt(_json(receipt), deployment, bundle)
        return {**result, "stage": "ready", "can_register": True, "message": "完整验证已通过，可以加入切换列表。"}
    return {**result, "stage": "needs_qualification", "message": "适配配置已找到；需先在停止当前运行时后执行下方完整验证命令。"}


def qualify(bundle_path: str, deployment_name: str, with_setup: bool = False) -> dict:
    name = _name(deployment_name)
    bundle = _inside(bundle_path, BUNDLE_ROOT)
    deployment = load_deployment(name)
    fingerprint = _fingerprint(deployment, bundle)
    run_dir = INTAKE_ROOT / "reports" / f"{time.time_ns()}-{name}"
    run_dir.mkdir(parents=True)
    command = ["/usr/bin/python3", str(REPO_ROOT / "policy/Pi_05/piper_deployments.py"),
               "onboard" if with_setup else "qualify", name, "--bundle", str(bundle), "--report-dir", str(run_dir)]
    # The canonical runner rejects active robot/model runtimes before GPU work.
    with _locked():
        with (run_dir / "qualification.log").open("xb") as stream:
            completed = subprocess.run(command, cwd=REPO_ROOT, env=_environment(), stdout=stream, stderr=subprocess.STDOUT, timeout=3600)
        if completed.returncode:
            raise ValueError(f"完整验证失败，当前部署未变。日志：{run_dir / 'qualification.log'}")
        report = run_dir / "qualification.json"
        _report_valid(_json(report), name, bundle)
        if _fingerprint(load_deployment(name), bundle) != fingerprint:
            raise ValueError("验证期间部署文件发生变化，请重新验证")
        receipt = {"schema": SCHEMA, "deployment": name, "bundle": str(bundle), "fingerprint": fingerprint,
                   "report": str(report), "report_sha256": _sha(report), "qualified_ns": time.time_ns()}
        _atomic(INTAKE_ROOT / "qualified" / f"{name}.json", receipt)
    return {"ok": True, "stage": "ready", "deployment": name, "report": str(report)}


def register(bundle_path: str, deployment_name: str = "", label: str = "") -> dict:
    with _locked():
        result = inspect_bundle(bundle_path, deployment_name)
        if result["stage"] == "registered":
            return {"ok": True, **result}
        if not result["can_register"]:
            raise ValueError(result["message"])
        name = result["deployment"]
        label = (label or result["label"]).strip()
        if not 1 <= len(label) <= 120:
            raise ValueError("按钮名称需要 1–120 个字符")
        receipt = _json(INTAKE_ROOT / "qualified" / f"{name}.json")
        # Verify again immediately before publication; never modify old releases.
        _verify_receipt(receipt, load_deployment(name), Path(result["bundle"]))
        _atomic(INTAKE_ROOT / "registered" / f"{name}.json",
                {**receipt, "label": label, "registered_ns": time.time_ns()})
    return {"ok": True, "stage": "registered", "deployment": name,
            "message": "已加入切换列表；当前模型未切换。"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for action in ("inspect", "qualify", "register"):
        command = commands.add_parser(action)
        command.add_argument("--bundle", required=True)
        command.add_argument("--deployment", default="", required=action == "qualify")
        if action == "register":
            command.add_argument("--label", default="")
        if action == "qualify":
            command.add_argument("--with-setup", action="store_true")
    args = parser.parse_args()
    try:
        if args.command == "inspect":
            result = inspect_bundle(args.bundle, args.deployment)
        elif args.command == "qualify":
            result = qualify(args.bundle, args.deployment, args.with_setup)
        else:
            result = register(args.bundle, args.deployment, args.label)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
