"""Release-bound execution intervals; model prediction horizon stays unchanged."""
import argparse
import json

from XPolicyLab.policy.Pi_05.piper_deployments import REPO_ROOT, load_deployment

STEPS = (10, 15, 20, 25)
CONTRACT = {
    "schema": "xpolicylab.native-execution-selection/v1",
    "steps": list(STEPS),
    "mode": "fixed",
    "restart_required": True,
    "prediction_horizon": 50,
    "model_execute_prefix_semantics": "recommendation_only",
}


def execution_options(name):
    deployment = load_deployment(name)
    release = deployment.get("release", {})
    if release.get("status") != "contract_complete":
        return ()
    evidence = json.loads((REPO_ROOT / release["evidence"]).read_text())
    selection = evidence.get("execution_selection")
    if selection is None:
        return ()
    native = evidence.get("native_contract", {})
    policy = deployment["policy"]
    if (selection != CONTRACT or policy["default_execute_mode"] != "fixed"
            or policy["action_horizon"] != 50 or policy["action_frequency_hz"] != 30
            or policy["default_execute_chunk"] != native.get("commit_steps")
            or native.get("future_budget_steps") != 5 or native.get("max_prefix_steps") != 6):
        raise ValueError("执行步数选择合同不匹配，请重新验证部署")
    return STEPS


def validate_execution_steps(name, steps):
    if type(steps) is not int or steps not in execution_options(name):
        raise ValueError("此部署仅支持已验证的执行步数：10、15、20、25")
    return steps


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("deployment")
    parser.add_argument("steps", type=int)
    args = parser.parse_args()
    print(validate_execution_steps(args.deployment, args.steps))
