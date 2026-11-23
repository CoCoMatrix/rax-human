from __future__ import annotations

import argparse
import time
from typing import Any

from .sdk import (
    CONTROLLER_IP_ENV,
    TianjiSdk,
    controller_ip_from_env,
    versions_compatible,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Explicit Tianji arm error reset; never enables or moves an arm."
    )
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument(
        "--controller-ip",
        help=f"controller IPv4 address; defaults to ${CONTROLLER_IP_ENV}",
    )
    parser.add_argument("--sdk-library", help="exact libMarvinSDK.so path")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="confirm the requested reset after checking current arm state",
    )
    return parser


def _arm(snapshot: Any, side: str) -> Any:
    return snapshot.left if side == "left" else snapshot.right


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not args.yes:
        _parser().error("--yes is required; no reset was sent")
    if args.timeout <= 0:
        _parser().error("--timeout must be positive")

    controller_ip = controller_ip_from_env(args.controller_ip)
    with TianjiSdk(args.sdk_library) as sdk:
        sdk.connect(controller_ip)
        sdk_version = sdk.sdk_version
        controller_version = sdk.controller_version()
        if not versions_compatible(sdk_version, controller_version):
            print(
                f"Refusing reset: SDK {sdk_version} and controller "
                f"{controller_version} are incompatible."
            )
            return 3

        before = sdk.wait_for_live_state(
            timeout_s=min(args.timeout, 1.0),
            require_advancing_frames=False,
        )
        target = _arm(before, args.arm)
        print(
            f"Before reset: {args.arm} state={target.state}, "
            f"error={target.error_code}, stopped={int(target.stopped)}"
        )
        if target.state != 100 and target.error_code == 0:
            print("No active arm error; reset was not sent.")
            return 0
        if not target.stopped:
            print("Refusing reset because the target arm is not stopped.")
            return 4

        sdk.clear_error(args.arm)
        deadline = time.monotonic() + args.timeout
        after = before
        while time.monotonic() < deadline:
            after = sdk.snapshot()
            target = _arm(after, args.arm)
            if target.state != 100 and target.error_code == 0:
                print(
                    f"After reset : {args.arm} state={target.state}, "
                    f"error={target.error_code}, stopped={int(target.stopped)}"
                )
                print("Error cleared; no servo-enable or motion command was sent.")
                return 0
            time.sleep(0.1)

        target = _arm(after, args.arm)
        print(
            f"After reset : {args.arm} state={target.state}, "
            f"error={target.error_code}, stopped={int(target.stopped)}"
        )
        print("Error remains active or reappeared.")
        return 5


if __name__ == "__main__":
    raise SystemExit(main())
