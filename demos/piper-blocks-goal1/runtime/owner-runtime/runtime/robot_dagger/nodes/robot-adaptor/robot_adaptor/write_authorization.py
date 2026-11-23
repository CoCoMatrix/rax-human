"""Fail-closed authorization wrapper for the unique GR3 Aurora writer."""

from __future__ import annotations

from typing import Any


def require_hardware_write_authorization(enabled: bool) -> None:
    if enabled is not True:
        raise PermissionError(
            "GR3 Aurora SDK connection requires explicit GR3_ENABLE_WRITE=1"
        )


class WriteAuthorizedClient:
    """Proxy every Aurora ``set_*`` call through one immutable write gate."""

    def __init__(self, client: Any, *, enabled: bool) -> None:
        self._client = client
        self._enabled = enabled is True

    def __getattr__(self, name: str) -> Any:
        target = getattr(self._client, name)
        if not name.startswith("set_") or not callable(target):
            return target

        def guarded(*args: Any, **kwargs: Any) -> Any:
            require_hardware_write_authorization(self._enabled)
            return target(*args, **kwargs)

        return guarded
