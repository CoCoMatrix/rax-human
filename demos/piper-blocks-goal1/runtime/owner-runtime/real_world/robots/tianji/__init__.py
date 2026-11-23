"""Tianji MARVIN dual-arm integration primitives.

The package intentionally has no Dora or robot-control side effects on import.
"""

from .sdk import (
    ArmSnapshot,
    SdkAbiError,
    SdkConnectionError,
    SdkLibraryNotFoundError,
    StatusTimeoutError,
    TianjiSdk,
    TianjiSnapshot,
    resolve_sdk_library,
    versions_compatible,
)

__all__ = [
    "ArmSnapshot",
    "SdkAbiError",
    "SdkConnectionError",
    "SdkLibraryNotFoundError",
    "StatusTimeoutError",
    "TianjiSdk",
    "TianjiSnapshot",
    "resolve_sdk_library",
    "versions_compatible",
]
