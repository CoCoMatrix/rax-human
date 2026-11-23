from __future__ import annotations

import ctypes
import ctypes.util
from dataclasses import asdict, dataclass
import ipaddress
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Literal, Mapping


SDK_LIBRARY_ENV = "TIANJI_SDK_LIBRARY"
SDK_ROOT_ENV = "TIANJI_SDK_ROOT"
CONTROLLER_IP_ENV = "TIANJI_CONTROLLER_IP"
EXPECTED_DCSS_SIZE = 1428
JOINT_COUNT = 7
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
BUNDLED_SDK_ROOT = REPOSITORY_ROOT / "third_party" / "robot_sdks" / "tianji_marvin"


class SdkLibraryNotFoundError(RuntimeError):
    pass


class SdkAbiError(RuntimeError):
    pass


class SdkConnectionError(RuntimeError):
    pass


class StatusTimeoutError(RuntimeError):
    pass


class _PackedStructure(ctypes.Structure):
    _layout_ = "gcc-sysv"
    _pack_ = 4


class StateCtr(_PackedStructure):
    _fields_ = [
        ("m_CurState", ctypes.c_int32),
        ("m_CmdState", ctypes.c_int32),
        ("m_ERRCode", ctypes.c_int32),
    ]


class RtOut(_PackedStructure):
    _fields_ = [
        ("m_OutFrameSerial", ctypes.c_int32),
        ("m_FB_Joint_Pos", ctypes.c_float * JOINT_COUNT),
        ("m_FB_Joint_Vel", ctypes.c_float * JOINT_COUNT),
        ("m_FB_Joint_PosE", ctypes.c_float * JOINT_COUNT),
        ("m_FB_Joint_Cmd", ctypes.c_float * JOINT_COUNT),
        ("m_FB_Joint_CToq", ctypes.c_float * JOINT_COUNT),
        ("m_FB_Joint_SToq", ctypes.c_float * JOINT_COUNT),
        ("m_FB_Joint_Them", ctypes.c_float * JOINT_COUNT),
        ("m_EST_Joint_Firc", ctypes.c_float * JOINT_COUNT),
        ("m_EST_Joint_Firc_Dot", ctypes.c_float * JOINT_COUNT),
        ("m_EST_Joint_Force", ctypes.c_float * JOINT_COUNT),
        ("m_EST_Cart_FN", ctypes.c_float * 6),
        ("m_TipDI", ctypes.c_uint8),
        ("m_LowSpdFlag", ctypes.c_uint8),
        ("m_pad", ctypes.c_char),
        ("m_TrajState", ctypes.c_uint8),
    ]


class RtIn(_PackedStructure):
    _fields_ = [
        ("m_RtInSwitch", ctypes.c_int32),
        ("m_ImpType", ctypes.c_int32),
        ("m_InFrameSerial", ctypes.c_int32),
        ("m_FrameMissCnt", ctypes.c_int16),
        ("m_MaxFrameMissCnt", ctypes.c_int16),
        ("m_SysCyc", ctypes.c_int32),
        ("m_SysCycMissCnt", ctypes.c_int16),
        ("m_MaxSysCycMissCnt", ctypes.c_int16),
        ("m_ToolKine", ctypes.c_float * 6),
        ("m_ToolDyn", ctypes.c_float * 10),
        ("m_Joint_CMD_Pos", ctypes.c_float * JOINT_COUNT),
        ("m_Joint_Vel_Ratio", ctypes.c_int16),
        ("m_Joint_Acc_Ratio", ctypes.c_int16),
        ("m_Joint_K", ctypes.c_float * JOINT_COUNT),
        ("m_Joint_D", ctypes.c_float * JOINT_COUNT),
        ("m_DragSpType", ctypes.c_int32),
        ("m_DragSpPara", ctypes.c_float * 6),
        ("m_Cart_KD_Type", ctypes.c_int32),
        ("m_Cart_K", ctypes.c_float * 6),
        ("m_Cart_D", ctypes.c_float * 6),
        ("m_Cart_KN", ctypes.c_float),
        ("m_Cart_DN", ctypes.c_float),
        ("m_Force_FB_Type", ctypes.c_int32),
        ("m_Force_Type", ctypes.c_int32),
        ("m_Force_Dir", ctypes.c_float * 6),
        ("m_Force_PIDUL", ctypes.c_float * JOINT_COUNT),
        ("m_Force_AdjLmt", ctypes.c_float),
        ("m_Force_Cmd", ctypes.c_float),
        ("m_SET_Tags", ctypes.c_uint8 * 16),
        ("m_Update_Tags", ctypes.c_uint8 * 16),
        ("m_PvtID", ctypes.c_uint8),
        ("m_PvtID_Update", ctypes.c_uint8),
        ("m_Pvt_RunID", ctypes.c_uint8),
        ("m_Pvt_RunState", ctypes.c_uint8),
    ]


class Dcss(_PackedStructure):
    _fields_ = [
        ("m_State", StateCtr * 2),
        ("m_In", RtIn * 2),
        ("m_Out", RtOut * 2),
        ("m_ParaName", ctypes.c_char * 30),
        ("m_ParaType", ctypes.c_uint8),
        ("m_ParaIns", ctypes.c_uint8),
        ("m_ParaValueI", ctypes.c_int32),
        ("m_ParaValueF", ctypes.c_float),
        ("m_ParaCmdSerial", ctypes.c_int16),
        ("m_ParaRetSerial", ctypes.c_int16),
    ]


@dataclass(frozen=True, slots=True)
class ArmSnapshot:
    sdk_arm: str
    logical_side: str
    state: int
    command_state: int
    error_code: int
    frame_serial: int
    stopped: bool
    trajectory_state: int
    joint_position_deg: tuple[float, ...]
    joint_position_rad: tuple[float, ...]
    joint_velocity_deg_s: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...]
    joint_command_deg: tuple[float, ...]
    joint_command_rad: tuple[float, ...]
    joint_current_raw: tuple[float, ...]
    joint_torque_raw: tuple[float, ...]
    joint_temperature_raw: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class TianjiSnapshot:
    timestamp_ns: int
    monotonic_ns: int
    left: ArmSnapshot
    right: ArmSnapshot

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _candidate_libraries(root: Path) -> list[Path]:
    direct = [root / "libMarvinSDK.so", root / "lib" / "libMarvinSDK.so"]
    matches = direct + sorted(root.glob("contrlSDK*/libMarvinSDK.so"))
    unique: list[Path] = []
    for candidate in matches:
        resolved = candidate.expanduser().resolve()
        if resolved.is_file() and resolved not in unique:
            unique.append(resolved)
    return unique


def resolve_sdk_library(
    explicit: str | os.PathLike[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    find_library: Callable[[str], str | None] = ctypes.util.find_library,
) -> str:
    """Resolve libMarvinSDK, preferring the repository-pinned local build."""

    environment = os.environ if environ is None else environ
    selected = explicit or environment.get(SDK_LIBRARY_ENV)
    if selected:
        path = Path(selected).expanduser().resolve()
        if not path.is_file():
            raise SdkLibraryNotFoundError(f"SDK library does not exist: {path}")
        return str(path)

    root_value = environment.get(SDK_ROOT_ENV)
    if root_value:
        root = Path(root_value).expanduser().resolve()
        if not root.is_dir():
            raise SdkLibraryNotFoundError(f"SDK root does not exist: {root}")
        candidates = _candidate_libraries(root)
        if len(candidates) == 1:
            return str(candidates[0])
        if not candidates:
            raise SdkLibraryNotFoundError(
                f"No libMarvinSDK.so found below SDK root: {root}"
            )
        raise SdkLibraryNotFoundError(
            f"Multiple SDK libraries found below {root}; set {SDK_LIBRARY_ENV} explicitly: "
            + ", ".join(map(str, candidates))
        )

    bundled_candidates = _candidate_libraries(BUNDLED_SDK_ROOT)
    if len(bundled_candidates) == 1:
        return str(bundled_candidates[0])
    if len(bundled_candidates) > 1:
        raise SdkLibraryNotFoundError(
            "Multiple repository Tianji SDK libraries found; remove stale build outputs: "
            + ", ".join(map(str, bundled_candidates))
        )

    system_library = find_library("MarvinSDK")
    if system_library:
        return system_library
    raise SdkLibraryNotFoundError(
        "Bundled Tianji SDK is not built. Run "
        "scripts/setup_robot_sdks.sh prepare tianji_marvin, or set "
        f"{SDK_LIBRARY_ENV}/{SDK_ROOT_ENV} for an explicitly qualified override."
    )


def parse_controller_ip(value: str) -> tuple[int, int, int, int]:
    try:
        address = ipaddress.ip_address(value)
    except ValueError as exc:
        raise ValueError(f"invalid controller IP address: {value!r}") from exc
    if not isinstance(address, ipaddress.IPv4Address):
        raise ValueError("the Tianji SDK supports IPv4 controller addresses only")
    if (
        address.is_unspecified
        or address.is_loopback
        or address.is_multicast
        or int(address) == 0xFFFFFFFF
    ):
        raise ValueError(f"controller IP is not a usable unicast address: {address}")
    return tuple(int(part) for part in str(address).split("."))  # type: ignore[return-value]


def controller_ip_from_env(
    explicit: str | None = None, *, environ: Mapping[str, str] | None = None
) -> str:
    environment = os.environ if environ is None else environ
    value = explicit or environment.get(CONTROLLER_IP_ENV)
    if not value:
        raise ValueError(
            f"controller IP is required; pass it explicitly or set {CONTROLLER_IP_ENV}"
        )
    parse_controller_ip(value)
    return value


def versions_compatible(sdk_version: int, controller_version: int) -> bool:
    """Apply the same major-version rule used by the vendor SDK."""

    return sdk_version > 0 and controller_version > 0 and (
        sdk_version // 1000 == controller_version // 1000
    )


def _float_tuple(values: Any) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def _radians(values: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(math.radians(value) for value in values)


def _valid_payload(status: Dcss) -> bool:
    return any(
        status.m_State[index].m_CmdState != 0
        or status.m_Out[index].m_OutFrameSerial != 0
        for index in range(2)
    )


class TianjiSdk:
    """One-process owner of the Tianji MARVIN SDK global connection."""

    def __init__(
        self,
        library: str | os.PathLike[str] | None = None,
        *,
        loader: Callable[[str], Any] = ctypes.CDLL,
        environ: Mapping[str, str] | None = None,
    ) -> None:
        self.library_path = resolve_sdk_library(library, environ=environ)
        self._library = loader(self.library_path)
        self._connected = False
        self._configure_signatures()
        self.check_abi()

    def _configure_signatures(self) -> None:
        api = self._library
        api.CheckSDKTypeCompat.argtypes = [ctypes.POINTER(ctypes.c_int)]
        api.CheckSDKTypeCompat.restype = ctypes.c_int
        api.OnGetSDKVersion.argtypes = []
        api.OnGetSDKVersion.restype = ctypes.c_long
        api.OnLinkTo.argtypes = [ctypes.c_ubyte] * 4
        api.OnLinkTo.restype = ctypes.c_bool
        api.OnRelease.argtypes = []
        api.OnRelease.restype = ctypes.c_bool
        api.OnGetBuf.argtypes = [ctypes.POINTER(Dcss)]
        api.OnGetBuf.restype = ctypes.c_bool
        api.OnGetServoErr_A.argtypes = [ctypes.POINTER(ctypes.c_long)]
        api.OnGetServoErr_A.restype = None
        api.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long)]
        api.OnGetServoErr_B.restype = None
        api.OnGetIntPara.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_long)]
        api.OnGetIntPara.restype = ctypes.c_long
        api.OnClearErr_A.argtypes = []
        api.OnClearErr_A.restype = None
        api.OnClearErr_B.argtypes = []
        api.OnClearErr_B.restype = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def sdk_version(self) -> int:
        return int(self._library.OnGetSDKVersion())

    def check_abi(self) -> None:
        local_size = ctypes.sizeof(Dcss)
        if local_size != EXPECTED_DCSS_SIZE:
            raise SdkAbiError(
                f"Python DCSS layout is {local_size} bytes; expected {EXPECTED_DCSS_SIZE}"
            )
        byte_order = ctypes.c_int(-1)
        result = int(self._library.CheckSDKTypeCompat(ctypes.byref(byte_order)))
        if result < 0:
            raise SdkAbiError(f"Marvin SDK ABI check failed with error mask {-result:#x}")
        if byte_order.value != 0:
            raise SdkAbiError("Marvin SDK requires a little-endian host")

    def connect(self, controller_ip: str) -> None:
        if self._connected:
            raise SdkConnectionError("the SDK instance is already connected")
        octets = parse_controller_ip(controller_ip)
        if not bool(self._library.OnLinkTo(*octets)):
            self._library.OnRelease()
            raise SdkConnectionError(
                "failed to initialize the Tianji connection; local UDP port 4730 may be occupied"
            )
        self._connected = True

    def close(self) -> None:
        if self._connected:
            self._library.OnRelease()
            self._connected = False

    def __enter__(self) -> TianjiSdk:
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def controller_version(self) -> int:
        self._require_connected()
        name = ctypes.create_string_buffer(b"VERSION", 30)
        value = ctypes.c_long(0)
        result = int(
            self._library.OnGetIntPara(
                ctypes.cast(name, ctypes.c_char_p), ctypes.byref(value)
            )
        )
        if result != 0:
            raise SdkConnectionError(f"failed to read controller VERSION: {result}")
        return int(value.value)

    def snapshot(self) -> TianjiSnapshot:
        self._require_connected()
        status = Dcss()
        if not bool(self._library.OnGetBuf(ctypes.byref(status))):
            raise SdkConnectionError("OnGetBuf failed")
        return TianjiSnapshot(
            timestamp_ns=time.time_ns(),
            monotonic_ns=time.monotonic_ns(),
            left=self._arm_snapshot(status, 0, "left"),
            right=self._arm_snapshot(status, 1, "right"),
        )

    def wait_for_live_state(
        self,
        *,
        timeout_s: float = 3.0,
        poll_interval_s: float = 0.02,
        require_advancing_frames: bool = True,
    ) -> TianjiSnapshot:
        if timeout_s <= 0 or poll_interval_s <= 0:
            raise ValueError("timeouts must be positive")
        self._require_connected()
        deadline = time.monotonic() + timeout_s
        first_frames: tuple[int, int] | None = None
        latest: TianjiSnapshot | None = None
        while time.monotonic() < deadline:
            raw = Dcss()
            if bool(self._library.OnGetBuf(ctypes.byref(raw))) and _valid_payload(raw):
                latest = TianjiSnapshot(
                    timestamp_ns=time.time_ns(),
                    monotonic_ns=time.monotonic_ns(),
                    left=self._arm_snapshot(raw, 0, "left"),
                    right=self._arm_snapshot(raw, 1, "right"),
                )
                frames = (latest.left.frame_serial, latest.right.frame_serial)
                if first_frames is None:
                    first_frames = frames
                    if not require_advancing_frames:
                        return latest
                elif all(current != first for current, first in zip(frames, first_frames)):
                    return latest
            time.sleep(poll_interval_s)
        detail = "advancing frames" if require_advancing_frames else "a valid payload"
        raise StatusTimeoutError(
            f"did not receive {detail} from both arms within {timeout_s:g} seconds"
        )

    def servo_errors(self) -> dict[str, tuple[int, ...]]:
        self._require_connected()
        errors_a = (ctypes.c_long * JOINT_COUNT)()
        errors_b = (ctypes.c_long * JOINT_COUNT)()
        self._library.OnGetServoErr_A(errors_a)
        self._library.OnGetServoErr_B(errors_b)
        return {
            "left": tuple(int(value) for value in errors_a),
            "right": tuple(int(value) for value in errors_b),
        }

    def clear_error(self, side: Literal["left", "right"]) -> None:
        """Request an arm error reset without enabling servos or changing mode."""

        self._require_connected()
        if side == "left":
            self._library.OnClearErr_A()
        elif side == "right":
            self._library.OnClearErr_B()
        else:
            raise ValueError("side must be 'left' or 'right'")

    def _require_connected(self) -> None:
        if not self._connected:
            raise SdkConnectionError("the SDK is not connected")

    @staticmethod
    def _arm_snapshot(status: Dcss, index: int, logical_side: str) -> ArmSnapshot:
        state = status.m_State[index]
        output = status.m_Out[index]
        position_deg = _float_tuple(output.m_FB_Joint_Pos)
        velocity_deg_s = _float_tuple(output.m_FB_Joint_Vel)
        command_deg = _float_tuple(output.m_FB_Joint_Cmd)
        return ArmSnapshot(
            sdk_arm="A" if index == 0 else "B",
            logical_side=logical_side,
            state=int(state.m_CurState),
            command_state=int(state.m_CmdState),
            error_code=int(state.m_ERRCode),
            frame_serial=int(output.m_OutFrameSerial),
            stopped=bool(output.m_LowSpdFlag),
            trajectory_state=int(output.m_TrajState),
            joint_position_deg=position_deg,
            joint_position_rad=_radians(position_deg),
            joint_velocity_deg_s=velocity_deg_s,
            joint_velocity_rad_s=_radians(velocity_deg_s),
            joint_command_deg=command_deg,
            joint_command_rad=_radians(command_deg),
            joint_current_raw=_float_tuple(output.m_FB_Joint_CToq),
            joint_torque_raw=_float_tuple(output.m_FB_Joint_SToq),
            joint_temperature_raw=_float_tuple(output.m_FB_Joint_Them),
        )
