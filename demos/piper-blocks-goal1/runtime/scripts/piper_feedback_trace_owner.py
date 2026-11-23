"""Opt-in diagnostic entrypoint. Calls the unchanged owner; never sends commands."""
import json, os, time, threading
from pathlib import Path
from XPolicyLab.real_world.robots.piper_x_dual.sdk import PiperSdkArmReader

_lock=threading.Lock()
_log=None
_original=PiperSdkArmReader.read_sample

def traced_read(self):
    global _log
    if _log is None:
        _log=Path(os.environ["RAX_FEEDBACK_TRACE"]).open("a", buffering=1)
    # Copy the actual public SDK result before owner conversion; no extra SDK read.
    if not getattr(self.arm,"_rax_diagnostic_wrapped",False):
        get=self.arm.get_joint_angles
        role=self.role
        def observed_get():
            start=time.monotonic_ns();result=get()
            row={"kind":"sdk","role":role,"mono_before_ns":start,
                 "mono_ns":time.monotonic_ns(),"wall_ns":time.time_ns(),
                 "q":None if result is None else list(result.msg),
                 "sdk_timestamp":None if result is None else result.timestamp}
            with _lock:_log.write(json.dumps(row)+"\n")
            return result
        self.arm.get_joint_angles=observed_get
        self.arm._rax_diagnostic_wrapped=True
    result=_original(self)
    row={"kind":"owner_sample","role":self.role,"mono_ns":time.monotonic_ns(),
         "wall_ns":time.time_ns(),"q":list(result.joint_position_rad),
         "arm_timestamp_s":result.arm_timestamp_s,
         "controller_mode":result.controller_mode}
    with _lock:_log.write(json.dumps(row)+"\n")
    return result

if __name__=="__main__":
    if not os.environ.get("RAX_FEEDBACK_TRACE"):raise RuntimeError("diagnostic path required")
    PiperSdkArmReader.read_sample=traced_read
    import runpy
    runpy.run_module("XPolicyLab.real_world.robots.piper_x_dual.dora_owner_node",run_name="__main__")
