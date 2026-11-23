# Tianji MARVIN dual-arm runtime

This package starts with a read-only SDK boundary. Importing it never connects,
clears errors, enables servos, or sends motion commands.

## SDK dependency

The vendor SDK `100343` source is pinned under
`third_party/robot_sdks/tianji_marvin`. Build it once on a new machine:

```bash
scripts/setup_robot_sdks.sh prepare tianji_marvin
```

The adapter selects that repository build by default. Explicit overrides remain
available in this precedence order: `--sdk-library`, `TIANJI_SDK_LIBRARY`, then
`TIANJI_SDK_ROOT`. A system `libMarvinSDK.so` is only a final compatibility
fallback.

## Read-only probe

```bash
export TIANJI_CONTROLLER_IP=192.0.2.10
xpolicylab-tianji-probe
```

The IP in this example is documentation-only. Use the controller address for the
actual deployment. The probe checks the local ctypes layout, vendor ABI, SDK and
controller versions, both-arm realtime frames, joint feedback, arm errors, and
servo errors. It always calls `OnRelease` when it exits.

The probe prints all readable diagnostics even when the realtime counters are
static or an arm reports an error. It returns a nonzero status in that degraded
case, so readiness checks cannot mistake a stale feedback buffer for a healthy
control stream.

## Explicit error reset

Error reset is kept separate from the read-only probe and requires both an arm
selection and `--yes`:

```bash
xpolicylab-tianji-reset-error --arm right --yes
```

It first verifies SDK/controller version compatibility, an active arm error,
and the stopped flag. It sends only the selected arm's vendor reset request;
it does not enable servos, change modes, or send a target.

Only one process may own the SDK connection because the vendor library binds
local UDP port 4730. Future Dora/ZMQ adapters must consume state from this owner
instead of loading the SDK again.
