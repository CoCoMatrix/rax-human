"""Loopback formal PolicyServer for Pi0.5-only real-observation shadow."""
from __future__ import annotations
import argparse
import asyncio
import json
from pathlib import Path
import signal
import sys
import time
import yaml

from .contracts import PiperContract, require
from .backend import OwnerObserver, PiperXPolicyBackend, write_json
from .description import PiperDescription
from .description_config import mast_configuration


class ShadowModel:
    def __init__(self, backend, owner):
        self.backend, self.owner = backend, owner

    async def reset(self):
        # Logical student cache only: never reset/home the physical arms.
        await self.backend.student.reset(trial_id='junqi-piper-shadow')
        return {'physical_reset': False}

    async def infer(self, observation):
        before = await asyncio.to_thread(self.owner.snapshot)
        obs = self.backend.observe(observation)
        started = time.monotonic()
        payload = await self.backend.propose(obs)
        after = await asyncio.to_thread(self.owner.snapshot)
        require(time.monotonic() - started <= 10, 'student exceeded shadow inference deadline')
        require(before['robot_status'].get('generation') == after['robot_status'].get('generation'),
                'owner generation changed during inference')
        # The formal runtime discards in-flight returns after pause/stop. Do not
        # infer a hardware acknowledgement from the server returning this frame.
        return await asyncio.to_thread(self.backend.execute, obs, payload,
                                       owner_before=before, owner_after=after)

    def get_capabilities(self):
        return {'robot_type': 'piper_x_dual', 'mode': 'pi05_only_shadow',
                'hardware_write_allowed': False, 'teacher_called': False}


async def run(args):
    runtime = args.runtime.resolve(strict=True)
    sys.path.insert(0, str(runtime))
    from client_server.ws.protocol.client import PolicyEvalClient, PolicyEvalClientConfig
    from client_server.ws.model_server import PolicyServer, PolicyServerConfig, deployment_identity_from_config
    config = yaml.safe_load(args.student_config.read_text())
    expected = deployment_identity_from_config(config)
    require(args.student_url == 'ws://127.0.0.1:19013', 'use isolated loopback PiPER student')
    student = PolicyEvalClient(PolicyEvalClientConfig(url=args.student_url,
        evaluation_id='junqi-gap-piper-shadow', max_connect_attempts=1,
        connect_timeout_s=5, request_timeout_s=15, compression=None, proxy=None))
    hello = await student.connect(handshake=True)
    server = None
    try:
        require(hello is not None and hello.payload.get('deployment') == expected,
                'PiPER student HELLO differs from pinned deployment identity')
        contract = PiperContract.from_runtime(runtime)
        description = PiperDescription(contract, args.description_assets)
        mast = mast_configuration(args.mast_joint_rad)
        backend = PiperXPolicyBackend(contract, student, args.output, task=args.task,
                                      identity=expected, max_observation_age_ms=300,
                                      description=description, mast_joint_rad=mast['joint_position_rad'],
                                      mast_metadata=mast)
        write_json(args.output / 'student-hello.json', hello.payload)
        model = ShadowModel(backend, OwnerObserver(args.owner_url, args.task))
        # Preserve verified student identity in the wire HELLO. Adapter identity
        # and mode are advertised separately in capabilities and run.json.
        server = PolicyServer(model, PolicyServerConfig(host='127.0.0.1', port=args.port,
            compression=None, deployment_identity=expected))
        await server.start()
        print(json.dumps({'status': 'ready', 'url': server.url,
                          'mode': 'pi05_only_shadow', 'output': str(args.output)}), flush=True)
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop.set)
        try:
            await asyncio.wait_for(stop.wait(), timeout=args.duration)
        except asyncio.TimeoutError:
            pass
    finally:
        if server is not None:
            await server.stop()
        await student.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', type=Path, required=True)
    p.add_argument('--student-config', type=Path, required=True)
    p.add_argument('--student-url', default='ws://127.0.0.1:19013')
    p.add_argument('--owner-url', default='http://127.0.0.1:18091')
    p.add_argument('--port', type=int, default=19014)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--task', required=True)
    p.add_argument('--duration', type=int, default=900)
    p.add_argument('--description-assets', type=Path,
        default=Path(__file__).resolve().parents[2] / 'assets/robots/piper/global-d435-fixed-v12')
    p.add_argument('--mast-joint-rad', type=float, default=None,
        help='Compatibility argument: v10 mast is fixed at 75 degrees; only zero extra angle is accepted.')
    args = p.parse_args()
    require(1 <= args.duration <= 1800, 'duration must be 1..1800 seconds')
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
