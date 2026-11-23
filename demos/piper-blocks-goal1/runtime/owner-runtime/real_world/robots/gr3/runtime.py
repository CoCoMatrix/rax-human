from __future__ import annotations

from XPolicyLab.real_world.inference import ActionChunkRuntime, ActionChunkStep
from XPolicyLab.real_world.rtc import RealTimeChunkingConfig
from XPolicyLab.real_world.robots.gr3.mapping import GR3_ACTION_SCHEMA


RuntimeStep = ActionChunkStep


class Gr3Runtime(ActionChunkRuntime):
    """Pure GR3 queue/control core; Dora I/O stays in the robot process."""

    def __init__(
        self,
        *,
        action_frequency_hz: float = 30.0,
        action_chunk_size: int = 16,
        trigger_steps: int = 0,
        auto_prefetch: bool = False,
        prefetch_quantile: float = 0.95,
        prefetch_margin_steps: int = 1,
        prefetch_window_size: int = 32,
        rtc_config: RealTimeChunkingConfig | None = None,
    ):
        super().__init__(
            GR3_ACTION_SCHEMA,
            action_frequency_hz=action_frequency_hz,
            action_chunk_size=action_chunk_size,
            trigger_steps=trigger_steps,
            auto_prefetch=auto_prefetch,
            prefetch_quantile=prefetch_quantile,
            prefetch_margin_steps=prefetch_margin_steps,
            prefetch_window_size=prefetch_window_size,
            rtc_config=rtc_config,
        )
