import asyncio
import logging
import os
import warnings
from datetime import datetime, timezone

import numpy as np
import requests
from dotenv import load_dotenv
from livekit import rtc
from livekit.api import LiveKitAPI, UpdateParticipantRequest

from farther_core.auth import generate_token
from farther_core.utils import get_timestamp_us

logger = logging.getLogger(__name__)
UTC = timezone.utc


class AdaptiveSender:
    """
    Adaptive sender that automatically adjusts to network conditions.

    - When network is good: sends at full input rate
    - When network is slow: only keeps latest data, prevents queue buildup

    Key insight: publish_data() returns immediately but data goes into SDK's internal queue.
    We use a background task to actually send, so new data can overwrite pending data
    while a send is in progress.
    """

    def __init__(self, topic: str):
        self.topic = topic
        self.pending_data = None  # Latest data waiting to be sent
        self.send_in_progress = False  # Whether a send task is running
        self.lock = asyncio.Lock()

    async def update(self, data: bytes, room: rtc.Room):
        """Update with new data. If send is in progress, just overwrites pending data."""
        async with self.lock:
            self.pending_data = data

            # If no send in progress, start one
            if not self.send_in_progress and room and room.isconnected:
                self.send_in_progress = True
                asyncio.create_task(self._send_loop(room))

    async def _send_loop(self, room: rtc.Room):
        """Background task that sends pending data until there's nothing left."""
        try:
            while True:
                async with self.lock:
                    if self.pending_data is None:
                        self.send_in_progress = False
                        return
                    data_to_send = self.pending_data
                    self.pending_data = None  # Clear before sending

                try:
                    if room and room.isconnected:
                        await room.local_participant.publish_data(
                            data_to_send, topic=self.topic, reliable=False
                        )
                except Exception as e:
                    logger.error(f"Failed to publish {self.topic} data: {e}")

                # Yield to allow other tasks and check for new data
                await asyncio.sleep(0)
        except Exception as e:
            logger.error(f"Send loop error for {self.topic}: {e}")
            async with self.lock:
                self.send_in_progress = False

# load_dotenv(Path(__file__).parent.parent / ".env.livekit", override=True, verbose=True)
load_dotenv(override=True, verbose=True)

LIVEKIT_URL = os.environ.get("LIVEKIT_URL")
LIVEKIT_API_KEY = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
DAEMON_SERVER_HOST = os.environ.get("DAEMON_SERVER_HOST")
CONNECT_VIA_DAEMON_SERVER = os.environ.get("CONNECT_VIA_DAEMON_SERVER", "0") == "1"

VIDEO_TOPIC = os.environ.get("VIDEO_TOPIC", "camera/top")
VIDEO_WIDTH = int(os.environ.get("VIDEO_WIDTH", "1280"))
VIDEO_HEIGHT = int(os.environ.get("VIDEO_HEIGHT", "480"))
VIDEO_FPS = int(os.environ.get("VIDEO_FPS", "30"))
VIDEO_MAX_BITRATE = int(os.environ.get("VIDEO_MAX_BITRATE", "5_000_000"))
VIDEO_SIMULCAST = os.environ.get("VIDEO_SIMULCAST", "0") == "1"

AUDIO_TOPIC = os.environ.get("AUDIO_TOPIC", "audio")

IDENTITY = os.environ.get("IDENTITY", "robot")


SAMPLE_RATE = int(os.environ.get("SAMPLE_RATE", "16000"))
NUM_CHANNELS = int(os.environ.get("NUM_CHANNELS", "1"))  # mono audio

ROOM_NAME = os.environ.get("ROOM_NAME", "teleop")
ROOM_PASSWORD = os.environ.get("ROOM_PASSWORD", "")

async def setup_room(room: rtc.Room, room_name: str = ROOM_NAME, room_password: str = ROOM_PASSWORD, identity: str = IDENTITY, host: str = DAEMON_SERVER_HOST, auto_subscribe: bool = False, dynacast: bool = False) -> rtc.Room:
    if CONNECT_VIA_DAEMON_SERVER:
        print(f"Connect to LiveKit via daemon server: {host}")
        livekit_token, livekit_host = generate_token_via_daemon_server(room_name, identity, room_password, host)
        print(f"Connect to LiveKit server: {livekit_host}")
    else:
        print(f"Connect to LiveKit server directly: {LIVEKIT_URL}")
        livekit_token = generate_token(room_name, identity, identity)
        livekit_host = LIVEKIT_URL

    @room.on("participant_connected")
    def on_participant_connected(participant: rtc.Participant):
        print(f"Participant connected: {participant.identity}")

    @room.on("participant_disconnected")
    def on_participant_disconnected(participant: rtc.Participant):
        print(f"Participant disconnected: {participant.identity}")

    @room.on("track_published")
    def on_track_published(publication: rtc.RemoteTrackPublication, participant: rtc.RemoteParticipant):
        print(f"Track published: {publication.name} by {participant.identity}")

    @room.on("participant_attributes_changed")
    def on_attributes_changed(changed_attributes: dict[str, str], participant: rtc.Participant):
        print(
            f"participant {participant.identity} attributes changed: {participant.attributes} {changed_attributes}",
        )

    print(f"Connecting to {livekit_host}")
    try:
        await room.connect(livekit_host, livekit_token, rtc.RoomOptions(auto_subscribe=auto_subscribe, dynacast=dynacast))
        print(f"Connected to room {room.name}")
    except rtc.ConnectError as e:
        warnings.warn(f"Failed to connect to the room: {e}", stacklevel=2)
        return None
    return room

def generate_token_via_daemon_server(room_name: str, identity: str, room_password: str, host: str):
    api_path = "/room/login"
    url = f"{host}{api_path}"

    params = {
        "room_name": room_name,
        "identity": identity,
        "password": room_password
    }

    logger.info(f"Requesting token from: {url}")

    try:
        response = requests.post(url, json=params)

        status_code = response.status_code
        response_content = response.text

        # Parse JSON body
        try:
            data = response.json()
        except ValueError as e:
            # If non-200 and not JSON, raise HTTP error with content
            if status_code != 200:
                raise Exception(f"HTTP Error {status_code}: {response_content}") from e
            raise ValueError(f"Invalid JSON response: {response_content}") from e

        # Handle non-200 responses
        if status_code != 200:
            error_msg = f"HTTP Error {status_code}"
            # Extract structured error info if available
            if isinstance(data, dict):
                server_code = data.get("code", "unknown")
                server_error = data.get("error", "Unknown error")
                error_msg += f" | Code: {server_code} | Message: {server_error}"
            raise Exception(error_msg)

        # Success (200 OK) - Extract token and host
        token = None
        livekit_host = None

        # Handle potentially nested 'data' structure or flat structure
        target_data = data
        if isinstance(data, dict) and "data" in data and isinstance(data["data"], dict):
            target_data = data["data"]

        if isinstance(target_data, dict):
            token = target_data.get("token")
            livekit_host = target_data.get("host")

        if not token:
            raise ValueError(f"Token missing in response: {data}")
        if not livekit_host:
            raise ValueError(f"LiveKit host missing in response: {data}")

        return token, livekit_host

    except requests.RequestException as e:
        logger.error(f"Failed to generate token: {e}")
        raise e
    except Exception as e:
        logger.error(f"Failed to generate token: {e}")
        raise e

async def make_video_track(room):
    source = rtc.VideoSource(VIDEO_WIDTH, VIDEO_HEIGHT)
    track = rtc.LocalVideoTrack.create_video_track(VIDEO_TOPIC, source)
    track_options = rtc.TrackPublishOptions(
        source=rtc.TrackSource.SOURCE_CAMERA,
        simulcast=VIDEO_SIMULCAST,
        video_codec=rtc.VideoCodec.VP9,
        video_encoding=rtc.VideoEncoding(
            max_framerate=VIDEO_FPS,
            max_bitrate=VIDEO_MAX_BITRATE,
        ),
    )

    publication = await room.local_participant.publish_track(track, track_options)
    print(f"[VIDEO] published track {publication.name} to room {room.name}")

    return source, track


async def make_audio_track(room, sample_rate=SAMPLE_RATE, num_channels=NUM_CHANNELS, track_name=AUDIO_TOPIC):
    source = rtc.AudioSource(
        sample_rate=sample_rate,
        num_channels=num_channels,
        queue_size_ms=20,
    )
    track = rtc.LocalAudioTrack.create_audio_track(track_name, source)
    audio_options = rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE)
    publication = await room.local_participant.publish_track(track, audio_options)
    print(f"[AUDIO] published track {publication.name} to room {room.name}")

    return source, track


async def make_video_room(room):
    source, track = await make_video_track(room)

    def capture(frame: np.ndarray, timestamp: datetime | int | None = None, encoding="rgba"):
        """_summary_

        Args:
            frame (np.ndarray): The frame to capture.
            timestamp (datetime | int | None, optional): If int, it should be in microseconds. Defaults to None.
            encoding (str, optional): Options are nv12 and rgba. Defaults to "rgba".

        Raises:
            RuntimeError: If encoding is not supported.
        """
        memview = memoryview(frame)
        if timestamp is None:
            timestamp = get_timestamp_us()
        elif isinstance(timestamp, datetime):
            timestamp = int(timestamp.timestamp() * 1_000_000)
        if encoding == "rgba":
            source.capture_frame(
                rtc.VideoFrame(VIDEO_WIDTH, VIDEO_HEIGHT, rtc.VideoBufferType.RGBA, memview),
                timestamp_us=timestamp,
            )
        elif encoding == "nv12":
            source.capture_frame(
                rtc.VideoFrame(VIDEO_WIDTH, VIDEO_HEIGHT, rtc.VideoBufferType.NV12, memview),
                timestamp_us=timestamp,
            )
        else:
            raise RuntimeError(f"Unsupported image encoding: {encoding}")

    return capture


async def make_audio_room(room, sample_rate=48000, num_channels=1, frame_size_ms=20):
    """
    Create audio room setup optimized for microphone input and rtc.AudioStream consumption
    - 48kHz sample rate (required for AudioStream)
    - 20ms frames (960 samples per frame at 48kHz)
    - Matches the working implementation from live_microphone_client.py
    """
    source, track = await make_audio_track(room, sample_rate=sample_rate, num_channels=num_channels, track_name=AUDIO_TOPIC)

    # 20ms chunks for AudioStream compatibility (960 samples at 48kHz)
    chunk_size = (sample_rate * frame_size_ms) // 1000  # 960 samples for 48kHz, 320 for 16kHz

    async def capture(audio_data: np.ndarray, timestamp: datetime | int | None = None):
        """
        Capture audio data and send to LiveKit.

        Args:
            audio_data: Audio data as numpy array, can be:
                - float32 in range [-1.0, 1.0] (from sounddevice)
                - int16 PCM data (already converted)
            timestamp: Optional timestamp (not used currently)
        """
        if audio_data.shape[0] == 0:
            return

        # === Step 1: Convert to int16 if needed (like _microphone_callback) ===
        if audio_data.dtype == np.float32 or audio_data.dtype == np.float64:
            # Convert from float [-1.0, 1.0] to int16 [-32768, 32767]
            audio_data = (audio_data * 32767).astype(np.int16)
        elif audio_data.dtype != np.int16:
            # Ensure int16 format
            audio_data = audio_data.astype(np.int16)

        # === Step 2: Handle channel layout ===
        if audio_data.ndim == 1:
            # Already flat mono audio
            audio_flat = audio_data
        elif audio_data.ndim == 2:
            if num_channels == 1:
                # Convert stereo/multi-channel to mono by taking first channel or averaging
                audio_flat = audio_data[:, 0] if audio_data.shape[1] > 0 else audio_data.flatten()
            else:
                # Multi-channel audio, interleave
                audio_flat = audio_data.flatten()
        else:
            audio_flat = audio_data.flatten()

        # === Step 3: Process in 20ms chunks (exactly like _microphone_callback) ===
        for i in range(0, len(audio_flat), chunk_size):
            actual_chunk_size = min(chunk_size, len(audio_flat) - i)
            chunk = audio_flat[i : i + actual_chunk_size]

            # Pad incomplete frames to maintain consistent frame size (960 samples)
            if actual_chunk_size < chunk_size:
                padding = np.zeros(chunk_size - actual_chunk_size, dtype=np.int16)
                chunk = np.concatenate([chunk, padding])
                actual_chunk_size = chunk_size

            # Convert to bytes (int16 → bytes)
            chunk_bytes = chunk.tobytes()

            # === Step 4: Create AudioFrame (EXACTLY like _microphone_callback) ===
            frame = rtc.AudioFrame(
                data=chunk_bytes,
                sample_rate=sample_rate,
                num_channels=num_channels,
                samples_per_channel=actual_chunk_size,
            )

            # === Step 5: Send to LiveKit (NO APM processing) ===
            await source.capture_frame(frame)

    return capture


SENTINEL = object()


# class Receiver:
#     def __init__(
#         self,
#         room: rtc.Room,
#     ):
#         self.room = room

#         self._queue = asyncio.Queue()
#         self._stopped = False

#     def _on_data_received(self, data: rtc.DataPacket):
#         raise NotImplementedError

#     async def __anext__(self) -> tuple[str, datetime, dict]:
#         """Async generator that yields the next hand tracking data, including head_array and hand_array."""
#         res = await self._queue.get()
#         if res is SENTINEL:
#             raise StopAsyncIteration
#         topic, ts, payload = res
#         return topic, datetime.fromtimestamp(ts / 1_000), payload

#     def __aiter__(self):
#         return self

#     def stop(self):
#         if not self._stopped:
#             self._stopped = True
#             self._queue.put_nowait(SENTINEL)


# class BodyTrackingReceiver(Receiver):
#     def __init__(self, room: rtc.Room):
#         super().__init__(room)
#         self.shape = (2, 25, 7)
#         self.room.on("data_received", self._on_data_received)

#     def _on_data_received(self, data: rtc.DataPacket):
#         try:
#             if data.topic == "body_tracking":
#                 payload = msgpack.unpackb(data.data, object_hook=mnp.decode)  # (1 + 2 * 25) * 7

#                 timestamp = payload["timestamp"]
#                 head = np.array(payload["head"], dtype=np.float32)
#                 left_hand = np.array(payload["left_hand"], dtype=np.float32)
#                 right_hand = np.array(payload["right_hand"], dtype=np.float32)

#                 self._queue.put_nowait(
#                     (
#                         data.topic,
#                         timestamp,
#                         {"left_hand": left_hand.ravel(), "right_hand": right_hand.ravel(), "head": head.ravel()},
#                     )
#                 )

#                 # logger.trace(f"received hand tracking data {payload.keys()} from {data.participant}")

#         except msgpack.UnpackException:
#             logger.warning("Received data is not valid MessagePack")
#         except Exception as e:
#             logger.error(f"Failed to process hand tracking data: {e}")


# class JointStateReceiver(Receiver):
#     def __init__(self, room: rtc.Room):
#         super().__init__(room)
#         self.room.on("data_received", self._on_data_received)

#     def _on_data_received(self, data: rtc.DataPacket):
#         try:
#             if data.topic == "joint_state":
#                 payload = msgpack.unpackb(data.data, object_hook=mnp.decode)

#                 timestamp = payload["timestamp"]
#                 q_dict = dict(zip(payload["joint_names"], payload["qpos"], strict=True))

#                 self._queue.put_nowait((data.topic, timestamp, q_dict))

#                 # logger.trace(f"received joint state data {payload.keys()} from {data.participant}")

#         except msgpack.UnpackException:
#             logger.warning("Received data is not valid MessagePack")
#         except Exception as e:
#             logger.error(f"Failed to process hand tracking data: {e}")


async def change_attrbutes(identity: str, attributes: dict[str, str], room: str = ROOM_NAME):
    async with LiveKitAPI() as lkapi:
        try:
            await lkapi.room.update_participant(
                UpdateParticipantRequest(room=room, identity=identity, attributes=attributes)
            )
        except Exception as e:
            logger.error(f"Failed to update participant attributes: {e}")


async def update_participant_metadata(identity: str, metadata: str, room: str = ROOM_NAME):
    """Update participant metadata (as a JSON string)."""
    async with LiveKitAPI() as lkapi:
        try:
            await lkapi.room.update_participant(
                UpdateParticipantRequest(room=room, identity=identity, metadata=metadata)
            )
        except Exception as e:
            logger.error(f"Failed to update participant metadata: {e}")
