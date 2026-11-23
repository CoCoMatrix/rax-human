"""TODO: Add docstring."""

import os
from datetime import UTC, datetime

import cv2
import depthai as dai
import numpy as np
import pyarrow as pa
import pytz
from dora import Node

from dora_depthai.utils import FrequencyCounter

IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", "640"))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", "400"))
DRAW_TIMESTAMP = os.getenv("DRAW_TIMESTAMP", "0") == "1"
FPS = int(os.getenv("FPS", "30"))
FORMAT = os.getenv("FORMAT", "bgr")  # nv12
DEBUG = os.getenv("DEBUG", "0") == "1"


RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C


def main():
    """TODO: Add docstring."""

    node = Node()

    always_none = node.next(timeout=0.001) is None
    always_none = node.next(timeout=0.001) is None

    c = FrequencyCounter(period=5)

    pipeline = dai.Pipeline()

    # Define source and output
    camRgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET)

    rgbOut = camRgb.requestOutput(
        size=(IMAGE_WIDTH, IMAGE_HEIGHT),
        fps=FPS,
        type=dai.ImgFrame.Type.BGR888i,
        # enableUndistortion=True,
        # resizeMode=dai.ImgResizeMode.LETTERBOX,
    )
    queue = rgbOut.createOutputQueue()
    # Properties
    # camRgb = pipeline.create(dai.node.ColorCamera)
    # camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    # camRgb.setVideoSize(IMAGE_WIDTH, IMAGE_HEIGHT)

    # queue = camRgb.video.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        if not always_none:
            event_stream_is_none = node.next(timeout=0.001) is None
            if event_stream_is_none:
                break

        frame = queue.get()

        timestamp = datetime.now(UTC)
        device_time_ns = int(frame.getTimestamp().total_seconds() * 1e9)
        device_frame_index = int(frame.getSequenceNum())
        host_time_ns = int(dai.Clock.now().total_seconds() * 1e9)
        # print(f"{timestamp=},{(host_time_ns-device_time_ns)*1e-6=}ms")
        timestamp_ns = int(timestamp.astimezone(UTC).timestamp() * 1e9)

        if frame is None:
            frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "Error: Could not read frame from depthai camera.",
                (30, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                1,
            )

        if DEBUG:
            c.update("image")

        frame = frame.getCvFrame()

        if DRAW_TIMESTAMP:
            cv2.putText(
                frame,
                f"{timestamp.astimezone(pytz.timezone('Asia/Shanghai')).strftime('%H:%M:%S.%f')}",
                (50, frame.shape[0] // 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
                1,
            )

        metadata = {
            "width": frame.shape[1],
            "height": frame.shape[0],
            "timestamp_ns": timestamp_ns,
            "source_timestamp_ns": timestamp_ns,
            "target_execution_timestamp_ns": timestamp_ns,
            "device_frame_index": device_frame_index,
            "device_time_ns": device_time_ns,
            "host_time_ns": host_time_ns,
            "encoding": "bgr8",
        }

        node.send_output("image", pa.array(frame.ravel()), metadata)


if __name__ == "__main__":
    main()
