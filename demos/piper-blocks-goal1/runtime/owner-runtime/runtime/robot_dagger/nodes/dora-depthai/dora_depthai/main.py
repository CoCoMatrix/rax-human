"""TODO: Add docstring."""

import os
from datetime import UTC, datetime, timedelta

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
FPS = float(os.getenv("FPS", "30"))
ENCODING = os.getenv("ENCODING", "bgr8")
DEPTH_ENCODING = os.getenv("DEPTH_ENCODING", "z16")
DEBUG = os.getenv("DEBUG", "0") == "1"
ROTATE = int(os.getenv("ROTATE", "0"))
CAMERA_SERIAL = os.getenv("CAMERA_SERIAL", None)

USE_DEPTH = os.getenv("USE_DEPTH", "0") == "1"
VIS_DEPTH = os.getenv("VIS_DEPTH", "0") == "1"

RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C


def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    # Log the depth, minDepth and maxDepth
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        # Clip the values to be in the 0-255 range
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        # Interpolate only valid logDepth values, setting the rest based on the mask
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        # Set invalid depth pixels to black
        depthFrameColor[invalidMask] = 0
    except IndexError:
        # Frame is likely empty
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
    return depthFrameColor


def main():
    """TODO: Add docstring."""

    node = Node()

    always_none = node.next(timeout=0.001) is None
    always_none = node.next(timeout=0.001) is None

    c = FrequencyCounter(period=5)

    # Create device with specific serial number if specified
    if CAMERA_SERIAL:
        available_devices = dai.Device.getAllAvailableDevices()
        device_info = None
        for device in available_devices:
            if device.deviceId == CAMERA_SERIAL:
                device_info = device
                print(f"Found camera with serial number: {CAMERA_SERIAL}")
                break
        if device_info is None:
            available_serials = [d.deviceId for d in available_devices]
            raise RuntimeError(
                f"Camera with serial number '{CAMERA_SERIAL}' not found. Available devices: {available_serials}"
            )
        device = dai.Device(device_info)
    else:
        print("No CAMERA_SERIAL specified, using default camera")
        device = dai.Device()

    with dai.Pipeline(device) as p:
        # rgbd = p.create(dai.node.RGBD).build(
        #     autocreate=True, mode=dai.node.StereoDepth.PresetMode.DEFAULT, size=(IMAGE_WIDTH, IMAGE_HEIGHT), fps=FPS
        # )

        # rgbd_queue = rgbd.rgbd.createOutputQueue()
        # # pointcloud = rgbd.pcl.createOutputQueue()

        # Define sources and outputs
        camRgb = p.create(dai.node.Camera).build(RGB_SOCKET)
        if USE_DEPTH:
            left = p.create(dai.node.Camera).build(LEFT_SOCKET)
            right = p.create(dai.node.Camera).build(RIGHT_SOCKET)
            stereo = p.create(dai.node.StereoDepth)
            sync = p.create(dai.node.Sync)
            platform = p.getDefaultDevice().getPlatform()
            if platform == dai.Platform.RVC4:
                raise ValueError("RVC4 is not supported")

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
            # stereo.setExtendedDisparity(True)
            stereo.setSubpixel(True)
            # stereo.setLeftRightCheck(False)
            stereo.setRectifyEdgeFillColor(0)
            stereo.enableDistortionCorrection(True)

            sync.setSyncThreshold(timedelta(seconds=1 / (2 * FPS)))

            rgbOut = camRgb.requestOutput(
                size=(IMAGE_WIDTH, IMAGE_HEIGHT),
                fps=FPS,
                type=dai.ImgFrame.Type.RGB888i,
                enableUndistortion=True,
                resizeMode=dai.ImgResizeMode.LETTERBOX,
            )
            leftOut = left.requestOutput(size=(640, 400), fps=FPS)
            rightOut = right.requestOutput(size=(640, 400), fps=FPS)

            # Linking
            rgbOut.link(sync.inputs["rgb"])
            leftOut.link(stereo.left)
            rightOut.link(stereo.right)

            stereo.depth.link(sync.inputs["depth_aligned"])
            rgbOut.link(stereo.inputAlignTo)

            queue = sync.out.createOutputQueue(maxSize=1, blocking=False)
        else:
            queue = camRgb.requestOutput(
                size=(IMAGE_WIDTH, IMAGE_HEIGHT),
                fps=FPS,
                type=dai.ImgFrame.Type.BGR888i,
                enableUndistortion=True,
                resizeMode=dai.ImgResizeMode.LETTERBOX,
            ).createOutputQueue()
        p.start()

        while p.isRunning():
            if not always_none:
                event_stream_is_none = node.next(timeout=0.001) is None
                if event_stream_is_none:
                    break

            frames = queue.get()

            if USE_DEPTH:
                frame = frames["rgb"]
                depth = frames["depth_aligned"].getCvFrame()
            else:
                frame = frames
            timestamp = datetime.now(UTC)
            device_time_ns = int(frame.getTimestamp().total_seconds() * 1e9)
            device_frame_index = int(frame.getSequenceNum())
            host_time_ns = int(dai.Clock.now().total_seconds() * 1e9)
            # print(f"{timestamp=},{(host_time_ns-device_time_ns)*1e-6=}")
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

            if ROTATE == 0:
                pass
            elif ROTATE == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                if USE_DEPTH:
                    depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
            elif ROTATE == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                if USE_DEPTH:
                    depth = cv2.rotate(depth, cv2.ROTATE_180)
            elif ROTATE == -90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                if USE_DEPTH:
                    depth = cv2.rotate(depth, cv2.ROTATE_90_COUNTERCLOCKWISE)
            else:
                raise ValueError(f"Invalid ROTATE value: {ROTATE}")

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
                "encoding": ENCODING,
            }
            if ENCODING == "bgr8" or ENCODING == "bgr":
                frame = frame.ravel()
            elif ENCODING == "rgb" or ENCODING == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).ravel()
            elif ENCODING in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                frame = cv2.imencode(f".{ENCODING}", frame)[1]
            else:
                raise ValueError(f"Invalid ENCODING value: {ENCODING}")

            node.send_output("image", pa.array(frame), metadata)
            if USE_DEPTH:
                if VIS_DEPTH:
                    depth_color = colorizeDepth(depth)
                    metadata = {
                        "width": depth_color.shape[1],
                        "height": depth_color.shape[0],
                        "timestamp_ns": timestamp_ns,
                        "device_time_ns": device_time_ns,
                        "host_time_ns": host_time_ns,
                        "encoding": "bgr8",
                    }
                    node.send_output("depth_color", pa.array(depth_color.ravel()), metadata)
                metadata = {
                    "width": depth.shape[1],
                    "height": depth.shape[0],
                    "timestamp_ns": timestamp_ns,
                    "device_time_ns": device_time_ns,
                    "host_time_ns": host_time_ns,
                    "encoding": DEPTH_ENCODING,
                }
                if DEPTH_ENCODING in ["z16", "mono16"]:
                    depth = depth.astype(np.uint16)
                    depth = depth.ravel()
                elif DEPTH_ENCODING in ["png", "tif", "tiff"]:
                    depth = cv2.imencode(f".{DEPTH_ENCODING}", depth)[1]
                else:
                    raise ValueError(f"Invalid DEPTH_ENCODING value: {DEPTH_ENCODING}")
                node.send_output("depth", pa.array(depth), metadata)


if __name__ == "__main__":
    main()
