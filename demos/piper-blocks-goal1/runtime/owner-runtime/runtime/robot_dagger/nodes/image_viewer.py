import os

import cv2
import numpy as np
from dora import Node


def main():
    node = Node()
    window_name = os.getenv("WINDOW_NAME", "GR3 Camera")
    width = int(os.getenv("IMAGE_WIDTH", "640"))
    height = int(os.getenv("IMAGE_HEIGHT", "400"))
    channels = int(os.getenv("IMAGE_CHANNELS", "3"))

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    print(f"图像查看器已启动，窗口名称: {window_name}")

    for event in node:
        if event["type"] != "INPUT" or event["id"] != "image":
            continue

        flat = event["value"].to_numpy(zero_copy_only=False)
        if flat.dtype != np.uint8:
            flat = flat.view(np.uint8) if flat.itemsize == 1 else flat.astype(np.uint8)

        try:
            image = flat.reshape(height, width, channels)
        except ValueError as error:
            print(f"解码图像失败: {error}")
            continue

        cv2.imshow(window_name, image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
