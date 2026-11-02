#!/usr/bin/env python3
"""Debug window: the camera image with the HSV detector's box.

Only watches topics and sends nothing, so it cannot affect the flight. The green box is
the one perception_node published for that exact frame.
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

WINDOW = 'HSV detection'
MAX_WAITING_IMAGES = 20
GREEN, RED, WHITE = (0, 220, 0), (0, 0, 255), (255, 255, 255)


def compose(rgb, box, t, frames, lost):
    """box is (u, v, w, h) with (u, v) the centre in pixels, or None for no detection."""
    camera = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if box is not None:
        u, v, w, h = box
        top_left = (int(round(u - w / 2)), int(round(v - h / 2)))
        bottom_right = (int(round(u + w / 2)), int(round(v + h / 2)))
        cv2.rectangle(camera, top_left, bottom_right, GREEN, 2)
        cv2.drawMarker(camera, (int(round(u)), int(round(v))), GREEN, cv2.MARKER_CROSS, 14, 2)
    status = 'DETECTED' if box is not None else 'NO DETECTION'
    cv2.putText(camera, f't={t:5.1f} s  lost {lost}/{frames}', (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2)
    cv2.putText(camera, status, (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN if box else RED, 2)
    return camera


class DetectionView:

    def __init__(self, node):
        self.images = {}
        self.latest = None
        self.frames = 0
        self.lost = 0
        # Best effort: this window must never hold back the publishers.
        image_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        box_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Image, '/camera/color/image_raw', self._on_image, image_qos)
        node.create_subscription(Detection2DArray, '/perception/target_bbox', self._on_box, box_qos)

    @staticmethod
    def _key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _on_image(self, msg):
        self.images[self._key(msg)] = msg
        while len(self.images) > MAX_WAITING_IMAGES:
            del self.images[min(self.images)]

    def _on_box(self, msg):
        key = self._key(msg)
        image = self.images.pop(key, None)
        if image is None:
            return
        for older in [k for k in self.images if k < key]:
            del self.images[older]
        self.frames += 1
        box = None
        if msg.detections:
            b = msg.detections[0].bbox
            box = (b.center.position.x, b.center.position.y, b.size_x, b.size_y)
        else:
            self.lost += 1
        rgb = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
        self.latest = compose(rgb, box, key[0] + key[1] * 1e-9, self.frames, self.lost)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('detection_view_node')
    view = DetectionView(node)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 640, 480)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if view.latest is not None:
                cv2.imshow(WINDOW, view.latest)
                view.latest = None
            cv2.waitKey(1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
