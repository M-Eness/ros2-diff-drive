#!/usr/bin/env python3
"""
Trafik Lambası OpenCV Dedektörü — HSV renk tabanlı destekleyici sistem
YOLO detektörünü destekler, hafif ve hızlı çalışır.

Yayınlar:
  /perception/traffic_light_cv  (String JSON)
    {"class": "Red"|"Yellow"|"Green", "confidence": 0.0-1.0,
     "source": "cv", "area": int}
"""
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# HSV renk aralıkları  (H=0-179, S=0-255, V=0-255)
# Geniş tutuldu — simülasyonda ışık kafa kutusuna yansıyor, tam saf renk değil
COLOR_RANGES = {
    'Red': [
        (np.array([  0,  80,  80]), np.array([ 15, 255, 255])),
        (np.array([160,  80,  80]), np.array([179, 255, 255])),
    ],
    'Yellow': [
        (np.array([15,  80, 80]), np.array([40, 255, 255])),
    ],
    'Green': [
        (np.array([40,  60, 60]), np.array([95, 255, 255])),
    ],
}

MIN_AREA      = 40    # piksel² — simülasyonda lamba küçük görünebilir
MIN_CIRCULAR  = 0.15  # kafa kutusu dikdörtgen, dairesellik düşük olabilir
ROI_TOP_FRAC  = 0.70  # üst %70


class TrafficLightCVNode(Node):

    def __init__(self):
        super().__init__('traffic_light_cv')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._img_cb, 10)
        self.pub = self.create_publisher(
            String, '/perception/traffic_light_cv', 10)

        self.get_logger().info('TrafficLightCV başlatıldı — HSV tabanlı')

    def _img_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, w = frame.shape[:2]
        roi  = frame[:int(h * ROI_TOP_FRAC), :]
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        best_color = None
        best_area  = 0

        for color_name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * area / (perimeter ** 2)
                if circularity < MIN_CIRCULAR:
                    continue
                if area > best_area:
                    best_area  = area
                    best_color = color_name

        if best_color:
            # Alan büyüdükçe güven artar, max 0.90
            conf = min(0.90, 0.65 + best_area / 8000.0)
            self.pub.publish(String(data=json.dumps({
                'class':      best_color,
                'confidence': round(conf, 2),
                'source':     'cv',
                'area':       int(best_area),
            })))


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightCVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
