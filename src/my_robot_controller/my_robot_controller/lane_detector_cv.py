import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class LaneDetectorCV(Node):
    def __init__(self):
        super().__init__('lane_detector_cv')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.pub_debug = self.create_publisher(Image, '/perception/lane_image_cv', 10)
        self.pub_offset = self.create_publisher(Float32, '/perception/lane_offset', 10)
        self.pub_info = self.create_publisher(String, '/perception/lane_info', 10)

        self.get_logger().info("Lane Detector (OpenCV) başlatıldı.")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # ROI — alt yarı
        roi = frame[h//2:, :]

        # Gri + Blur + Canny
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # Hough
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30,
                                 minLineLength=30, maxLineGap=20)

        left_x = []
        right_x = []
        debug = frame.copy()

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                cx = (x1 + x2) / 2

                if slope < -0.3 and cx < w / 2:
                    left_x.append(cx)
                    cv2.line(debug, (x1, y1 + h//2), (x2, y2 + h//2), (0, 255, 0), 2)
                elif slope > 0.3 and cx > w / 2:
                    right_x.append(cx)
                    cv2.line(debug, (x1, y1 + h//2), (x2, y2 + h//2), (0, 0, 255), 2)

        # Merkez hesapla
        offset = 0.0
        info = {"left": bool(left_x), "right": bool(right_x), "offset": 0.0}

        if left_x and right_x:
            lane_center = (np.mean(left_x) + np.mean(right_x)) / 2
            offset = float(lane_center - w / 2) / (w / 2)
            info["offset"] = round(offset, 3)
            cv2.line(debug, (int(lane_center), h//2), (int(lane_center), h), (255, 255, 0), 3)
        elif left_x:
            offset = -0.5
            info["offset"] = offset
        elif right_x:
            offset = 0.5
            info["offset"] = offset

        # Publish
        self.pub_offset.publish(Float32(data=offset))

        info_msg = String()
        info_msg.data = json.dumps(info)
        self.pub_info.publish(info_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        self.pub_debug.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
