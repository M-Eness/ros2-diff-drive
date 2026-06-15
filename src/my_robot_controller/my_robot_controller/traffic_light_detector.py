import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import os
from ament_index_python.packages import get_package_share_directory

_pkg_share = get_package_share_directory('my_robot_controller')
MODEL_PATH = os.path.join(_pkg_share, 'models', 'weights', 'traffic_light_best.pt')
CONF_THRESHOLD = 0.5
CLASS_NAMES = {0: 'Green', 1: 'Red', 2: 'Yellow'}

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pub_detections = self.create_publisher(String, '/perception/traffic_light', 10)
        self.pub_red = self.create_publisher(Bool, '/perception/flags/traffic_light_red', 10)
        self.pub_green = self.create_publisher(Bool, '/perception/flags/traffic_light_green', 10)

        self.get_logger().info("Traffic Light Detector başlatıldı.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)[0]

        detections = []
        red = False
        green = False

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            name = CLASS_NAMES.get(cls_id, f"class_{cls_id}")
            detections.append({"class_id": cls_id, "class_name": name, "confidence": round(conf, 3), "bbox": [x1, y1, x2, y2]})

            if name == 'Red':
                red = True
            elif name == 'Green':
                green = True

        if detections:
            msg_out = String()
            msg_out.data = json.dumps(detections)
            self.pub_detections.publish(msg_out)
            self.get_logger().info(f"Işık: {[d['class_name'] for d in detections]}")

        self.pub_red.publish(Bool(data=red))
        self.pub_green.publish(Bool(data=green))

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
