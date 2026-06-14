import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json

CLASS_NAMES = [
    "yaya_gecidi", "ada_etrafinda_don", "trafik_isiklari",
    "saga_donulmez", "sola_donulmez", "girilmez",
    "sagdan_gidiniz", "soldan_gidiniz", "saga_mecburi_yon",
    "sola_mecburi_yon", "ileri_mecburi_yon", "ileri_ve_saga_mecburi_yon",
    "ileri_ve_sola_mecburi_yon", "ileriden_saga_mecburi_yon",
    "ileriden_sola_mecburi_yon", "serit_duzenleme_ileri_sola",
    "serit_duzenleme_ileri_saga", "iki_yonlu_yol",
    "park_etmek_yasaktir", "park_yeri", "tunel", "durak", "dur"
]

MODEL_PATH = "/home/rana/runs/detect/train/weights/best.pt"
CONF_THRESHOLD = 0.5

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        
        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()
        
        # ZED 2 kamera topic'i
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Detection sonuçları
        self.publisher = self.create_publisher(String, '/traffic_sign_detections', 10)
        
        self.get_logger().info("Traffic Sign Detector başlatıldı.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        
        detections = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"class_{cls_id}"
            detections.append({
                "class_id": cls_id,
                "class_name": name,
                "confidence": round(conf, 3),
                "bbox": [x1, y1, x2, y2]
            })
        
        if detections:
            msg_out = String()
            msg_out.data = json.dumps(detections)
            self.publisher.publish(msg_out)
            self.get_logger().info(f"Tespit: {[d['class_name'] for d in detections]}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
