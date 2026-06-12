#!/usr/bin/env python3
"""
Lane Detection Node — UFLD (Ultra-Fast Lane Detection)
-------------------------------------------------------
Subscribes : /camera/image_raw         (sensor_msgs/Image)
Publishes  : /perception/lane_detection (std_msgs/String — JSON)
             /perception/lane_image     (sensor_msgs/Image — debug görsel)

Model: cfzd/Ultra-Fast-Lane-Detection — ResNet-18, TuSimple
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import numpy as np
import json
import os

try:
    from cv_bridge import CvBridge
except ImportError:
    raise RuntimeError("cv_bridge bulunamadı.")

import torch
import torch.nn.functional as F
from my_robot_controller.ufld_model import parsingNet

# Model varsayılan yolu — ament_index ile paketin share dizininden bul
try:
    from ament_index_python.packages import get_package_share_directory
    _PKG_DIR = get_package_share_directory('my_robot_controller')
except Exception:
    _PKG_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),  # my_robot_controller/ klasörü
    'weights', 'best.pth'
)

# TuSimple row anchors (56 adet)
TUSIMPLE_ROW_ANCHORS = [
    64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112,
    116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160,
    164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208,
    212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256,
    260, 264, 268, 272, 276, 280, 284, 288
]

INPUT_W = 800
INPUT_H = 288
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class LaneDetectionNode(Node):

    def __init__(self):
        super().__init__('lane_detection_node')

        # ---------- Parametreler ----------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('debug_image', True)
        self.declare_parameter('griding_num', 100)
        self.declare_parameter('num_lanes', 4)
        self.declare_parameter('confidence_threshold', 0.6)

        image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        model_path    = self.get_parameter('model_path').get_parameter_value().string_value
        self.debug    = self.get_parameter('debug_image').get_parameter_value().bool_value
        griding_num   = self.get_parameter('griding_num').get_parameter_value().integer_value
        num_lanes     = self.get_parameter('num_lanes').get_parameter_value().integer_value
        self.conf_thr = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # ---------- Cihaz ----------
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Cihaz: {self.device}')

        # ---------- Model ----------
        self.model = parsingNet(size=(288, 800), pretrained=False, backbone="18", cls_dim=(griding_num + 1, 56, num_lanes), use_aux=False)
        self._load_model(model_path)
        self.model.to(self.device)
        self.model.eval()

        self.griding_num = griding_num
        self.num_lanes   = num_lanes

        # ---------- Publisher / Subscriber ----------
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pub_lanes = self.create_publisher(String, '/perception/lane_detection', 10)
        if self.debug:
            self.pub_debug = self.create_publisher(Image, '/perception/lane_image', 10)

        self.bridge = CvBridge()
        self.get_logger().info(f'LaneDetectionNode (UFLD) başlatıldı — topic: {image_topic}')
        self.get_logger().info(f'Model yolu: {model_path}')

    def _load_model(self, model_path: str):
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model bulunamadı: {model_path}')
            raise FileNotFoundError(f'{model_path} bulunamadı.')

        state = torch.load(model_path, map_location='cpu')
        if 'model' in state:
            state = state['model']
        state = {k.replace('module.', ''): v for k, v in state.items()}
        self.model.load_state_dict(state, strict=False)
        self.get_logger().info('Model ağırlıkları yüklendi.')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge hatası: {e}')
            return

        orig_h, orig_w = frame.shape[:2]
        tensor = self._preprocess(frame)

        with torch.no_grad():
            output = self.model(tensor)

        lanes = self._decode(output, orig_w, orig_h)

        result = {'lanes': lanes, 'num_detected': sum(1 for l in lanes if l)}
        lane_msg = String()
        lane_msg.data = json.dumps(result)
        self.pub_lanes.publish(lane_msg)

        if self.debug:
            debug_frame = self._draw(frame.copy(), lanes)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_msg.header = msg.header
                self.pub_debug.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f'Debug publish hatası: {e}')

    def _preprocess(self, frame: np.ndarray) -> torch.Tensor:
        img = cv2.resize(frame, (INPUT_W, INPUT_H))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = (img - MEAN) / STD
        img = img.transpose(2, 0, 1)
        tensor = torch.from_numpy(img).unsqueeze(0)
        return tensor.to(self.device)

    def _decode(self, output: torch.Tensor, orig_w: int, orig_h: int):
        output = output.squeeze(0)  # (56, 4, 101)
        prob = F.softmax(output[:, :, :-1], dim=2)
        max_prob, col_idx = prob.max(dim=2)

        col_sample = np.linspace(0, INPUT_W - 1, self.griding_num)
        col_sample_w = col_sample[1] - col_sample[0]

        lanes = []
        for lane_idx in range(self.num_lanes):
            points = []
            for row_idx in range(len(TUSIMPLE_ROW_ANCHORS)):
                conf = max_prob[row_idx, lane_idx].item()
                if conf < self.conf_thr:
                    continue
                x_ufld = col_idx[row_idx, lane_idx].item() * col_sample_w
                y_ufld = TUSIMPLE_ROW_ANCHORS[row_idx]
                x_orig = int(x_ufld * orig_w / INPUT_W)
                y_orig = int(y_ufld * orig_h / INPUT_H)
                points.append({'x': x_orig, 'y': y_orig, 'conf': round(conf, 3)})
            lanes.append(points if len(points) >= 2 else [])

        return lanes

    def _draw(self, frame: np.ndarray, lanes: list) -> np.ndarray:
        colors = [(0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
        for lane_idx, points in enumerate(lanes):
            if not points:
                continue
            color = colors[lane_idx % len(colors)]
            pts = [(p['x'], p['y']) for p in points]
            for i in range(len(pts) - 1):
                cv2.line(frame, pts[i], pts[i + 1], color, 3)
            for pt in pts:
                cv2.circle(frame, pt, 4, color, -1)
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()