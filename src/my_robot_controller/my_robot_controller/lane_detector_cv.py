#!/usr/bin/env python3
"""
Lane Detector CV - Robotaksi TEKNOFEST 2026 (Fixed)
===================================================
DÜZELTMELER: adapt_c düşürüldü, ROI küçültüldü, f_conf güncelleniyor.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import math


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__("lane_detector_node")

        # ─── PARAMETRELER (İYİLEŞTİRİLDİ) ──────────────────────────
        self.declare_parameter("adapt_c", -8)       # Çok agresif değil
        self.declare_parameter("canny_low", 20)
        self.declare_parameter("canny_high", 50)
        self.declare_parameter("hough_threshold", 10)
        self.declare_parameter("hough_min_line_length", 15)
        self.declare_parameter("hough_max_line_gap", 80)
        self.declare_parameter("roi_horizon", 0.35)
        self.declare_parameter("roi_bottom_cut", 0.15)
        self.declare_parameter("roi_top_width", 0.50)
        self.declare_parameter("roi_bottom_width", 0.75) # 0.95'ten düşürüldü
        self.declare_parameter("show_debug", True)
        self.declare_parameter("filter_alpha", 0.15)

        self.get_logger().info("🚗 Classic Lane Detector (Fixed) başlatıldı")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.callback, qos_profile_sensor_data
        )

        self.pub_img   = self.create_publisher(Image,   "/perception/lane_image_cv", 10)
        self.pub_off   = self.create_publisher(Float32, "/perception/lane_offset", 10)
        self.pub_ang   = self.create_publisher(Float32, "/perception/target_angle", 10)
        self.pub_conf  = self.create_publisher(Float32, "/perception/lane_confidence", 10)
        self.pub_info  = self.create_publisher(String,  "/perception/lane_info", 10)

        self.f_offset = 0.0
        self.f_angle  = 0.0
        self.f_conf   = 0.0

    def p(self, name):
        return self.get_parameter(name).value

    def build_roi(self, H, W):
        """Trapezoid (yamuk) maske oluşturur."""
        top_y = int(H * self.p("roi_horizon"))
        bot_y = int(H * (1.0 - self.p("roi_bottom_cut")))
        cx = W / 2.0
        half_top = cx * self.p("roi_top_width")
        half_bot = cx * self.p("roi_bottom_width")

        pts = np.array([[
            [int(cx - half_bot), bot_y],
            [int(cx - half_top), top_y],
            [int(cx + half_top), top_y],
            [int(cx + half_bot), bot_y]
        ]], dtype=np.int32)

        mask = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(mask, pts, 255)
        return mask, top_y, bot_y

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        H, W = frame.shape[:2]
        debug = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, self.p("adapt_c")
        )

        roi_mask, top_y, bot_y = self.build_roi(H, W)
        roi = cv2.bitwise_and(binary, roi_mask)

        edges = cv2.Canny(roi, self.p("canny_low"), self.p("canny_high"))

        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            threshold=self.p("hough_threshold"),
            minLineLength=self.p("hough_min_line_length"),
            maxLineGap=self.p("hough_max_line_gap")
        )

        left_segs, right_segs = [], []
        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                dx, dy = x2 - x1, y2 - y1
                if abs(dx) < 1e-6: continue
                slope = dy / dx
                if abs(slope) < 0.3 or abs(slope) > 8.0: continue
                length = math.hypot(dx, dy)
                mid_x = (x1 + x2) / 2.0

                # DÜZELTİLDİ: Ortadan net bir şekilde ayır
                if mid_x < W / 2.0:
                    left_segs.append((x1, y1, x2, y2, length))
                else:
                    right_segs.append((x1, y1, x2, y2, length))

        def fit(segs):
            # DÜZELTİLDİ: Tek segment varsa da kabul et (len >= 1)
            if len(segs) < 1: return None
            xs, ys, ws = [], [], []
            for x1, y1, x2, y2, l in segs:
                xs += [x1, x2]
                ys += [y1, y2]
                ws += [l, l]
            try:
                a, b = np.polyfit(ys, xs, 1, w=np.array(ws))
                return a * H + b, a * top_y + b, a
            except:
                return None

        lf = fit(left_segs)
        rf = fit(right_segs)

        has_l, has_r = lf is not None, rf is not None

        # DÜZELTİLDİ: f_conf artık burada güncelleniyor
        self.f_conf = 1.0 if (has_l or has_r) else 0.0

        offset = self.f_offset
        angle = self.f_angle
        if has_l and has_r:
            center = (lf[0] + rf[0]) / 2.0
            offset = (center - W / 2.0) / (W / 2.0)
            angle = math.degrees(math.atan((lf[2] + rf[2]) / 2.0))
        elif has_l:
            offset = ((lf[0] + W * 0.25) - W / 2.0) / (W / 2.0)
            angle = math.degrees(math.atan(lf[2]))
        elif has_r:
            offset = ((rf[0] - W * 0.25) - W / 2.0) / (W / 2.0)
            angle = math.degrees(math.atan(rf[2]))

        alpha = self.p("filter_alpha")
        self.f_offset = (1 - alpha) * self.f_offset + alpha * offset
        self.f_angle = (1 - alpha) * self.f_angle + alpha * angle

        self.pub_off.publish(Float32(data=float(self.f_offset)))
        self.pub_ang.publish(Float32(data=float(self.f_angle)))
        self.pub_conf.publish(Float32(data=float(self.f_conf)))
        
        info_dict = {
            "left": has_l,
            "right": has_r,
            "conf": float(self.f_conf),
            "offset": float(self.f_offset),
            "angle": float(self.f_angle)
        }
        self.pub_info.publish(String(data=json.dumps(info_dict)))

        if self.p("show_debug"):
            half_top = int((W / 2) * 0.50)
            half_bot = int((W / 2) * 0.75)
            roi_pts = np.array([[W//2 - half_bot, bot_y], [W//2 - half_top, top_y], [W//2 + half_top, top_y], [W//2 + half_bot, bot_y]])
            cv2.polylines(debug, [roi_pts], True, (80, 80, 80), 2)

            for x1, y1, x2, y2, _ in left_segs:
                cv2.line(debug, (x1, y1), (x2, y2), (255, 0, 0), 2)
            for x1, y1, x2, y2, _ in right_segs:
                cv2.line(debug, (x1, y1), (x2, y2), (0, 0, 255), 2)

            if lf:
                cv2.line(debug, (int(lf[1]), top_y), (int(lf[0]), H), (255, 255, 0), 4)
            if rf:
                cv2.line(debug, (int(rf[1]), top_y), (int(rf[0]), H), (0, 255, 255), 4)

            cv2.putText(debug, f"Offset: {self.f_offset:+.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug, f"Angle:  {self.f_angle:+.1f}°", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            status = "L+R" if (has_l and has_r) else ("L" if has_l else ("R" if has_r else "KAYIP"))
            color  = (0, 255, 0) if (has_l and has_r) else (0, 165, 255) if (has_l or has_r) else (0, 0, 255)
            cv2.putText(debug, f"Serit: {status}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            self.pub_img.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

        if hasattr(self, 'counter') and self.counter % 30 == 0:
            self.get_logger().info(f"off={self.f_offset:+.2f} ang={self.f_angle:+.1f}° L={has_l} R={has_r} conf={self.f_conf:.2f}")
        self.counter = getattr(self, 'counter', 0) + 1


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()