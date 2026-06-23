#!/usr/bin/env python3
"""
Lane Detector CV - Robotaksi TEKNOFEST 2026
===========================================
ROS1 referans kodundan alınan:
  - Beyaz renk filtresi (BGR: [190,109,109] - [255,255,255])
  - Contour tabanlı yaklaşım (ek olarak)
  
Pipeline:
  1. BGR -> Beyaz renk maskesi (inRange)
  2. Adaptive threshold (yerel kontrast)
  3. İkisini birleştir (OR)
  4. ROI trapezoid
  5. Canny + HoughLinesP
  6. Sol/Sağ fit + offset/angle
  7. EMA filtre + publish
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


class LaneDetectorCV(Node):

    def __init__(self):
        super().__init__("lane_detector_cv")

        # ─── BEYAZ RENK FİLTRESİ ──────────────────────────────────────
        # ROS1 referans: lower=[190,109,109], upper=[255,255,255]
        self.declare_parameter("white_lower_b", 180)
        self.declare_parameter("white_lower_g", 100)
        self.declare_parameter("white_lower_r", 100)
        self.declare_parameter("use_color_filter", True)

        # ─── ADAPTIVE THRESHOLD ───────────────────────────────────────
        self.declare_parameter("block_size", 51)
        self.declare_parameter("adapt_c",   -18)
        self.declare_parameter("blur_kernel", 5)

        # ─── CANNY ────────────────────────────────────────────────────
        self.declare_parameter("canny_low",  30)
        self.declare_parameter("canny_high", 80)

        # ─── ROI ──────────────────────────────────────────────────────
        self.declare_parameter("roi_horizon",      0.35)
        self.declare_parameter("roi_bottom_cut",   0.10)
        self.declare_parameter("roi_top_width",    0.50)
        self.declare_parameter("roi_bottom_width", 0.90)

        # ─── HOUGH ────────────────────────────────────────────────────
        self.declare_parameter("hough_threshold",       10)
        self.declare_parameter("hough_min_line_length", 20)
        self.declare_parameter("hough_max_line_gap",    80)

        # ─── SLOPE FİLTRE ─────────────────────────────────────────────
        self.declare_parameter("min_slope_abs", 0.3)
        self.declare_parameter("max_slope_abs", 8.0)

        # ─── FİLTRE ───────────────────────────────────────────────────
        self.declare_parameter("filter_alpha",     0.15)
        self.declare_parameter("min_fit_segments", 2)
        self.declare_parameter("min_total_length", 30.0)
        # Kavşak koruma: bir frame'de bu kadardan fazla açı/offset değişimi izin verme
        self.declare_parameter("angle_jump_max",  12.0)   # derece — yol bitiminde ani açı sıçramasını bastır
        self.declare_parameter("offset_jump_max",  0.20)  # normalize birim

        # ─── DEBUG ────────────────────────────────────────────────────
        self.declare_parameter("show_debug", True)

        self.get_logger().info("Lane Detector CV (Renk Filtreli) başlatıldı - Robotaksi 2026")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.callback, qos_profile_sensor_data
        )

        self.pub_img  = self.create_publisher(Image,   "/perception/lane_image_cv",  10)
        self.pub_off  = self.create_publisher(Float32, "/perception/lane_offset",     10)
        self.pub_ang  = self.create_publisher(Float32, "/perception/target_angle",    10)
        self.pub_conf = self.create_publisher(Float32, "/perception/lane_confidence", 10)
        self.pub_info = self.create_publisher(String,  "/perception/lane_info",       10)

        self.f_offset = 0.0
        self.f_angle  = 0.0
        self.f_conf   = 0.0
        self.counter  = 0

        self.lane_width_px          = 0.0
        self.lane_width_initialized = False

    def p(self, name):
        return self.get_parameter(name).value

    # ─── ROI ──────────────────────────────────────────────────────────

    def build_roi(self, H, W):
        top_y    = int(H * self.p("roi_horizon"))
        bot_y    = int(H * (1.0 - self.p("roi_bottom_cut")))
        cx       = W / 2.0
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

    # ─── FİT ──────────────────────────────────────────────────────────

    def fit_lane(self, segs, H, top_y):
        min_segs = self.p("min_fit_segments")
        min_len  = self.p("min_total_length")
        if len(segs) < min_segs:
            return None
        total_len = sum(s[4] for s in segs)
        if total_len < min_len:
            return None
        xs, ys, ws = [], [], []
        for x1, y1, x2, y2, l in segs:
            xs += [x1, x2]
            ys += [y1, y2]
            ws += [l, l]
        try:
            a, b = np.polyfit(ys, xs, 1, w=np.array(ws))
            return a * H + b, a * top_y + b, a
        except Exception:
            return None

    # ─── ANA CALLBACK ─────────────────────────────────────────────────

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'imgmsg_to_cv2 hatası: {e}', throttle_duration_sec=5.0)
            return
        H, W  = frame.shape[:2]
        debug = frame.copy()

        # ── 1. BEYAZ RENK FİLTRESİ ────────────────────────────────────
        # ROS1 referanstan: beyaz şeritler BGR=[190-255, 109-255, 109-255]
        if self.p("use_color_filter"):
            lower = np.array([
                self.p("white_lower_b"),
                self.p("white_lower_g"),
                self.p("white_lower_r")
            ], dtype=np.uint8)
            upper = np.array([255, 255, 255], dtype=np.uint8)
            color_mask = cv2.inRange(frame, lower, upper)
        else:
            color_mask = np.ones((H, W), dtype=np.uint8) * 255

        # ── 2. ADAPTIVE THRESHOLD ─────────────────────────────────────
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        k       = max(3, self.p("blur_kernel") | 1)
        gray    = cv2.GaussianBlur(gray, (k, k), 0)
        block   = max(3, self.p("block_size") | 1)
        binary  = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            self.p("adapt_c")
        )

        # ── 3. İKİSİNİ BİRLEŞTİR ─────────────────────────────────────
        combined = cv2.bitwise_or(color_mask, binary)

        # ── 4. ROI ────────────────────────────────────────────────────
        roi_mask, top_y, bot_y = self.build_roi(H, W)
        roi = cv2.bitwise_and(combined, roi_mask)

        # ── 5. CANNY + HOUGH ──────────────────────────────────────────
        edges = cv2.Canny(roi, self.p("canny_low"), self.p("canny_high"))

        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            threshold=self.p("hough_threshold"),
            minLineLength=self.p("hough_min_line_length"),
            maxLineGap=self.p("hough_max_line_gap")
        )

        # ── 6. SOL/SAĞ AYIR ───────────────────────────────────────────
        min_s = self.p("min_slope_abs")
        max_s = self.p("max_slope_abs")
        left_segs, right_segs = [], []

        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                dx, dy = float(x2 - x1), float(y2 - y1)
                if abs(dx) < 1e-6:
                    continue
                slope  = dy / dx
                abs_sl = abs(slope)
                if abs_sl < min_s or abs_sl > max_s:
                    continue
                length = math.hypot(dx, dy)
                seg    = (x1, y1, x2, y2, length)
                mid_x  = (x1 + x2) / 2.0

                # Eğim + konum bazlı ayrım
                if slope < 0 and mid_x < W * 0.65:
                    left_segs.append(seg)
                elif slope > 0 and mid_x > W * 0.35:
                    right_segs.append(seg)

        # ── 7. FİT ────────────────────────────────────────────────────
        lf = self.fit_lane(left_segs,  H, top_y)
        rf = self.fit_lane(right_segs, H, top_y)
        has_l, has_r = lf is not None, rf is not None

        # ── 8. LANE WIDTH ─────────────────────────────────────────────
        if has_l and has_r:
            w = abs(rf[0] - lf[0])
            if self.lane_width_initialized:
                self.lane_width_px = 0.9 * self.lane_width_px + 0.1 * w
            else:
                self.lane_width_px          = w
                self.lane_width_initialized = True

        half_lane = (self.lane_width_px / 2.0) if self.lane_width_initialized else (W * 0.25)

        # ── 9. OFFSET + ANGLE ─────────────────────────────────────────
        offset = self.f_offset
        angle  = self.f_angle

        if has_l and has_r:
            center = (lf[0] + rf[0]) / 2.0
            offset = (center - W / 2.0) / (W / 2.0)
            angle  = math.degrees(math.atan((lf[2] + rf[2]) / 2.0))
        elif has_l:
            offset = ((lf[0] + half_lane) - W / 2.0) / (W / 2.0)
            angle  = math.degrees(math.atan(lf[2]))
        elif has_r:
            offset = ((rf[0] - half_lane) - W / 2.0) / (W / 2.0)
            angle  = math.degrees(math.atan(rf[2]))

        # ── 10. CONFIDENCE ────────────────────────────────────────────
        total_len = (sum(s[4] for s in left_segs) +
                     sum(s[4] for s in right_segs))
        n_segs    = len(left_segs) + len(right_segs)

        if has_l or has_r:
            raw_conf = min(1.0, (n_segs / 8.0) * 0.5 + (total_len / 400.0) * 0.5)
            if has_l and has_r:
                raw_conf = min(1.0, raw_conf + 0.3)
        else:
            raw_conf = 0.0

        # ── 11. EMA FİLTRE + KAVŞAK KORUMASI ─────────────────────────
        a = self.p("filter_alpha")

        # Kavşak: önceki güven yüksekken ani sıçrama = yanlış algı
        # → alpha'yı düşür (çok yavaş güncelle), eski değeri koru
        if self.f_conf > 0.45:
            angle_jump  = abs(angle  - self.f_angle)
            offset_jump = abs(offset - self.f_offset)
            if angle_jump > self.p("angle_jump_max"):
                a = min(a, 0.04)   # çok yavaş güncelle
            if offset_jump > self.p("offset_jump_max"):
                a = min(a, 0.04)

        self.f_offset = (1 - a) * self.f_offset + a * offset
        self.f_angle  = (1 - a) * self.f_angle  + a * angle
        self.f_conf   = (1 - a) * self.f_conf   + a * raw_conf

        # ── 12. PUBLISH ───────────────────────────────────────────────
        self.pub_off.publish(Float32(data=float(self.f_offset)))
        self.pub_ang.publish(Float32(data=float(self.f_angle)))
        self.pub_conf.publish(Float32(data=float(self.f_conf)))
        self.pub_info.publish(String(data=json.dumps({
            "left":   has_l,
            "right":  has_r,
            "conf":   float(self.f_conf),
            "offset": float(self.f_offset),
            "angle":  float(self.f_angle)
        })))

        # ── 13. DEBUG ─────────────────────────────────────────────────
        if self.p("show_debug"):
            # ROI çerçeve
            half_top_px = int((W / 2) * self.p("roi_top_width"))
            half_bot_px = int((W / 2) * self.p("roi_bottom_width"))
            roi_pts     = np.array([
                [W // 2 - half_bot_px, bot_y],
                [W // 2 - half_top_px, top_y],
                [W // 2 + half_top_px, top_y],
                [W // 2 + half_bot_px, bot_y]
            ])
            cv2.polylines(debug, [roi_pts], True, (80, 80, 80), 2)

            # Segmentler
            for x1, y1, x2, y2, _ in left_segs:
                cv2.line(debug, (x1, y1), (x2, y2), (255, 100, 0), 2)
            for x1, y1, x2, y2, _ in right_segs:
                cv2.line(debug, (x1, y1), (x2, y2), (0, 100, 255), 2)

            # Fit çizgileri
            if lf:
                cv2.line(debug,
                         (int(lf[1]), top_y), (int(lf[0]), H),
                         (0, 255, 255), 4)  # cyan = sol
            if rf:
                cv2.line(debug,
                         (int(rf[1]), top_y), (int(rf[0]), H),
                         (255, 255, 0), 4)  # sarı = sağ

            # Merkez çizgisi
            if lf and rf:
                cx_bot = int((lf[0] + rf[0]) / 2)
                cx_top = int((lf[1] + rf[1]) / 2)
                cv2.line(debug, (cx_top, top_y), (cx_bot, H), (0, 255, 0), 2)

            # HUD
            cv2.putText(debug, f"Offset: {self.f_offset:+.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug, f"Angle:  {self.f_angle:+.1f}deg", (10, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug, f"Conf:   {self.f_conf:.2f}", (10, 86),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            status = ("L+R" if (has_l and has_r) else
                      "L"   if has_l else
                      "R"   if has_r else "KAYIP")
            color  = ((0, 255, 0)   if (has_l and has_r) else
                      (0, 165, 255) if (has_l or has_r)  else
                      (0, 0, 255))
            cv2.putText(debug, f"Serit: {status}", (10, 114),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if self.lane_width_initialized:
                cv2.putText(debug, f"LaneW: {self.lane_width_px:.0f}px", (10, 142),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # Renk maskesi PiP (sağ üst)
            pip_w      = min(160, W // 4)
            pip_h      = int(pip_w * H / W)
            mask_small = cv2.resize(
                cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR),
                (pip_w, pip_h)
            )
            px, py = W - pip_w - 10, 10
            debug[py:py + pip_h, px:px + pip_w] = mask_small
            cv2.rectangle(debug, (px, py), (px + pip_w, py + pip_h), (0, 255, 0), 1)
            cv2.putText(debug, "White", (px + 2, py + pip_h - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'debug yayın hatası: {e}', throttle_duration_sec=5.0)

        # Log
        self.counter += 1
        if self.counter % 30 == 0:
            self.get_logger().info(
                f"off={self.f_offset:+.2f} ang={self.f_angle:+.1f}° "
                f"L={has_l} R={has_r} conf={self.f_conf:.2f} "
                f"segs=({len(left_segs)},{len(right_segs)})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'lane_detector_cv beklenmedik hata: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()