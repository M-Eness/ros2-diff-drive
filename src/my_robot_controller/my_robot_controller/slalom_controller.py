#!/usr/bin/env python3
"""
Slalom Controller v1.0 — Robotaksi TEKNOFEST 2026
==================================================
Turuncu/Sarı konileri algılayarak slalom parkurunu geçer.
Mevcut mimariyi bozmaz; /cmd_vel_slalom topiğine Twist yayınlar.
"""

import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class SlalomController(Node):

    def __init__(self):
        super().__init__("slalom_controller")

        # ─── KONİ RENK ARALIĞI (HSV) ────────────────────────────────
        self.declare_parameter("cone_hue_low",    5)
        self.declare_parameter("cone_hue_high",  25)
        self.declare_parameter("cone_sat_low",   120)
        self.declare_parameter("cone_sat_high",  255)
        self.declare_parameter("cone_val_low",   100)
        self.declare_parameter("cone_val_high",  255)

        # ─── PERSPEKTİF / KAMERA ────────────────────────────────────
        self.declare_parameter("camera_fov_h",   62.2)
        self.declare_parameter("camera_fov_v",   48.8)
        self.declare_parameter("cone_real_h",    0.20)
        self.declare_parameter("camera_height",  0.25)
        self.declare_parameter("camera_tilt",    10.0)

        # ─── SÜRÜŞ ──────────────────────────────────────────────────
        self.declare_parameter("target_speed",   0.8)
        self.declare_parameter("min_speed",      0.3)
        self.declare_parameter("max_steering",   0.5)
        self.declare_parameter("lookahead_dist", 0.6)
        self.declare_parameter("wheel_base",     1.2)

        # ─── KAPI GEÇİŞ (Geriye dönük uyumluluk için duruyor) ───────
        self.declare_parameter("gate_width_min", 0.3)
        self.declare_parameter("gate_width_max", 1.5)
        self.declare_parameter("pass_threshold", 0.3)

        # ─── SLALOM AYARLARI ────────────────────────────────────────
        self.declare_parameter("slalom_offset",     0.9)  # metre cinsinden yana kayma mesafesi
        self.declare_parameter("switch_cone_dist",  1.8)  # bir sonraki dubaya geçiş algılama mesafesi

        # ─── DEBUG ──────────────────────────────────────────────────
        self.declare_parameter("show_debug", True)

        self.get_logger().info("Slalom Controller v1.0 başlatıldı")

        self.sub_img = self.create_subscription(
            Image, "/camera/image_raw", self._image_cb, 10
        )
        self.sub_mission = self.create_subscription(
            String, "/mission/command", self._mission_cb, 10
        )

        self.pub_cmd   = self.create_publisher(Twist, "/cmd_vel_slalom", 10)
        self.pub_debug = self.create_publisher(Image, "/slalom/debug", 10)
        self.pub_mission = self.create_publisher(String, "/mission/command", 10)

        self.bridge  = CvBridge()
        self.enabled = False
        self.frame   = None
        self.counter = 0

        self.state          = "SCAN"
        self.slalom_direction   = 1    # 1: solundan geç, -1: sağından geç
        self.prev_closest_x     = None
        self.passed_cones_count = 0
        self.last_cone_seen_time = None

        self.create_timer(0.05, self._control_loop)

    def _p(self, name):
        return self.get_parameter(name).value

    # ─── MİSYON KOMUTU ────────────────────────────────────────────────

    def _mission_cb(self, msg: String):
        if msg.data == "SLALOM_START":
            self.enabled = True
            self.state   = "SCAN"
            self.slalom_direction   = 1
            self.prev_closest_x     = None
            self.passed_cones_count = 0
            self.last_cone_seen_time = self.get_clock().now()
            self.get_logger().info("Slalom modu aktif (Tek sıra slalom sürümü)")
        elif msg.data == "SLALOM_STOP":
            self.enabled = False
            self.get_logger().info("Slalom modu durduruldu")

    # ─── GÖRÜNTÜ İŞLEME ──────────────────────────────────────────────

    def _image_cb(self, msg):
        if not self.enabled:
            return
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _detect_cones(self, img):
        hsv   = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self._p("cone_hue_low"),  self._p("cone_sat_low"),  self._p("cone_val_low")],  dtype=np.uint8)
        upper = np.array([self._p("cone_hue_high"), self._p("cone_sat_high"), self._p("cone_val_high")], dtype=np.uint8)
        mask  = cv2.inRange(hsv, lower, upper)
        mask  = cv2.erode(mask,  None, iterations=1)
        mask  = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cone_points = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:
                continue
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cone_points.append((cx, cy, area))
        return mask, cone_points

    def _pixel_to_world(self, px, py, img_w, img_h):
        fov_h     = math.radians(self._p("camera_fov_h"))
        f_px      = (img_w / 2.0) / math.tan(fov_h / 2.0)
        tilt      = math.radians(self._p("camera_tilt"))
        cam_h     = self._p("camera_height")

        v0             = img_h / 2.0
        angle_to_cone  = tilt + math.atan((py - v0) / f_px)
        if angle_to_cone <= 0.001:
            x_forward = 5.0
        else:
            x_forward = cam_h / math.tan(angle_to_cone)

        dx_px     = px - img_w / 2.0
        y_lat     = -dx_px * x_forward / f_px
        return x_forward, y_lat

    def _find_next_gate(self, cones):
        if len(cones) < 2:
            return None

        best_dist = float('inf')
        best_gate = None
        for i in range(len(cones)):
            for j in range(i + 1, len(cones)):
                x1, y1 = cones[i][0], cones[i][1]
                x2, y2 = cones[j][0], cones[j][1]
                if x1 < 0.2 or x2 < 0.2:
                    continue
                width = abs(y1 - y2)
                if width < self._p("gate_width_min") or width > self._p("gate_width_max"):
                    continue
                cx   = (x1 + x2) / 2.0
                cy   = (y1 + y2) / 2.0
                dist = math.hypot(cx, cy)
                if dist < best_dist:
                    best_dist = dist
                    best_gate = (cx, cy)
        return best_gate

    # ─── KONTROL DÖNGÜSÜ ──────────────────────────────────────────────

    def _control_loop(self):
        if not self.enabled or self.frame is None:
            self.pub_cmd.publish(Twist())
            return

        debug_img = self.frame.copy()
        H, W      = debug_img.shape[:2]

        mask, cone_px = self._detect_cones(self.frame)

        # 1. Duba koordinatlarını hesapla ve filtrele
        cones_world = []
        for (px, py, area) in cone_px:
            x, y = self._pixel_to_world(px, py, W, H)
            # Robotun önünde (0.2m ile 8.0m arasında) ve şerit sınırlarında (y yanal genişliği 2.0m'den küçük)
            if 0.2 < x < 8.0 and abs(y) < 2.0:
                cones_world.append((x, y, px, py))

        # 2. Algılanan duba sayısına göre karar ver
        if len(cones_world) == 0:
            # Görünürde duba yoksa
            # Eğer en az 3 duba geçtiysek ve 1.5 saniyedir duba görmüyorsak slalom bitmiştir!
            now = self.get_clock().now()
            if self.passed_cones_count >= 3:
                time_since_last_cone = 0.0
                if self.last_cone_seen_time is not None:
                    time_since_last_cone = (now - self.last_cone_seen_time).nanoseconds / 1e9
                if time_since_last_cone > 1.5:
                    self.get_logger().warn(f"Tüm dubalar geçildi (Toplam: {self.passed_cones_count}) → SLALOM_STOP gönderiliyor.")
                    stop_msg = String()
                    stop_msg.data = "SLALOM_STOP"
                    self.pub_mission.publish(stop_msg)
                    self.enabled = False
                    self.pub_cmd.publish(Twist())
                    return

            # Arama/Scan durumu: Dubaları aramak ve yolu ortalamak için hafif sol/dönüş hareketi yap
            twist = Twist()
            twist.linear.x  = self._p("min_speed")
            # Robot henüz dönüşü tamamlamamış olabileceği için arama yaparken sola dönmeyi tamamlaması istenir (yaw_rate = 0.40)
            twist.angular.z = 0.40
            self.pub_cmd.publish(twist)
            self.state = "SCAN"
        else:
            # En az bir duba görüyoruz. Zaman damgasını güncelle
            self.last_cone_seen_time = self.get_clock().now()

            # Dubaları X mesafelerine göre sırala
            cones_world.sort(key=lambda c: c[0])
            x0, y0 = cones_world[0][0], cones_world[0][1]

            # 3. Duba geçiş algılama (X değerindeki sıçramayı kontrol et)
            if self.prev_closest_x is not None:
                # Eğer en yakın dubanın X'i bir önceki kareye göre aniden arttıysa (örn. > 2.0m),
                # bu durum en yakın dubayı geçtiğimizi ve filtrenin dışına çıktığını gösterir.
                if x0 - self.prev_closest_x > 2.0:
                    self.slalom_direction = -self.slalom_direction
                    self.passed_cones_count += 1
                    self.get_logger().warn(
                        f"Duba geçildi! Toplam: {self.passed_cones_count} | Yeni Yön: {'SOL' if self.slalom_direction == 1 else 'SAG'}"
                    )
            self.prev_closest_x = x0

            # 4. Hedef duba seçimi ve kayma yönü
            # Eğer 2 veya daha fazla duba görüyorsak ve en yakın duba geçiş mesafesinin altına indiyse,
            # bir sonraki dubayı hedef almaya başlarız.
            if len(cones_world) >= 2 and x0 < self._p("switch_cone_dist"):
                target_x = cones_world[1][0]
                target_y = cones_world[1][1]
                # İkinci dubanın etrafından dolanmak için ilk dubanın tersi yönüne kayıyoruz
                target_dir = -self.slalom_direction
                self.state = f"APPROACH_CONE_2 ({'SOL' if target_dir == 1 else 'SAG'})"
            else:
                target_x = x0
                target_y = y0
                target_dir = self.slalom_direction
                self.state = f"APPROACH_CONE_1 ({'SOL' if target_dir == 1 else 'SAG'})"

            # 5. Yanal hedef sapması (Slalom Offset) hesapla
            target_y_steer = target_y + target_dir * self._p("slalom_offset")

            # 6. Pure Pursuit Direksiyon Kontrolü
            L            = self._p("lookahead_dist")
            target_angle = math.atan2(target_y_steer, target_x)
            steering     = math.atan2(2.0 * self._p("wheel_base") * math.sin(target_angle), L)
            steering     = max(-self._p("max_steering"), min(self._p("max_steering"), steering))

            # 7. Sürüş komutunu yayınla
            twist = Twist()
            twist.linear.x  = self._p("target_speed")
            twist.angular.z = steering
            self.pub_cmd.publish(twist)

            # Debug çizimleri
            if self._p("show_debug"):
                mask_rgb  = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                debug_img = cv2.addWeighted(debug_img, 0.7, mask_rgb, 0.3, 0)
                
                # Algılanan tüm dubaları işaretle
                for i, (cx_w, cy_w, px, py) in enumerate(cones_world):
                    color = (0, 0, 255) if i == 0 else (0, 255, 255) # En yakın kırmızı, diğerleri sarı
                    cv2.circle(debug_img, (int(px), int(py)), 8, color, -1)
                    cv2.putText(debug_img, f"{cx_w:.1f}m", (int(px) + 10, int(py) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # Hedef noktasını ekrana yaz
                cv2.putText(debug_img, f"Hedef X: {target_x:.2f} Y_steer: {target_y_steer:.2f}", 
                            (W // 2 - 120, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(debug_img, f"[{self.state}] Gecilen: {self.passed_cones_count}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

        self.counter += 1
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"[{self.state}] Gecilen duba: {self.passed_cones_count}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlalomController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
