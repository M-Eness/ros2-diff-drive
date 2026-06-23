#!/usr/bin/env python3
"""
================================================================================
Advanced Lane Controller v3.2 - Robotaksi TEKNOFEST 2026
================================================================================
Tümleşik Özellikler:
  - Engel algılama (Obstacle) entegrasyonu: STOP / SLOW / CLEAR
  - Şerit kaybolursa DÖNEREK ARA (Spin & Sweep) mantığı
  - Dönüş açıları ve kazançlar agresifleştirildi (Daha hızlı dönüş)
================================================================================
"""

import math
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

_OBSTACLE_TIMEOUT_S = 2.0   # bu kadar süre mesaj gelmezse CLEAR say


class LaneControllerNode(Node):

    def __init__(self):
        super().__init__("lane_controller_node")

        # ─── PID ──────────────────────────────────────────────────────
        self.declare_parameter("kp",  0.28)
        self.declare_parameter("ki",  0.002)
        self.declare_parameter("kd",  0.04)
        self.declare_parameter("kff", 0.12)  # feedforward

        # ─── LOOKAHEAD ────────────────────────────────────────────────
        self.declare_parameter("lookahead_base",  0.03)
        self.declare_parameter("lookahead_speed", 0.05)

        # ─── AÇI + OFFSET KOMPANSASYONU ───────────────────────────────
        self.declare_parameter("k_angle",       0.35)
        self.declare_parameter("k_offset_pred", 0.12)

        # ─── HIZ ──────────────────────────────────────────────────────
        self.declare_parameter("target_speed",       5.0)
        self.declare_parameter("min_speed",          0.60)
        self.declare_parameter("recovery_speed",     1.5)
        self.declare_parameter("speed_filter_alpha", 0.12)

        # ─── GÜVENİLİRLİK ────────────────────────────────────────────
        self.declare_parameter("confidence_threshold", 0.30)

        # ─── DİREKSİYON ───────────────────────────────────────────────
        self.declare_parameter("max_steer_straight", 0.40)
        self.declare_parameter("max_steer_turn",     0.60)  # 🔥 0.52'den 0.60'a çıkarıldı (Daha keskin dönüş)
        self.declare_parameter("steer_rate_limit",   0.08)   # daha hızlı direksiyon tepkisi
        self.declare_parameter("steer_filter_alpha", 0.35)   # daha çabuk güncelleme

        # ─── TEK ŞERİT ────────────────────────────────────────────────
        self.declare_parameter("single_lane_yaw",  0.40)  # 🔥 0.20'den 0.40'a çıkarıldı (Daha güçlü dönüş)
        self.declare_parameter("single_lane_gain", 0.45)  # 🔥 0.30'dan 0.45'e çıkarıldı (Offset'e daha hızlı tepki)
        self.declare_parameter("info_timeout",     1.5)

        # ─── DÖNEREK BULMA (SEARCH & SWEEP) ──────────────────────────
        self.declare_parameter("search_yaw_rate",    0.75)
        self.declare_parameter("search_switch_time", 2.0)
        # Kavşak geçişi: her iki şerit kaybolunca önce bu kadar süre düz git
        self.declare_parameter("intersection_straight_s", 2.5)
        self.declare_parameter("straight_lock_duration",  1.2)

        # ─── DEADBAND ─────────────────────────────────────────────────
        self.declare_parameter("error_deadband", 0.025)  # küçük sapmaları yoksay → titreme önler

        # ─── INTEGRAL ─────────────────────────────────────────────────
        self.declare_parameter("integral_max",   3.5)
        self.declare_parameter("integral_decay", 0.90)

        # ─── STATE MACHINE ────────────────────────────────────────────
        self.declare_parameter("turn_enter_thresh", 0.25)
        self.declare_parameter("turn_exit_thresh",  0.10)
        self.declare_parameter("approach_enter",    0.08)
        self.declare_parameter("approach_exit",     0.04)

        # ─── FİZİKSEL ─────────────────────────────────────────────────
        self.declare_parameter("wheel_base",      1.2)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("min_dt",          0.01)

        self.get_logger().info("Lane Controller v3.2 (Agresif Dönüş) başlatıldı - Robotaksi 2026")

        # ─── SUBSCRIBERS / PUBLISHERS ─────────────────────────────────
        self.sub_off  = self.create_subscription(Float32, "/perception/lane_offset",     self._cb_offset,     10)
        self.sub_ang  = self.create_subscription(Float32, "/perception/target_angle",    self._cb_angle,      10)
        self.sub_conf = self.create_subscription(Float32, "/perception/lane_confidence", self._cb_confidence, 10)
        self.sub_info = self.create_subscription(String,  "/perception/lane_info",       self._cb_info,       10)
        self.sub_obs  = self.create_subscription(String,  "/obstacle/state",             self._cb_obstacle,   10)
        # bt_decision_node bu çıktıyı alıp /cmd_vel'e yönlendiriyor
        self.pub_cmd  = self.create_publisher(Twist, "/cmd_vel_lane", 10)

        # ─── ALGILAMA VERİLERİ ────────────────────────────────────────
        self.offset     = 0.0
        self.angle_deg  = 0.0
        self.confidence = 0.0
        self.has_left   = False
        self.has_right  = False

        # ─── ENGEL DURUMU ─────────────────────────────────────────────
        self.obstacle_zone     = 'CLEAR'
        self.obstacle_dist     = 99.0
        self.obstacle_angle_deg = 0.0
        self.last_obstacle_time = self.get_clock().now()

        # ─── STATE ────────────────────────────────────────────────────
        self.turn_state    = "STRAIGHT"
        self.last_seen     = "NONE"
        self.integral      = 0.0
        self.prev_error    = 0.0
        self.prev_offset   = 0.0
        self.filtered_steer    = 0.0
        self.prev_steer        = 0.0
        self.filtered_speed    = self._p("target_speed")
        self.prev_angle_deg    = 0.0
        self.filtered_ang_rate = 0.0

        # Arama (Search) değişkenleri
        self.search_dir        = 1.0
        self.search_start_time = self.get_clock().now()
        # Kavşak düz geçiş: her iki şerit kaybolunca önce düz git
        self.both_lost_since: float | None = None
        self.straight_lock     = False

        self.last_time         = self.get_clock().now()
        self.last_info_time    = self.get_clock().now()
        self.dt                = 0.05
        self.loop_n            = 0

        self.create_timer(1.0 / self._p("control_rate_hz"), self._control_loop)

    # ─── YARDIMCI ─────────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    def _clip(self, val, lo, hi):
        return max(lo, min(hi, val))

    # ─── CALLBACKS ────────────────────────────────────────────────────

    def _cb_offset(self, msg):
        self.offset = float(msg.data)

    def _cb_angle(self, msg):
        self.angle_deg = float(msg.data)

    def _cb_confidence(self, msg):
        self.confidence = float(msg.data)

    def _cb_obstacle(self, msg):
        """/obstacle/state JSON → {'zone': 'STOP'|'SLOW'|'CLEAR', 'distance': float, ...}"""
        self.last_obstacle_time = self.get_clock().now()
        try:
            d = json.loads(msg.data)
            self.obstacle_zone      = d.get('zone', 'CLEAR')
            self.obstacle_dist      = float(d.get('distance', 99.0))
            self.obstacle_angle_deg = float(d.get('angle_deg', 0.0))
        except Exception as e:
            self.get_logger().warn(f'obstacle/state parse hatası: {e}')

    def _cb_info(self, msg):
        self.last_info_time = self.get_clock().now()
        try:
            d = json.loads(msg.data)
            self.has_left   = bool(d.get("left",   False))
            self.has_right  = bool(d.get("right",  False))
            self.confidence = float(d.get("conf", self.confidence))

            if "offset" in d:
                self.offset = float(d["offset"])
            if "angle" in d:
                self.angle_deg = float(d["angle"])

            if self.has_left and self.has_right:
                self.last_seen       = "BOTH"
            elif self.has_left:
                self.last_seen       = "LEFT"
            elif self.has_right:
                self.last_seen       = "RIGHT"

        except Exception as e:
            self.get_logger().warn(f"lane_info parse hatası: {e}")

    # ─── STATE MACHINE ────────────────────────────────────────────────

    def _update_dt(self):
        now = self.get_clock().now()
        raw_dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.dt = self._clip(raw_dt, self._p("min_dt"), 0.5)
        self.last_time = now

    def _update_turn_state(self, intensity):
        s = self.turn_state
        if s == "STRAIGHT":
            if intensity > self._p("approach_enter"):
                self.turn_state = "APPROACHING"
        elif s == "APPROACHING":
            if intensity > self._p("turn_enter_thresh"):
                self.turn_state = "TURNING"
            elif intensity < self._p("approach_exit"):
                self.turn_state = "STRAIGHT"
        elif s == "TURNING":
            if intensity < self._p("turn_exit_thresh"):
                self.turn_state = "APPROACHING"
        elif s == "RECOVERY":
            if self.has_left and self.has_right:
                self.turn_state = "STRAIGHT"
                self.get_logger().info("Recovery tamamlandı → STRAIGHT")

    # ─── HATA + PID ───────────────────────────────────────────────────

    def _compute_error(self):
        angle_rad   = math.radians(self.angle_deg)
        speed       = self.filtered_speed

        # Offset prediction
        off_rate         = (self.offset - self.prev_offset) / self.dt
        self.prev_offset = self.offset
        pred_off         = self.offset + off_rate * self._p("k_offset_pred")

        # Dynamic lookahead
        la   = self._p("lookahead_base") + speed * self._p("lookahead_speed")
        la_t = la * math.sin(angle_rad)

        # Açı kompansasyonu
        ang_t = self._p("k_angle") * math.sin(angle_rad)

        error = pred_off + ang_t + la_t

        # Deadband
        if abs(error) < self._p("error_deadband"):
            error = 0.0
        return error

    def _update_pid(self, error):
        kp = self._p("kp")
        ki = self._p("ki")
        kd = self._p("kd")

        # Adaptive kazanç — yol bitiminde salınımı önlemek için düşürüldü
        if self.turn_state == "APPROACHING":
            kp *= 1.10
        elif self.turn_state == "TURNING":
            kp *= 1.25
            kd *= 1.20

        # Confidence ağırlığı
        cw  = max(0.25, self.confidence)
        kp *= cw
        kd *= cw

        # Integral
        if self.has_left and self.has_right:
            self.integral += error * self.dt
            self.integral  = self._clip(
                self.integral,
                -self._p("integral_max"),
                 self._p("integral_max")
            )
        else:
            self.integral *= self._p("integral_decay")

        p = kp * error
        i = ki * self.integral
        d = kd * (error - self.prev_error) / max(self.dt, 0.001)
        self.prev_error = error
        return p + i + d

    # ─── DİREKSİYON ───────────────────────────────────────────────────

    def _compute_steering(self, pid_out):
        angle_rad = math.radians(self.angle_deg)

        # Feedforward
        curv = math.sin(angle_rad) / max(self._p("wheel_base"), 0.1)
        ff   = self._p("kff") * curv

        raw = -(pid_out + ff)

        # Dynamic limit
        max_s = (self._p("max_steer_turn")
                 if self.turn_state == "TURNING"
                 else self._p("max_steer_straight"))
        raw = self._clip(raw, -max_s, max_s)

        # Rate limiter
        rl    = self._p("steer_rate_limit")
        delta = self._clip(raw - self.prev_steer, -rl, rl)
        raw   = self.prev_steer + delta
        self.prev_steer = raw

        # Low-pass
        a = self._p("steer_filter_alpha")
        self.filtered_steer = (1 - a) * self.filtered_steer + a * raw
        return self.filtered_steer

    # ─── HIZ ──────────────────────────────────────────────────────────

    def _compute_speed(self, intensity):
        tgt = self._p("target_speed")
        mn  = self._p("min_speed")

        # Risk skoru
        r_off  = 0.30 * min(1.0, abs(self.offset) / 1.0)
        r_ang  = 0.35 * min(1.0, abs(self.angle_deg) / 30.0)
        r_conf = 0.20 * (1.0 - self.confidence)
        r_turn = 0.15 * min(1.0, intensity / 0.8)
        risk   = self._clip(r_off + r_ang + r_conf + r_turn, 0.0, 0.75)

        if abs(self.angle_deg) > 15.0:
            target = tgt * 0.80
        else:
            target = tgt * (1.0 - risk)

        if self.turn_state == "RECOVERY" or not (self.has_left and self.has_right):
            target = min(target, self._p("recovery_speed"))

        if self.confidence < self._p("confidence_threshold"):
            target = min(target, tgt * 0.45)

        target = max(target, mn)

        a = self._p("speed_filter_alpha")
        self.filtered_speed = (1 - a) * self.filtered_speed + a * target
        return self.filtered_speed

    def _compute_yaw_rate(self, steer, speed):
        if abs(speed) < 0.01:
            return 0.0
        return self._clip(
            speed * math.tan(steer) / self._p("wheel_base"),
            -1.2, 1.2
        )

    # ─── ŞERİT KAYIP MANTIĞI ──────────────────────────────────────────

    def _apply_lane_logic(self, yaw_rate, speed):
        now      = self.get_clock().now()
        info_age = (now.nanoseconds - self.last_info_time.nanoseconds) / 1e9

        # Timeout
        if info_age > self._p("info_timeout"):
            self.turn_state = "RECOVERY"
            self.get_logger().warn(f"Timeout {info_age:.1f}s → yavaşlıyor")
            return self._clip(yaw_rate, -0.35, 0.35), self._p("min_speed")

        slr = self._p("single_lane_yaw")
        sg  = self._p("single_lane_gain")
        rec = self._p("recovery_speed")

        # ── İKİSİ DE YOK veya STRAIGHT LOCK aktif ──────────────────────
        # Eğer her iki şerit de yoksa, straight lock başlasın
        if not self.has_left and not self.has_right:
            if self.both_lost_since is None:
                self.both_lost_since   = now.nanoseconds / 1e9
                self.search_start_time = now
                self.search_dir        = 1.0
                self.straight_lock     = True
                self.get_logger().info("Şerit kayboldu — kavşak düz geçiş (straight lock) başlıyor")

        # Eğer şu an straight lock aktifse:
        if self.straight_lock:
            elapsed_lost = (now.nanoseconds / 1e9) - self.both_lost_since
            
            # Eğer her iki şerit birden geldiyse ("tam diğer şeriti bulunca") -> erken kilit aç
            if self.has_left and self.has_right:
                self.straight_lock = False
                self.both_lost_since = None
                self.get_logger().info("Diğer şerit tam olarak bulundu — kilit açıldı, şeride dönülüyor")
                return yaw_rate, speed
            
            # Kilit süresi boyunca düz git
            if elapsed_lost < self._p("straight_lock_duration"):
                gentle_yaw = self._clip(yaw_rate, -0.15, 0.15)
                return gentle_yaw, rec
            else:
                # Kilit süresi doldu, ancak hala her iki şerit birden bulunamadı.
                # Tek şerit varsa kilidi açıp dönebiliriz.
                if self.has_left or self.has_right:
                    self.straight_lock = False
                    self.both_lost_since = None
                    self.get_logger().info("Kilit süresi doldu ve tek şerit bulundu — şeride dönülüyor")
                else:
                    # Hala hiçbir şerit yoksa SPIN & SWEEP arama moduna geç
                    straight_dur = self._p("intersection_straight_s")
                    if elapsed_lost < straight_dur:
                        # SPIN moduna geçmeden önce düz gitmeye devam et
                        gentle_yaw = self._clip(yaw_rate, -0.15, 0.15)
                        return gentle_yaw, rec
                    
                    # SPIN & SWEEP arama
                    search_rate = self._p("search_yaw_rate")
                    switch_time = self._p("search_switch_time")
                    elapsed_search = (now.nanoseconds - self.search_start_time.nanoseconds) / 1e9
                    if elapsed_search > switch_time:
                        self.search_dir       *= -1
                        self.search_start_time = now
                        self.get_logger().warn(f"Tarama yönü: {'SOL' if self.search_dir > 0 else 'SAĞ'}")
                    final_yaw = self._clip(search_rate * self.search_dir, -0.75, 0.75)
                    return final_yaw, self._p("min_speed")

        # Normal sürüş: şeritler var
        self.both_lost_since = None  # Güvenlik sıfırlaması

        # L+R → PID
        if self.has_left and self.has_right:
            return yaw_rate, speed

        # Sadece SOL → sağa yönelim (PID + sabit bileşen)
        if self.has_left and not self.has_right:
            self.turn_state = "RECOVERY"
            correction = -(sg * self.offset + slr)
            correction = self._clip(correction, -0.6, 0.6)
            if self.loop_n % 30 == 0:
                self.get_logger().info(f"[Recovery] SOL var → sağa (yaw={correction:.2f})")
            return correction, min(speed, rec)

        # Sadece SAĞ → sola yönelim
        if self.has_right and not self.has_left:
            self.turn_state = "RECOVERY"
            correction = -(sg * self.offset - slr)
            correction = self._clip(correction, -0.6, 0.6)
            if self.loop_n % 30 == 0:
                self.get_logger().info(f"[Recovery] SAĞ var → sola (yaw={correction:.2f})")
            return correction, min(speed, rec)

    # ─── ANA DÖNGÜ ────────────────────────────────────────────────────

    def _control_loop(self):
        self._update_dt()

        # Açı değişim hızı
        raw_rate = self._clip(
            (self.angle_deg - self.prev_angle_deg) / self.dt,
            -60.0, 60.0
        )
        self.filtered_ang_rate = (0.7 * self.filtered_ang_rate + 0.3 * raw_rate)
        self.prev_angle_deg = self.angle_deg

        # Turn intensity
        intensity = (abs(math.radians(self.angle_deg)) +
                     0.4 * abs(math.radians(self.filtered_ang_rate)))

        self._update_turn_state(intensity)

        error    = self._compute_error()
        pid_out  = self._update_pid(error)
        steering = self._compute_steering(pid_out)
        speed    = self._compute_speed(intensity)
        yaw_rate = self._compute_yaw_rate(steering, speed)

        final_yaw, final_speed = self._apply_lane_logic(yaw_rate, speed)

        # ─── ENGEL OVERRIDE ───────────────────────────────────────────
        now = self.get_clock().now()
        obs_age = (now.nanoseconds - self.last_obstacle_time.nanoseconds) / 1e9
        if obs_age > _OBSTACLE_TIMEOUT_S:
            effective_zone = 'CLEAR'
        else:
            effective_zone = self.obstacle_zone

        if effective_zone == 'STOP':
            final_speed = 0.0
            final_yaw   = 0.0
            if self.loop_n % 20 == 0:
                self.get_logger().warn(
                    f'[ENGEL] DUR! dist={self.obstacle_dist:.2f}m açı={self.obstacle_angle_deg:.1f}°'
                )
        elif effective_zone == 'SLOW':
            slow_factor  = 0.40
            final_speed  = min(final_speed, self._p('target_speed') * slow_factor)
            if self.loop_n % 40 == 0:
                self.get_logger().info(f'[ENGEL] YAVAŞ dist={self.obstacle_dist:.2f}m')

        twist = Twist()
        twist.linear.x  = float(final_speed)
        twist.angular.z = float(final_yaw)
        self.pub_cmd.publish(twist)

        self.loop_n += 1
        if self.loop_n % 30 == 0:
            self.get_logger().info(
                f"[Ctrl] {self.turn_state}|{effective_zone} | "
                f"L={self.has_left} R={self.has_right} | "
                f"off={self.offset:+.2f} ang={self.angle_deg:+.1f}° "
                f"conf={self.confidence:.2f} | "
                f"steer={steering:+.2f} yaw={final_yaw:+.2f} spd={final_speed:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        try:
            node.pub_cmd.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()