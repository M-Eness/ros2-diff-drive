#!/usr/bin/env python3
"""
Blinker / Işık Kontrol Node'u v2.0 — Robotaksi TEKNOFEST 2026
==============================================================
Araç sinyal, fren ve ikaz lambalarını yönetir.

Mantık:
  • Ön farlar    : daima açık
  • Sol sinyal   : yaw_rate > +eşik → sol blink
  • Sağ sinyal   : yaw_rate < −eşik → sağ blink
  • Fren lambası : hız < brake_thresh → kırmızı
  • 4'lü ikaz    : lane RECOVERY  VEYA  obstacle STOP/SLOW
  • Engel ikazı  : obstacle STOP → hızlı blink (2× normal hz)

Yayınlar:
  /blinker/state  (String JSON) — durum logu, her tick
  /light_config   (ros_gz_interfaces/Light) — Gazebo bridge (opsiyonel)
"""

import math
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, ColorRGBA
from tf2_msgs.msg import TFMessage

try:
    from ros_gz_interfaces.msg import Light as GzLight
    _HAS_GZ_LIGHT = True
except ImportError:
    _HAS_GZ_LIGHT = False

# Robot çerçevesindeki ışık ofsetleri [x, y, z]
_OFFSETS = {
    "headlight_left":  ( 1.0,  0.50, 0.45),
    "headlight_right": ( 1.0, -0.50, 0.45),
    "indicator_left":  ( 0.8,  0.62, 0.40),
    "indicator_right": ( 0.8, -0.62, 0.40),
    "brake_left":      (-1.0,  0.50, 0.40),
    "brake_right":     (-1.0, -0.50, 0.40),
}

AMBER = (1.0, 0.55, 0.0)
RED   = (1.0, 0.0,  0.0)
WHITE = (1.0, 1.0,  1.0)


class BlinkerNode(Node):

    def __init__(self):
        super().__init__("blinker_node")

        # ─── PARAMETRELER ─────────────────────────────────────────────
        self.declare_parameter("blink_hz",            2.0)   # normal blink hızı
        self.declare_parameter("obstacle_blink_hz",   4.0)   # engel ikazı hızı
        self.declare_parameter("turn_yaw_thresh",     0.15)
        self.declare_parameter("stop_speed_thresh",   0.05)
        self.declare_parameter("brake_speed_thresh",  0.25)
        self.declare_parameter("enable_gz_bridge",    True)  # Light mesajı yayınla

        # ─── SUBSCRIBER'LAR ───────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_vel   = self.create_subscription(Twist,     "/cmd_vel",              self._cb_vel,      10)
        self.sub_info  = self.create_subscription(String,    "/perception/lane_info", self._cb_lane,     10)
        self.sub_obs   = self.create_subscription(String,    "/obstacle/state",       self._cb_obstacle, 10)
        self.sub_poses = self.create_subscription(TFMessage, "/gz_model_poses",       self._cb_poses,    sensor_qos)

        # ─── PUBLISHER'LAR ────────────────────────────────────────────
        self.pub_state = self.create_publisher(String, "/blinker/state", 10)

        self.pub_gz = None
        if _HAS_GZ_LIGHT and self._p("enable_gz_bridge"):
            self.pub_gz = self.create_publisher(GzLight, "/light_config", 10)

        # ─── ARAÇ DURUMU ──────────────────────────────────────────────
        self.linear_x     = 0.0
        self.angular_z    = 0.0
        self.in_recovery  = False

        # ─── ENGEL DURUMU ─────────────────────────────────────────────
        self.obstacle_zone = 'CLEAR'

        # ─── ROBOT POSE (dünya koordinatları) ─────────────────────────
        self.rx   = 2.0
        self.ry   = 3.0
        self.rz   = 0.02
        self.ryaw = 0.0

        # ─── BLINK SAYACI ─────────────────────────────────────────────
        self.blink_phase = False
        self.tick_count  = 0

        # Normal timer — obstacle hızlı blink için tick_count ile böl
        base_period = 1.0 / (2.0 * self._p("obstacle_blink_hz"))
        self.create_timer(base_period, self._tick)

        self.get_logger().info(
            f"BlinkerNode v2.0 başlatıldı | "
            f"gz_bridge={'açık' if self.pub_gz else 'kapalı'}"
        )

    def _p(self, name):
        return self.get_parameter(name).value

    # ─── CALLBACKS ────────────────────────────────────────────────────

    def _cb_vel(self, msg):
        self.linear_x  = float(msg.linear.x)
        self.angular_z = float(msg.angular.z)

    def _cb_lane(self, msg):
        try:
            d = json.loads(msg.data)
            left  = bool(d.get("left",  False))
            right = bool(d.get("right", False))
            self.in_recovery = not (left and right)
        except Exception:
            pass

    def _cb_obstacle(self, msg):
        try:
            d = json.loads(msg.data)
            self.obstacle_zone = d.get('zone', 'CLEAR')
        except Exception:
            pass

    def _cb_poses(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "my_robot/base_footprint":
                self.rx = t.transform.translation.x
                self.ry = t.transform.translation.y
                self.rz = t.transform.translation.z
                q       = t.transform.rotation
                self.ryaw = 2.0 * math.atan2(q.z, q.w)
                break

    # ─── ANA DÖNGÜ ────────────────────────────────────────────────────

    def _tick(self):
        self.tick_count = (self.tick_count + 1) % 10000

        speed = self.linear_x
        yaw   = self.angular_z

        # Engel durumu
        obstacle_stop = (self.obstacle_zone == 'STOP')
        obstacle_slow = (self.obstacle_zone == 'SLOW')

        # 4'lü ikaz koşulları: dur, recovery, engel
        stopped   = abs(speed) < self._p("stop_speed_thresh")
        hazard    = stopped or self.in_recovery or obstacle_stop or obstacle_slow

        # Sinyal lambası (hazard yokken)
        turning_left  = yaw >  self._p("turn_yaw_thresh") and not hazard
        turning_right = yaw < -self._p("turn_yaw_thresh") and not hazard

        # Fren lambası
        braking = (speed < self._p("brake_speed_thresh")) or obstacle_stop

        # Blink hızı: engel STOP → hızlı blink (obstacle_blink_hz)
        #             diğer → normal blink (blink_hz / obstacle_blink_hz oranı)
        normal_ratio = max(1, round(
            self._p("obstacle_blink_hz") / max(self._p("blink_hz"), 0.1)
        ))
        if obstacle_stop:
            blink_on = (self.tick_count % 2 == 0)   # her tick değişir (en hızlı)
        else:
            blink_on = (self.tick_count % (normal_ratio * 2) < normal_ratio)

        left_blink  = blink_on and (turning_left  or hazard)
        right_blink = blink_on and (turning_right or hazard)

        # Işıkları güncelle
        self._light("headlight_left",  True,        *WHITE, rng=14.0)
        self._light("headlight_right", True,        *WHITE, rng=14.0)
        self._light("indicator_left",  left_blink,  *AMBER)
        self._light("indicator_right", right_blink, *AMBER)
        self._light("brake_left",      braking,     *RED)
        self._light("brake_right",     braking,     *RED)

        # Durum JSON'ı yayınla
        state = {
            "zone":        self.obstacle_zone,
            "hazard":      hazard,
            "turn_left":   turning_left,
            "turn_right":  turning_right,
            "braking":     braking,
            "recovery":    self.in_recovery,
            "blink_on":    blink_on,
        }
        self.pub_state.publish(String(data=json.dumps(state)))

    # ─── IŞIK YAYINI ──────────────────────────────────────────────────

    def _light(self, name: str, on: bool,
               r: float, g: float, b: float, rng: float = 6.0):
        """Gazebo Light mesajı oluştur ve yayınla (bridge aktifse)."""
        if self.pub_gz is None:
            return

        intensity = 1.0 if on else 0.0
        msg             = GzLight()
        msg.name        = name
        msg.type        = GzLight.POINT
        msg.diffuse     = ColorRGBA(r=r * intensity, g=g * intensity,
                                    b=b * intensity, a=1.0)
        msg.specular    = ColorRGBA(r=r * 0.3 * intensity,
                                    g=g * 0.3 * intensity,
                                    b=b * 0.3 * intensity, a=1.0)
        msg.intensity             = intensity
        msg.range                 = rng
        msg.attenuation_constant  = 0.10
        msg.attenuation_linear    = 0.04
        msg.attenuation_quadratic = 0.003

        # Robot pozisyonuna göre ışık konumu
        ox, oy, oz = _OFFSETS[name]
        c, s = math.cos(self.ryaw), math.sin(self.ryaw)
        msg.pose.position.x    = self.rx + ox * c - oy * s
        msg.pose.position.y    = self.ry + ox * s + oy * c
        msg.pose.position.z    = self.rz + oz
        msg.pose.orientation.w = 1.0

        self.pub_gz.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BlinkerNode()
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
