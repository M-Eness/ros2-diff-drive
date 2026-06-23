#!/usr/bin/env python3
"""
BT Decision Node v3.1 - Robotaksi TEKNOFEST 2026 (Soft Stop & Parametrized Turn)
=================================================
Düzeltmeler:
  - Durak dönüş açısı parametre haline getirildi.
  - Dönüş sonrası ani fren yapmak yerine yumuşak duruş (soft stop) eklendi.
"""

import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

# ── Levha grupları ────────────────────────────────────────────────────────────
STOP_SIGNS      = {"dur"}
NO_ENTRY_SIGNS  = {"girilmez"}
SLOW_SIGNS      = {"yaya_gecidi", "ada_etrafinda_don"}
TURN_RIGHT_SIGNS = {"saga_mecburi_yon", "ileri_ve_saga_mecburi_yon",
                    "ileriden_saga_mecburi_yon"}
TURN_LEFT_SIGNS  = {"sola_mecburi_yon", "ileri_ve_sola_mecburi_yon",
                    "ileriden_sola_mecburi_yon"}
PASSENGER_SIGNS = {"durak"}
PARK_SIGNS      = {"park_yeri"}
TUNNEL_SIGNS    = {"tunel"}


class BTDecisionNode(Node):

    def __init__(self):
        super().__init__('bt_decision_node')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('tur_modu', 2)
        self.declare_parameter('normal_speed', 0.3)
        self.declare_parameter('slow_speed', 0.15)
        self.declare_parameter('turn_speed', 0.50)
        self.declare_parameter('turn_duration', 2.5)
        self.declare_parameter('stop_duration', 3.0)
        self.declare_parameter('durak_turn_duration', 3.2)
        self.declare_parameter('durak_turn_angular', -0.5)   # 🔥 Dönüş yönünü buradan değiştirebilirsin (Sol: +0.5, Sağ: -0.5)
        self.declare_parameter('cooldown_duration', 10.0)
        self.declare_parameter('passenger_wait_min', 15.0)
        self.declare_parameter('passenger_wait_max', 20.0)
        self.declare_parameter('lane_cmd_timeout', 0.5)
        self.declare_parameter('obstacle_timeout', 2.0)
        self.declare_parameter('sign_min_bbox_area', 3000)
        self.declare_parameter('passenger_min_bbox_area', 7000)
        self.declare_parameter('sign_min_frames', 5)
        self.declare_parameter('park_align_duration', 3.0)
        self.declare_parameter('park_reverse_duration', 4.5)
        self.declare_parameter('park_straight_duration', 2.5)
        self.declare_parameter('park_reverse_speed', -0.12)
        self.declare_parameter('park_turn_speed', 0.50)
        self.declare_parameter('park_align_speed', 0.20)
        self.declare_parameter('gps_fallback_speed', 0.3)
        self.declare_parameter('slalom_timeout', 0.5)
        self.declare_parameter('durak_only_mode', False)

        self.tur_modu = self.get_parameter('tur_modu').value

        # ── Algı değişkenleri ─────────────────────────────────────────────────
        self.traffic_light_red   = False
        self.traffic_light_green = True
        self.green_light_time    = None

        self.current_sign        = None
        self.lane_offset         = 0.0
        self.lane_left           = False
        self.lane_right          = False

        self.lane_cmd: Twist | None = None
        self.last_lane_cmd_time     = None
        self.gps_cmd: Twist | None  = None
        self.last_gps_cmd_time      = None
        self.slalom_cmd: Twist | None = None
        self.last_slalom_cmd_time   = None

        self.current_lat = None
        self.current_lon = None
        self.current_wp_type = None
        self.current_wp_dur = 15.0
        self.current_wp_lat = None
        self.current_wp_lon = None
        self.ref_lat = 41.015137
        self.ref_lon = 28.979530
        self.earth_radius = 6378137.0

        self.obstacle_zone     = 'CLEAR'
        self.obstacle_dist     = 99.0
        self.obstacle_angle    = 0.0
        self.last_obstacle_time = None

        self.sign_candidate        = None
        self.sign_candidate_frames = 0

        self.state = "NORMAL"
        self.last_seen = "NONE"

        # Zamanlayıcılar
        self.stop_until           = 0.0
        self.turn_until           = 0.0
        self.passenger_until      = 0.0
        self.durak_turn_until     = 0.0
        self.cooldown_until       = 0.0

        self.park_phase       = "IDLE"
        self.park_phase_until = 0.0
        self.in_park_zone     = False
        self.park_dir         = 1
        self.park_done_published = False

        self.loop_n     = 0
        self.speed_limit = None

        # ── Subscribers (sadece DURAK navigasyonu için gerekenler) ───────────
        self.create_subscription(Twist,     '/cmd_vel_gps',    self._cb_gps_cmd,   10)
        self.create_subscription(NavSatFix, '/fix',            self._cb_gps,       10)
        self.create_subscription(String,    '/map/waypoints',  self._cb_waypoints, 10)

        # -- Devre dışı (durak_only_mode) --
        if not self.get_parameter('durak_only_mode').value:
            self.create_subscription(Bool,    '/perception/flags/traffic_light_red',   self._cb_red,        10)
            self.create_subscription(Bool,    '/perception/flags/traffic_light_green',  self._cb_green,      10)
            self.create_subscription(String,  '/traffic_sign_detections',               self._cb_sign,       10)
            self.create_subscription(Float32, '/perception/lane_offset',                self._cb_offset,     10)
            self.create_subscription(String,  '/perception/lane_info',                  self._cb_lane_info,  10)
            self.create_subscription(Twist,   '/cmd_vel_lane',                          self._cb_lane_cmd,   10)
            self.create_subscription(Twist,   '/cmd_vel_slalom',                        self._cb_slalom_cmd, 10)
            self.create_subscription(String,  '/obstacle/state',                        self._cb_obstacle,   10)
            self.create_subscription(String,  '/mission/command',                       self._cb_mission,    10)
            self.create_subscription(Float32, '/speed_limit',                           self._cb_speed_limit,10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.status_pub = self.create_publisher(String, '/bt/status', 10)
        self.pub_wp_reached = self.create_publisher(String, '/map/wp_reached', 10)
        self.pub_durak_dist = self.create_publisher(Float32, '/durak/dist',      10)
        self.pub_durak_rem  = self.create_publisher(Int32,   '/durak/remaining', 10)

        # BT döngüsü 10 Hz
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'BTDecisionNode v3.1 başlatıldı | tur_modu={self.tur_modu}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_red(self, msg):
        self.traffic_light_red = msg.data
        if msg.data:
            self.green_light_time = None

    def _cb_green(self, msg):
        prev = self.traffic_light_green
        self.traffic_light_green = msg.data
        if msg.data and not prev:
            self.green_light_time = time.time()

    def _cb_sign(self, msg):
        if time.time() < self.cooldown_until:
            self.sign_candidate = None
            self.sign_candidate_frames = 0
            return
        try:
            dets = json.loads(msg.data)
            if not dets:
                self.sign_candidate = None
                self.sign_candidate_frames = 0
                return
            best = max(dets, key=lambda d: d['confidence'])
            if best['confidence'] < 0.60:
                self.sign_candidate = None
                self.sign_candidate_frames = 0
                return
            b = best.get('bbox', [0, 0, 0, 0])
            area = (b[2] - b[0]) * (b[3] - b[1])
            name = best['class_name']
            min_area = self.get_parameter('passenger_min_bbox_area').value if name in PASSENGER_SIGNS else self.get_parameter('sign_min_bbox_area').value
            if area < min_area:
                self.sign_candidate = None
                self.sign_candidate_frames = 0
                return
            if name == self.sign_candidate:
                self.sign_candidate_frames += 1
            else:
                self.sign_candidate = name
                self.sign_candidate_frames = 1
            if self.sign_candidate_frames >= self.get_parameter('sign_min_frames').value:
                self.current_sign = name
                self.get_logger().info(f'Levha onaylandı: {name} ({self.sign_candidate_frames} frame, alan={area}px²)')
        except Exception:
            pass

    def _cb_offset(self, msg):
        self.lane_offset = float(msg.data)

    def _cb_lane_info(self, msg):
        try:
            d = json.loads(msg.data)
            self.lane_left  = bool(d.get('left',  False))
            self.lane_right = bool(d.get('right', False))
            if self.lane_left and self.lane_right:
                self.last_seen = "BOTH"
            elif self.lane_left:
                self.last_seen = "LEFT"
            elif self.lane_right:
                self.last_seen = "RIGHT"
        except Exception:
            pass

    def _cb_lane_cmd(self, msg):
        self.lane_cmd = msg
        self.last_lane_cmd_time = time.time()

    def _cb_gps_cmd(self, msg):
        self.gps_cmd = msg
        self.last_gps_cmd_time = time.time()

    def _cb_gps(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def _cb_waypoints(self, msg):
        try:
            if msg.data == 'FINISH':
                self.current_wp_type = 'FINISH'
                return
            wp = json.loads(msg.data)
            self.current_wp_type = wp.get('type')
            self.current_wp_dur = float(wp.get('dur', 15.0))
            self.current_wp_lat = float(wp.get('lat'))
            self.current_wp_lon = float(wp.get('lon'))
        except Exception:
            pass

    def _cb_slalom_cmd(self, msg):
        self.slalom_cmd = msg
        self.last_slalom_cmd_time = time.time()

    def _cb_obstacle(self, msg):
        try:
            d = json.loads(msg.data)
            self.obstacle_zone = d.get('zone', 'CLEAR')
            self.obstacle_dist = d.get('distance', 99.0)
            self.obstacle_angle = d.get('angle_deg', 0.0)
            self.last_obstacle_time = time.time()
        except Exception:
            pass

    def _cb_mission(self, msg):
        cmd = msg.data
        if cmd == "PARK_ZONE":
            self.in_park_zone = True
            self.get_logger().info('Park bölgesine girildi.')
        elif cmd == "PARK" and self.state != "PARK":
            self.get_logger().info('Mission → PARK modu başlıyor.')
            self.in_park_zone = False
            self._start_park()
        elif cmd == "SLALOM_START":
            self.state = "SLALOM"
            self.get_logger().info('Mission → SLALOM modu')
        elif cmd == "SLALOM_STOP":
            self.state = "NORMAL"
            self.get_logger().info('Mission → SLALOM bitti → NORMAL')
        elif cmd == "EMERGENCY_STOP":
            self.state = "EMERGENCY"
            self.get_logger().error('UMS-1 KILL → EMERGENCY DUR!')
        elif cmd == "EMERGENCY_CLEAR":
            if self.state == "EMERGENCY":
                self.state = "NORMAL"
                self.get_logger().info('UMS-1 Kill temizlendi → NORMAL')
        elif cmd == "RESET":
            self.state = "NORMAL"
            self.get_logger().info('Mission → RESET')

    def _cb_speed_limit(self, msg):
        val = float(msg.data)
        self.speed_limit = max(0.05, min(1.5, val))
        self.get_logger().info(f'Hız limiti güncellendi: {self.speed_limit:.2f} m/s')

    # ── Yardımcı Fonksiyonlar ─────────────────────────────────────────────────

    def _stop(self):
        """Tam durdurma."""
        self.cmd_pub.publish(Twist())

    def _move(self, linear, angular=0.0):
        """Hareket emri gönderir, hız limitini uygular."""
        t = Twist()
        lin = float(linear)
        if self.speed_limit is not None and lin > 0:
            lin = min(lin, self.speed_limit)
        t.linear.x = lin
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def _status(self, s: str):
        """Durum yayını."""
        self.status_pub.publish(String(data=s))

    def _set_cooldown(self):
        """Levha işlendikten sonra belirli süre aynı levhayı yoksay."""
        self.cooldown_until = time.time() + self.get_parameter('cooldown_duration').value
        self.current_sign = None

    def _effective_obstacle_zone(self) -> str:
        """Obstacle mesajı zaman aşımına uğradıysa CLEAR döndür."""
        if self.last_obstacle_time is None:
            return 'CLEAR'
        if time.time() - self.last_obstacle_time > self.get_parameter('obstacle_timeout').value:
            return 'CLEAR'
        return self.obstacle_zone

    def _wgs84_to_enu(self, lat, lon):
        """GPS koordinatlarını yerel ENU (metre) sistemine çevirir."""
        dlat = math.radians(lat - self.ref_lat)
        dlon = math.radians(lon - self.ref_lon)
        y = dlat * self.earth_radius
        x = dlon * self.earth_radius * math.cos(math.radians(self.ref_lat))
        return x, y

    def _relay_lane(self):
        """
        DURAK hedefi varken GPS yönlendirmesi baskındır (%35–%65), şerit yardımcıdır.
        Hedef yokken saf şerit takibi. Şerit yoksa tam GPS kontrolü.
        """
        gps_speed = self.get_parameter('normal_speed').value
        gps_fresh = False
        if self.gps_cmd is not None and time.time() - self.last_gps_cmd_time < 0.5:
            gps_speed = self.gps_cmd.linear.x
            gps_fresh = True

        # Şerit yoksa tam GPS kontrolü
        if not (self.lane_left or self.lane_right) and gps_fresh:
            self.cmd_pub.publish(self.gps_cmd)
            self._status("NORMAL_GPS_CTRL")
            return

        # DURAK hedefi varken GPS yönlendirmesini hesapla
        gps_blend = 0.0
        if gps_fresh and self.current_wp_type == 'DURAK' and \
                self.current_lat is not None and self.current_wp_lat is not None:
            cx, cy = self._wgs84_to_enu(self.current_lat, self.current_lon)
            wx, wy = self._wgs84_to_enu(self.current_wp_lat, self.current_wp_lon)
            wp_dist = math.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)
            if wp_dist > 30.0:
                gps_blend = 0.55          # uzakta: %55 GPS, %45 şerit
            elif wp_dist > 10.0:
                gps_blend = 0.65          # orta: %65 GPS, %35 şerit
            elif wp_dist > 3.0:
                gps_blend = 0.75          # yakın: %75 GPS, %25 şerit
            else:
                gps_blend = 0.85          # çok yakın: %85 GPS, %15 şerit

        lane_timeout = self.get_parameter('lane_cmd_timeout').value
        if self.lane_cmd is not None and self.last_lane_cmd_time is not None and \
                time.time() - self.last_lane_cmd_time < lane_timeout:
            t = Twist()
            t.linear.x = float(gps_speed)
            if gps_blend > 0.0:
                t.angular.z = (1.0 - gps_blend) * self.lane_cmd.angular.z \
                              + gps_blend * self.gps_cmd.angular.z
                self._status("NORMAL_GPS_NAV")
            else:
                t.angular.z = self.lane_cmd.angular.z
                self._status("NORMAL_LANE_CTRL")
            self.cmd_pub.publish(t)
        else:
            # Fallback: GPS varsa GPS, yoksa offset kontrolü
            if gps_fresh and gps_blend > 0.0:
                self.cmd_pub.publish(self.gps_cmd)
                self._status("NORMAL_GPS_CTRL")
            else:
                t = Twist()
                t.linear.x = float(gps_speed)
                t.angular.z = -self.lane_offset * 0.8
                self.cmd_pub.publish(t)
                self._status("NORMAL_FALLBACK")

    def _start_park(self):
        """Park sekansını başlatır."""
        self.state = "PARK"
        self.park_phase = "IDLE"
        self.park_phase_until = 0.0
        self._status("PARK_BASLADI")

    def _run_park(self, now: float) -> bool:
        """
        Park alt-durumlarını yönetir. Tamamlanana kadar True döndürür.
        """
        d = self.park_dir

        if self.park_phase == "IDLE":
            self.park_phase = "HIZALAMA"
            self.park_phase_until = now + self.get_parameter('park_align_duration').value
            self.get_logger().info('Park ▶ hizalama')

        if self.park_phase == "HIZALAMA":
            if now < self.park_phase_until:
                self._move(self.get_parameter('park_align_speed').value)
                self._status("PARK_HIZALAMA")
                return True
            self.park_phase = "GERI_DONUS"
            self.park_phase_until = now + self.get_parameter('park_reverse_duration').value
            self.get_logger().info('Park ▶ geri dönüş')

        if self.park_phase == "GERI_DONUS":
            if now < self.park_phase_until:
                self._move(self.get_parameter('park_reverse_speed').value,
                           d * (-self.get_parameter('park_turn_speed').value))
                self._status("PARK_GERI")
                return True
            self.park_phase = "DUZELT"
            self.park_phase_until = now + self.get_parameter('park_straight_duration').value
            self.get_logger().info('Park ▶ düzeltme')

        if self.park_phase == "DUZELT":
            if now < self.park_phase_until:
                self._move(self.get_parameter('park_reverse_speed').value,
                           d * self.get_parameter('park_turn_speed').value)
                self._status("PARK_DUZELT")
                return True
            self.park_phase = "TAMAM"
            self.get_logger().info('Park ▶ TAMAMLANDI')

        self._stop()
        self._status("PARK_TAMAM")
        return True

    # ── Ana Tick (Aktif: GPS + DURAK yönetimi) ───────────────────────────────

    def _tick(self):
        now = time.time()

        # DURAK bekleme
        if self.state == "PASSENGER":
            if now < self.passenger_until:
                self.cmd_pub.publish(Twist())
                rem = self.passenger_until - now
                self._status(f"DURUYORUM ({rem:.0f}sn)")
                self.pub_durak_rem.publish(Int32(data=int(rem)))
                return
            self.pub_durak_rem.publish(Int32(data=0))
            self.state = "NORMAL"
            self.pub_wp_reached.publish(String(data='OK'))
            self.get_logger().info('Durak bekleme bitti → sonraki durak')
            return

        # Durağa GPS yakınlık kontrolü
        if (self.current_wp_type == "DURAK" and
                self.current_lat is not None and self.current_wp_lat is not None):
            cx, cy = self._wgs84_to_enu(self.current_lat, self.current_lon)
            wx, wy = self._wgs84_to_enu(self.current_wp_lat, self.current_wp_lon)
            dist = math.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)
            self.pub_durak_dist.publish(Float32(data=float(dist)))
            if dist < 8.0 and now > self.cooldown_until:
                wait = random.uniform(
                    self.get_parameter('passenger_wait_min').value,
                    self.get_parameter('passenger_wait_max').value)
                self.get_logger().warn(f'DURAK: {dist:.2f}m → {wait:.0f}s bekleme')
                self.passenger_until = now + wait
                self.state = "PASSENGER"
                self._set_cooldown()
                self.cmd_pub.publish(Twist())
                self._status("DURUYORUM")
                return

        # GPS komutunu ilet
        if (self.gps_cmd is not None and self.last_gps_cmd_time is not None and
                now - self.last_gps_cmd_time < 0.5 and self.gps_cmd.linear.x > 0.0):
            self.cmd_pub.publish(self.gps_cmd)
            self._status("GPS_DURAK_NAV")
            return

        self.cmd_pub.publish(Twist())
        self._status("GPS_BEKLE")

    # ── Tam Mantık (devre dışı — ileride aktifleştirilebilir) ─────────────────

    def _tick_disabled(self):
        now = time.time()
        self.loop_n += 1

        # 0. EMERGENCY – en yüksek öncelik
        if self.state == "EMERGENCY":
            self._stop()
            self._status("UMS_EMERGENCY_DUR")
            return

        # 1. TUR MODU (sadece engel tepkisi)
        if self.tur_modu == 1:
            obs = self._effective_obstacle_zone()
            if obs == 'STOP':
                self._stop()
                self._status("TUR1_ENGEL_DUR")
            else:
                self._relay_lane()
                if obs == 'SLOW':
                    self._status("TUR1_ENGEL_YAVAS")
            return

        # 1.5 DURAK_ONLY modu — sadece şerit takibi + DURAK GPS durağı
        if self.get_parameter('durak_only_mode').value:
            if self.state == "PASSENGER":
                if now < self.passenger_until:
                    self._stop()
                    rem = self.passenger_until - now
                    self._status(f"DURUYORUM ({rem:.0f}sn)")
                    self.pub_durak_rem.publish(Int32(data=int(rem)))
                    return
                self.get_logger().info('Durak bekleme süresi doldu. Devam.')
                self.pub_durak_rem.publish(Int32(data=0))
                self.state = "NORMAL"
                self.pub_wp_reached.publish(String(data='OK'))
                return
            if (self.state == "NORMAL" and self.current_wp_type == "DURAK" and
                    self.current_lat is not None and self.current_wp_lat is not None):
                cx, cy = self._wgs84_to_enu(self.current_lat, self.current_lon)
                wx, wy = self._wgs84_to_enu(self.current_wp_lat, self.current_wp_lon)
                dist = math.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)
                self.pub_durak_dist.publish(Float32(data=float(dist)))
                if dist < 8.0 and time.time() > self.cooldown_until:
                    wait = random.uniform(
                        self.get_parameter('passenger_wait_min').value,
                        self.get_parameter('passenger_wait_max').value)
                    self.get_logger().warn(f'DURAK GPS: {dist:.2f}m → {wait:.0f}s bekleme')
                    self.passenger_until = now + wait
                    self.state = "PASSENGER"
                    self._set_cooldown()
                    self._stop()
                    self._status("DURUYORUM")
                    return
            # Şerit takibi + GPS yönlendirme karışımı (robot yolda kalır)
            self._relay_lane()
            return

        # 2. KIRMIZI IŞIK
        if self.traffic_light_red:
            self._stop()
            self._status("KIRMIZI_ISIK")
            return

        # 3. GİRİLMEZ — geri çekil, sonra 180° U-dönüşü
        if self.state == "NO_ENTRY":
            if now < self.stop_until:
                # Faz 1: 1.5s geri
                self._move(-0.15)
                self._status("GIRILMEZ_GERI")
                return
            if now < self.turn_until:
                # Faz 2: ~180° sağa dön (π / 0.5 rad/s ≈ 6.3s)
                self._move(0.25, -0.5)
                self._status("GIRILMEZ_DONUS")
                return
            self.state = "NORMAL"

        # 4. DUR LEVHASI
        if self.state == "STOP":
            if now < self.stop_until:
                self._stop()
                self._status("DUR_LEVHASI_BEKLE")
                return
            self.state = "NORMAL"

        # 5. ENGEL STOP
        obs = self._effective_obstacle_zone()
        if obs == 'STOP':
            self._stop()
            if self.loop_n % 20 == 0:
                self.get_logger().warn(f'[ENGEL] DUR dist={self.obstacle_dist:.2f}m')
            self._status(f"ENGEL_DUR_{self.obstacle_dist:.1f}m")
            return

        # 6. SLALOM
        if self.state == "SLALOM":
            if (self.slalom_cmd is not None and self.last_slalom_cmd_time is not None
                    and time.time() - self.last_slalom_cmd_time < self.get_parameter('slalom_timeout').value):
                self.cmd_pub.publish(self.slalom_cmd)
                self._status("SLALOM_AKTIF")
            else:
                self._stop()
                self._status("SLALOM_BEKLE")
            return

        # 7. PARK
        if self.state == "PARK":
            self._run_park(now)
            return

        # 7.5. PARK_DONE — park tamamlandı, hareketsiz bekle
        if self.state == "PARK_DONE":
            self._stop()
            self._status("PARK_TAMAMLANDI")
            if not self.park_done_published:
                self.park_done_published = True
                self.pub_wp_reached.publish(String(data='OK'))
            return

        # 8. DURAK TURN (Parametrik Dönüş + Yumuşak Duruş)
        if self.state == "DURAK_TURN":
            if now < self.durak_turn_until:
                # 🔥 Dönüşü daha yumuşak yapmak için hızı yarıya indir (0.3 -> 0.15)
                self._move(self.get_parameter('normal_speed').value * 0.5,
                           self.get_parameter('durak_turn_angular').value)
                self._status("DURAGA_DONULUYOR")
                return
            
            # 🔥 YUMUŞAK FREN: Dönüş bitti, hızı düşür ve tam dur
            if self.durak_turn_until + 0.5 > now:
                self._move(self.get_parameter('normal_speed').value * 0.2, 0.0)
                self._status("YAVASLIYOR")
                return

            self._stop()
            self.state = "PASSENGER"
            return

        # 9. PASSENGER (yolcu bekleme)
        if self.state == "PASSENGER":
            if now < self.passenger_until:
                self._stop()
                rem = self.passenger_until - now
                self._status(f"DURUYORUM ({rem:.0f}sn)")
                self.pub_durak_rem.publish(Int32(data=int(rem)))
                return
            self.get_logger().info('Durak bekleme süresi doldu. Yolcu alındı, devam.')
            self.pub_durak_rem.publish(Int32(data=0))
            self.state = "NORMAL"
            self.pub_wp_reached.publish(String(data='OK'))
            return

        # 9.5. Durak mesafesi: dashboard yayını + GPS ±1 m tetikleyici
        if self.state == "NORMAL" and self.current_wp_type == "DURAK" and self.current_lat is not None and self.current_wp_lat is not None:
            curr_x, curr_y = self._wgs84_to_enu(self.current_lat, self.current_lon)
            wp_x, wp_y = self._wgs84_to_enu(self.current_wp_lat, self.current_wp_lon)
            dist = math.sqrt((wp_x - curr_x)**2 + (wp_y - curr_y)**2)
            self.pub_durak_dist.publish(Float32(data=float(dist)))
            if dist < 3.0:  # durak levha bölgesine 3m yaklaşıldı → dur
                pw_min = self.get_parameter('passenger_wait_min').value
                pw_max = self.get_parameter('passenger_wait_max').value
                wait = random.uniform(pw_min, pw_max)
                self.get_logger().warn(f'DURAK GPS tetiklendi: dist={dist:.2f}m → {wait:.0f}s bekleme')
                self.passenger_until = now + wait
                self.state = "PASSENGER"
                self._set_cooldown()
                self._stop()
                self._status("DURUYORUM")
                return

        # 9.6. Park bölgesi GPS tetikleyicisi
        if self.state == "NORMAL" and self.current_wp_type == "PARK" and self.current_lat is not None and self.current_wp_lat is not None:
            curr_x, curr_y = self._wgs84_to_enu(self.current_lat, self.current_lon)
            wp_x, wp_y = self._wgs84_to_enu(self.current_wp_lat, self.current_wp_lon)
            dist = math.sqrt((wp_x - curr_x)**2 + (wp_y - curr_y)**2)
            self.pub_durak_dist.publish(Float32(data=float(dist)))
            if dist < 3.0:
                self.get_logger().warn(f'PARK GPS tetiklendi: dist={dist:.2f}m → park başlıyor')
                self.park_done_published = False
                self.state = "PARK_DONE"
                self._stop()
                self._status("PARK_TAMAMLANDI")
                return

        # 10. ENGEL SLOW
        if obs == 'SLOW':
            if (self.lane_cmd is not None and self.last_lane_cmd_time is not None
                    and time.time() - self.last_lane_cmd_time < self.get_parameter('lane_cmd_timeout').value):
                t = Twist()
                t.linear.x = min(self.lane_cmd.linear.x, self.get_parameter('slow_speed').value)
                t.angular.z = self.lane_cmd.angular.z
                self.cmd_pub.publish(t)
            else:
                self._move(self.get_parameter('slow_speed').value, -self.lane_offset * 0.8)
            if self.loop_n % 20 == 0:
                self.get_logger().info(f'[ENGEL] YAVAS dist={self.obstacle_dist:.2f}m')
            self._status(f"ENGEL_YAVAS_{self.obstacle_dist:.1f}m")
            return

        # 11. DÖNÜŞ (mecburi yön levhası)
        if self.state == "TURN_RIGHT":
            if now < self.turn_until:
                self._move(self.get_parameter('normal_speed').value, -self.get_parameter('turn_speed').value)
                self._status("SAGA_DONUS")
                return
            self.state = "NORMAL"

        if self.state == "TURN_LEFT":
            if now < self.turn_until:
                self._move(self.get_parameter('normal_speed').value, self.get_parameter('turn_speed').value)
                self._status("SOLA_DONUS")
                return
            self.state = "NORMAL"

        # 12. TÜNEL
        if self.state == "TUNNEL":
            self._relay_lane()
            self._status("TUNEL_GECISI")
            return

        # 13. YAVAŞ LEVHASI (yaya geçidi vb.)
        sign = self.current_sign
        if sign in SLOW_SIGNS:
            self._move(self.get_parameter('slow_speed').value, -self.lane_offset * 0.8)
            self._status(f"YAVAS_{sign}")
            return

        # 14. YENİ LEVHA TETİKLEYİCİLERİ (diğer levhalar)
        if sign in STOP_SIGNS:
            self.get_logger().warn('DUR levhası')
            self.stop_until = now + self.get_parameter('stop_duration').value
            self.state = "STOP"
            self._set_cooldown()
            self._stop()
            self._status("DUR_LEVHASI")
            return

        if sign in NO_ENTRY_SIGNS:
            self.get_logger().warn('GİRİLMEZ → geri + U-dönüşü')
            self.stop_until = now + 1.5          # faz 1: 1.5s geri
            self.turn_until = now + 1.5 + 6.5   # faz 2: 6.5s dönüş (~180°)
            self.state = "NO_ENTRY"
            self._set_cooldown()
            self._stop()
            self._status("GIRILMEZ")
            return

        if sign in TURN_RIGHT_SIGNS:
            self.turn_until = now + self.get_parameter('turn_duration').value
            self.state = "TURN_RIGHT"
            self._set_cooldown()
            self._status("SAGA_DONUS_BASLADI")
            return

        if sign in TURN_LEFT_SIGNS:
            self.turn_until = now + self.get_parameter('turn_duration').value
            self.state = "TURN_LEFT"
            self._set_cooldown()
            self._status("SOLA_DONUS_BASLADI")
            return

        if sign in PASSENGER_SIGNS:
            # GPS ±1m zaten doğru yerde durdurur — levha ek güvencedir
            lane_visible = self.lane_left or self.lane_right
            if self.current_wp_type == "DURAK" and lane_visible:
                wait = random.uniform(self.get_parameter('passenger_wait_min').value,
                                      self.get_parameter('passenger_wait_max').value)
                self.get_logger().warn(f'DURAK levhası onaylandı → {wait:.0f}s bekleme')
                self.passenger_until = now + wait
                self.state = "PASSENGER"
                self._set_cooldown()
                self._stop()
                self._status("DURUYORUM")
                return

        if sign in PARK_SIGNS and self.in_park_zone:
            self.get_logger().info('PARK YERİ levhası + bölge → park başlıyor')
            self.in_park_zone = False
            self._start_park()
            self._set_cooldown()
            return

        if sign in TUNNEL_SIGNS:
            self.state = "TUNNEL"
            self._set_cooldown()
            self._status("TUNEL_BASLADI")
            return

        # 15. NORMAL – standart şerit takibi (durak_nav kontrolündedir, pasif kal)
        if self.loop_n % 30 == 0:
            self._status("NORMAL_DURAK_NAV")
        return


def main(args=None):
    rclpy.init(args=args)
    node = BTDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()