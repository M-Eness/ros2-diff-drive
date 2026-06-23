#!/usr/bin/env python3
"""
DurakNav — Şerit Öncelikli + GPS Yön Katkılı Navigasyon
MİMARİ:
  L90  (açı <30°) : lane %90, GPS %10  — düz gidiş
  L65  (30-55°)   : lane %65, GPS %35  — hafif dönüş yaklaşımı
  TURN (açı >55°) : saf GPS            — kavşak/sert dönüş
  NOLANE          : conf=0 ise GPS 1.0 m/s yavaş
  PARK wp tipi    : bekleme_s = -1.0 → park manevrasını tetikler
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32

# ── ROTA: (x, y, isim, bekleme_s) ───────────────────────────────────────────
#   bekleme_s = 0    → ara nokta
#   bekleme_s > 0    → DURAK (bekleme_s saniye dur)
#   bekleme_s = -1.0 → PARK (manevra tetikleyici)
#   ✓ = Gazebo ölçümü    ⚠ = tahmini
ROUTE = [
    ( 29.075,   0.796,  'DURAK1',        15.0),  # ✓
    ( 45.4116,  1.9865, 'WP_D2_GIRIS',    0.0),  # ✓
    ( 46.691,   9.462,  'WP_D2_ARA',      0.0),  # ✓
    ( 48.469,  16.954,  'DURAK2',        15.0),  # ✓
    ( 46.591,  27.417,  'WP_D2_CIKIS',    0.0),  # ✓
    ( 45.500,  31.500,  'WP_SAG_KOSE',    0.0),  # sağ üst köşe — dönüş başlangıcı
    ( 18.500,  33.500,  'WP_UST_TL2',     0.0),  # tl_2 önü (18.82, 33.53)
    ( 14.649,  31.617,  'WP_SOL_KOSE',    0.0),  # ✓ ölçüldü — aşağı dönüş köşesi
    ( 11.430, -14.644,  'DURAK3',        15.0),  # ✓
    ( 17.0,   -40.0,    'WP_PARK_GIRIS',  0.0),  # park alanına hizalanma (silindir merkezi x≈17)
    ( 17.0,   -57.5,    'PARK',          -1.0),  # park tetikleyici — silindir y≈-60.8, 3m önce
]

# ── PARAMETRELER ──────────────────────────────────────────────────────────────
WP_RADIUS    = 3.5    # m — ara noktaya "ulaşıldı"
DURAK_RADIUS = 2.0    # m — durağa "ulaşıldı"
PARK_RADIUS  = 3.0    # m — park wp'sine "ulaşıldı"
MAX_SPEED    = 3.5    # m/s
STEER_LIMIT  = 0.5    # rad

LANE_TIMEOUT = 0.5    # s — bu kadar eski lane komutu geçersiz

# Park manevra parametreleri
# Faz 1 — HIZALAMA: ileri giderek konilerle hizala
PARK_ALIGN_SPD  = 0.8    # m/s (ileri)
PARK_ALIGN_DUR  = 4.0    # s — tetikleyici y=-57.5, silindir y=-60.8 → ~3.2m ileri (arasına girer)
# Faz 2 — GERI: silindirler arasında dur
PARK_REV_SPD    = -0.4   # m/s (geri)
PARK_REV_STEER  = 0.0    # rad/s — düz geri
PARK_REV_DUR    = 3.0    # s — ~1.2m geri → y≈-59.6 (iki silindir arasında)
# Faz 3 — DUZELT: düzeltme adımı (gerekirse)
PARK_CORR_SPD   = -0.3   # m/s (geri)
PARK_CORR_STEER = 0.0    # rad/s
PARK_CORR_DUR   = 0.0    # s — 0 ise atla

# Köşe eşikleri: açı büyüdükçe GPS ağırlığı artar
CORNER_SOFT = math.radians(20)   # <20°: düz — lane %90
CORNER_HARD = math.radians(50)   # >50°: köşe — lane %30, GPS %70

NOGPS_STEER_GAIN = 0.6
NOGPS_SPEED      = 1.2

DURAK_CENTER_STEER = 0.20

ROBOT_FRAME = 'my_robot'
POS_ALPHA   = 0.35

TL_ACTIVE    = 1.5
TL_STOP_DIST = 12.0
TL_FOV_HALF  = math.radians(50)

TL_POSITIONS = [
    (10.7719,  0.6677),
    (18.8216, 33.5324),
    (40.6710,  0.0121),
]

EARTH_RADIUS = 6371000.0
REF_LAT      = 41.015137
REF_LON      = 28.979530


def _latlon_to_xy(lat, lon):
    dlat = math.radians(lat - REF_LAT)
    dlon = math.radians(lon - REF_LON)
    x = EARTH_RADIUS * dlon * math.cos(math.radians(REF_LAT))
    y = EARTH_RADIUS * dlat
    return x, y


class DurakNav(Node):

    def __init__(self):
        super().__init__('durak_nav')

        self.robot_x   = None
        self.robot_y   = None
        self.robot_yaw = None
        self._sx       = None
        self._sy       = None
        self._logged   = False

        self.wp_idx      = 0
        self.waiting     = False
        self.wait_until  = 0.0
        self._wait_start = 0.0

        # Park manevra state
        self.parking           = False
        self.park_phase        = 'IDLE'   # IDLE → ALIGN → REVERSE → CORRECT → DONE
        self.park_phase_until  = 0.0

        self.lane_cmd        = None
        self.lane_cmd_time   = 0.0
        self.lane_confidence = 0.0

        self.tl_yolo_color = None;  self.tl_yolo_time  = 0.0
        self.tl_sim_states = {};    self.tl_sim_time   = 0.0

        self.declare_parameter('require_start', False)
        self.declare_parameter('geojson_path',  '')
        req = self.get_parameter('require_start').value
        self._started = not req
        self._lane_ever_seen  = False
        self._conf_high_count = 0
        self._CONF_START_FRAMES = 8   # ~0.4s @ 20Hz — gürültü filtresi
        self._lane_ok_prev    = False  # mod değişimini loglamak için
        self._diag_loop       = 0

        self._route = list(ROUTE)
        gpath = self.get_parameter('geojson_path').value
        if gpath:
            self._load_geojson_route(gpath)

        self.create_subscription(TFMessage, '/gz/world_poses',             self._pose_cb,    10)
        self.create_subscription(Twist,     '/cmd_vel_lane',               self._lane_cb,    10)
        self.create_subscription(Float32,   '/perception/lane_confidence', self._conf_cb,    10)
        self.create_subscription(String,    '/perception/traffic_light',   self._tl_yolo_cb, 10)
        self.create_subscription(String,    '/traffic_light/state',        self._tl_sim_cb,  10)
        self.create_subscription(String,    '/mission/command',            self._cmd_cb,     10)

        self.pub        = self.create_publisher(Twist,   '/cmd_vel',         10)
        self.pub_status = self.create_publisher(String,  '/bt/status',       10)
        self.pub_dist   = self.create_publisher(Float32, '/durak/dist',      10)
        self.pub_rem    = self.create_publisher(Int32,   '/durak/remaining', 10)
        self.pub_wp     = self.create_publisher(String,  '/map/waypoints',   10)

        self.create_timer(0.1, self._control)

        durak_count = sum(1 for r in self._route if r[3] > 0)
        self.get_logger().info(
            f'DurakNav başladı — {len(self._route)} nokta ({durak_count} DURAK) | '
            f'require_start={req}')

    # ── GEOJSON ──────────────────────────────────────────────────────────────
    def _load_geojson_route(self, path):
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            durak_feats = [
                feat for feat in data.get('features', [])
                if feat.get('properties', {}).get('gorev_tipi') == 'DURAK'
            ]
            it = iter(durak_feats)
            new_route = []
            for entry in self._route:
                if entry[3] > 0:
                    try:
                        feat = next(it)
                        lon, lat = feat['geometry']['coordinates'][:2]
                        x, y = _latlon_to_xy(lat, lon)
                        dur = float(feat.get('properties', {}).get('bekleme_suresi', entry[3]))
                        new_route.append((x, y, entry[2], dur))
                        self.get_logger().info(f'GEOJSON {entry[2]}: ({x:.2f}, {y:.2f})')
                    except StopIteration:
                        new_route.append(entry)
                else:
                    new_route.append(entry)
            self._route = new_route
        except Exception as e:
            self.get_logger().error(f'GEOJSON hatası: {e} — hardcoded ROUTE kullanılıyor')

    # ── UMS ──────────────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        if cmd == 'START':
            self._started = True
            self.get_logger().info('UMS GO → görev başladı')
        elif cmd == 'EMERGENCY_STOP':
            self.pub.publish(Twist())
            self._started = False
            self.get_logger().error('EMERGENCY_STOP!')
        elif cmd == 'EMERGENCY_CLEAR':
            self._started = True

    # ── Konum ────────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id != ROBOT_FRAME:
                continue
            x = t.transform.translation.x
            y = t.transform.translation.y
            if abs(x) < 0.001 and abs(y) < 0.001:
                continue
            if self._sx is None:
                self._sx, self._sy = x, y
            else:
                self._sx = POS_ALPHA * x + (1.0 - POS_ALPHA) * self._sx
                self._sy = POS_ALPHA * y + (1.0 - POS_ALPHA) * self._sy
            self.robot_x = self._sx
            self.robot_y = self._sy
            q = t.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if not self._logged:
                self._logged = True
                self.get_logger().info(f'Konum → frame:{ROBOT_FRAME} ({x:.1f},{y:.1f})')
            return

    # ── Lane ─────────────────────────────────────────────────────────────────
    def _lane_cb(self, msg: Twist):
        self.lane_cmd      = msg
        self.lane_cmd_time = time.time()

    def _conf_cb(self, msg: Float32):
        self.lane_confidence = float(msg.data)
        if self.lane_confidence > 0.1:
            self._conf_high_count += 1
        else:
            self._conf_high_count = 0  # streak sıfırla
        if self._conf_high_count >= self._CONF_START_FRAMES and not self._lane_ever_seen:
            self._lane_ever_seen = True
            if not self._started:
                self._started = True
                self.get_logger().warn('Kamera hazır — görev otomatik başladı')

    # ── Trafik lambası ────────────────────────────────────────────────────────
    _TL_MAP = {'Green': 'YESIL', 'Yellow': 'SARI', 'Red': 'KIRMIZI'}

    def _tl_yolo_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            dets = data if isinstance(data, list) else [data]
            best_color, best_conf = None, 0.0
            for d in dets:
                conf = float(d.get('confidence', 0.0))
                if conf < 0.55:
                    continue
                raw   = d.get('class_name') or d.get('class', '')
                color = self._TL_MAP.get(raw)
                if color and conf > best_conf:
                    best_color, best_conf = color, conf
            if best_color:
                self.tl_yolo_color = best_color
                self.tl_yolo_time  = time.time()
        except Exception:
            pass

    def _tl_sim_cb(self, msg: String):
        try:
            states = {}
            for part in msg.data.split(','):
                k, v = part.strip().split('=')
                states[k.strip()] = v.strip()
            self.tl_sim_states = states
            self.tl_sim_time   = time.time()
        except Exception:
            pass

    def _tl_in_front(self):
        if self.robot_x is None or self.robot_yaw is None:
            return None, 999.0
        best_name, best_dist = None, 999.0
        for i, (tx, ty) in enumerate(TL_POSITIONS):
            dx   = tx - self.robot_x
            dy   = ty - self.robot_y
            dist = math.hypot(dx, dy)
            if dist > TL_STOP_DIST:
                continue
            diff = math.atan2(
                math.sin(math.atan2(dy, dx) - self.robot_yaw),
                math.cos(math.atan2(dy, dx) - self.robot_yaw))
            if abs(diff) > TL_FOV_HALF:
                continue
            if dist < best_dist:
                best_dist, best_name = dist, f'tl{i+1}'
        return best_name, best_dist

    def _get_tl(self):
        now = time.time()
        if (now - self.tl_sim_time) < 1.0:
            tl_name, _ = self._tl_in_front()
            if tl_name:
                color = self.tl_sim_states.get(tl_name)
                if color:
                    return color
        if (now - self.tl_yolo_time) < TL_ACTIVE:
            return self.tl_yolo_color
        return None

    # ── Park manevrası ────────────────────────────────────────────────────────
    def _run_parking(self, now: float):
        """3 fazlı park manevrasını yönetir; tamamlandığında PARK_TAMAMLANDI yayınlar."""
        if self.park_phase == 'IDLE':
            self.park_phase       = 'ALIGN'
            self.park_phase_until = now + PARK_ALIGN_DUR
            self.get_logger().info('Park ▶ faz 1/3 — hizalama (ileri)')

        if self.park_phase == 'ALIGN':
            if now < self.park_phase_until:
                cmd = Twist()
                cmd.linear.x = PARK_ALIGN_SPD
                self.pub.publish(cmd)
                self.pub_status.publish(String(data=f'PARK_HIZALAMA ({self.park_phase_until - now:.0f}s)'))
                return
            self.park_phase       = 'REVERSE'
            self.park_phase_until = now + PARK_REV_DUR
            self.get_logger().info('Park ▶ faz 2/3 — geri park')

        if self.park_phase == 'REVERSE':
            if now < self.park_phase_until:
                cmd = Twist()
                cmd.linear.x  = PARK_REV_SPD
                cmd.angular.z = PARK_REV_STEER
                self.pub.publish(cmd)
                self.pub_status.publish(String(data=f'PARK_GERI ({self.park_phase_until - now:.0f}s)'))
                return
            if PARK_CORR_DUR > 0.0:
                self.park_phase       = 'CORRECT'
                self.park_phase_until = now + PARK_CORR_DUR
                self.get_logger().info('Park ▶ faz 3/3 — düzeltme')
            else:
                self.park_phase = 'DONE'
                self.get_logger().info('Park ▶ TAMAMLANDI')

        if self.park_phase == 'CORRECT':
            if now < self.park_phase_until:
                cmd = Twist()
                cmd.linear.x  = PARK_CORR_SPD
                cmd.angular.z = PARK_CORR_STEER
                self.pub.publish(cmd)
                self.pub_status.publish(String(data=f'PARK_DUZELT ({self.park_phase_until - now:.0f}s)'))
                return
            self.park_phase = 'DONE'
            self.get_logger().info('Park ▶ TAMAMLANDI')

        # DONE — tam dur, tekrar yayınlama
        self.pub.publish(Twist())
        self.pub_status.publish(String(data='PARK_TAMAMLANDI'))

    # ── Ana kontrol ───────────────────────────────────────────────────────────
    def _control(self):
        if self.robot_x is None:
            return

        if not self._started:
            self.pub.publish(Twist())
            self.pub_status.publish(String(data='UMS_GO_BEKLENIYOR'))
            return

        # Park manevras aktifse navigasyonu devre dışı bırak
        if self.parking:
            self._run_parking(time.time())
            return

        if self.wp_idx >= len(self._route):
            self.pub.publish(Twist())
            self.pub_status.publish(String(data='GOREV_BITTI'))
            return

        now = time.time()
        rx, ry, rname, rwait = self._route[self.wp_idx]
        is_durak = rwait > 0.0
        is_park  = rwait < 0.0
        radius   = DURAK_RADIUS if is_durak else (PARK_RADIUS if is_park else WP_RADIUS)

        durak_total = sum(1 for r in self._route if r[3] > 0)
        durak_done  = sum(1 for i in range(self.wp_idx) if self._route[i][3] > 0)

        # ── Bekleme ───────────────────────────────────────────────────────────
        if self.waiting:
            rem = max(0.0, self.wait_until - now)
            self.pub_rem.publish(Int32(data=int(rem)))
            self.pub_status.publish(String(data=f'DURUYORUM_{rname} ({rem:.0f}s)'))
            cmd = Twist()
            lane_fresh   = (self.lane_cmd is not None and now - self.lane_cmd_time < 0.5)
            wait_elapsed = now - self._wait_start
            if lane_fresh and wait_elapsed < 5.0 and abs(self.lane_cmd.angular.z) > 0.03:
                cmd.linear.x  = 0.08
                cmd.angular.z = float(max(-DURAK_CENTER_STEER,
                                          min(DURAK_CENTER_STEER,
                                              self.lane_cmd.angular.z * 0.5)))
            self.pub.publish(cmd)
            if rem <= 0.0:
                self.waiting       = False
                self.lane_cmd      = None
                self.lane_cmd_time = 0.0
                self.wp_idx       += 1
                nxt = self._route[self.wp_idx][2] if self.wp_idx < len(self._route) else 'BİTTİ'
                self.get_logger().info(f'Bekleme bitti → {nxt}')
            return

        # ── Mesafe + GPS yönü ─────────────────────────────────────────────────
        dx   = rx - self.robot_x
        dy   = ry - self.robot_y
        dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_err    = math.atan2(math.sin(target_angle - self.robot_yaw),
                                  math.cos(target_angle - self.robot_yaw))

        self.pub_dist.publish(Float32(data=float(dist)))
        wp_type_str = 'PARK' if is_park else ('DURAK' if is_durak else 'WP')
        self.pub_wp.publish(String(data=json.dumps({
            'type': wp_type_str,
            'desc': f'{rname} ({durak_done+1}/{durak_total})' if is_durak else rname,
            'dur':  rwait, 'lat': 0.0, 'lon': 0.0,
        })))
        self.get_logger().info(
            f'→{rname} {dist:.1f}m yaw={math.degrees(self.robot_yaw):.0f}° '
            f'açı={math.degrees(angle_err):.0f}° conf={self.lane_confidence:.1f}',
            throttle_duration_sec=1.0)

        # ── Noktaya ulaşıldı ─────────────────────────────────────────────────
        if dist < radius:
            if is_park:
                self.get_logger().warn(f'{rname} ULAŞILDI → park manevrasını başlatıyor')
                self.parking      = True
                self.park_phase   = 'IDLE'
                self.pub.publish(Twist())
                self.pub_status.publish(String(data='PARK_BASLADI'))
            elif is_durak:
                wait_s = 3.0
                self.get_logger().warn(f'{rname} ULAŞILDI → {wait_s:.1f}s bekleme')
                self.waiting     = True
                self.wait_until  = now + wait_s
                self._wait_start = now
                self.pub.publish(Twist())
                self.pub_status.publish(String(data=f'DURUYORUM_{rname}'))
            else:
                self.get_logger().info(f'{rname} geçildi → sonraki')
                self.wp_idx       += 1
                self.lane_cmd      = None
                self.lane_cmd_time = 0.0
            return

        # ── Trafik lambası ────────────────────────────────────────────────────
        tl_color = self._get_tl()

        if tl_color == 'KIRMIZI':
            self.pub.publish(Twist())
            self.pub_status.publish(String(data='TL_KIRMIZI_DUR'))
            self.get_logger().info('TL KIRMIZI — dur', throttle_duration_sec=1.0)
            return

        tl_speed_cap = 0.8 if tl_color == 'SARI' else MAX_SPEED

        # ── Lane aktif mi? ────────────────────────────────────────────────────
        lane_ok = (self.lane_cmd is not None and
                   now - self.lane_cmd_time < LANE_TIMEOUT and
                   self.lane_confidence > 0.1)

        # ── Navigasyon: açı tabanlı GPS/lane ağırlığı ───────────────────────────
        if lane_ok:
            ae = abs(angle_err)
            if ae < CORNER_SOFT:
                # Düz gidiş — lane güvenilir, GPS sadece küçük katkı
                w_lane, w_gps = 0.90, 0.10
                gps_cap = 0.12
                speed = min(self.lane_cmd.linear.x, tl_speed_cap)
                mode  = 'LANE'
            elif ae < CORNER_HARD:
                # Köşe yaklaşımı — GPS katkısını artır
                t = (ae - CORNER_SOFT) / (CORNER_HARD - CORNER_SOFT)  # 0..1
                w_lane = 0.90 - 0.55 * t   # 0.90 → 0.35
                w_gps  = 1.0 - w_lane
                gps_cap = 0.30
                speed = min(self.lane_cmd.linear.x * (1.0 - 0.3 * t), tl_speed_cap)
                mode  = 'CORNER'
            else:
                # Sert köşe — GPS baskın, lane sadece çarpma önleme
                w_lane, w_gps = 0.30, 0.70
                gps_cap = STEER_LIMIT
                speed = min(0.9, tl_speed_cap)
                mode  = 'TURN'

            if tl_color == 'SARI':
                mode += '_SARI'

            gps_steer = max(-gps_cap, min(gps_cap, angle_err * 0.8))
            cmd_steer = max(-STEER_LIMIT, min(STEER_LIMIT,
                            w_lane * self.lane_cmd.angular.z + w_gps * gps_steer))
        else:
            # Lane yok → GPS ile yavaş ilerle
            gps_steer = max(-STEER_LIMIT * 0.7, min(STEER_LIMIT * 0.7,
                            angle_err * NOGPS_STEER_GAIN))
            cmd_steer = gps_steer
            speed     = NOGPS_SPEED
            mode      = 'NOLANE'

        # ── Durağa / park noktasına yaklaşırken yavaşla ─────────────────────────
        if is_durak:
            if dist < 4.0:
                speed = min(speed, 0.8)
            elif dist < 8.0:
                speed = min(speed, 2.0)
        elif is_park:
            if dist < 5.0:
                speed = min(speed, 0.8)
            elif dist < 12.0:
                speed = min(speed, 1.5)

        tl_suffix = f'_{tl_color}' if tl_color else ''
        status = f'{mode}{tl_suffix}→{rname} {dist:.1f}m açı={math.degrees(angle_err):.0f}°'

        # Mod değişimi uyarısı
        if lane_ok != self._lane_ok_prev:
            if lane_ok:
                self.get_logger().warn('LANE MOD: lane görüldü, şerit takibine geçildi')
            else:
                lane_age = now - self.lane_cmd_time if self.lane_cmd else -1
                self.get_logger().warn(
                    f'NOLANE MOD: lane kaybedildi — '
                    f'conf={self.lane_confidence:.2f} cmd_yaş={lane_age:.2f}s')
            self._lane_ok_prev = lane_ok

        # Her 2 saniyede bir tanı logu
        self._diag_loop += 1
        if self._diag_loop % 20 == 0:
            cmd_age = now - self.lane_cmd_time if self.lane_cmd else -1
            self.get_logger().info(
                f'[{mode}] →{rname} {dist:.1f}m | '
                f'conf={self.lane_confidence:.2f} cmd_yaş={cmd_age:.2f}s | '
                f'steer={cmd_steer:+.2f} spd={speed:.2f}')

        cmd = Twist()
        cmd.linear.x  = float(speed)
        cmd.angular.z = float(cmd_steer)
        self.pub.publish(cmd)
        self.pub_status.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = DurakNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
