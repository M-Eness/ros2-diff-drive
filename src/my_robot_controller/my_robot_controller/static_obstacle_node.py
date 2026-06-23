#!/usr/bin/env python3
"""
Engel Algilama v2.0 - Robotaksi TEKNOFEST 2026
===============================================
2D LaserScan tabanlı, O(n) gap-detection algoritması.
lane_controller_node ile /obstacle/state üzerinden entegre çalışır.

Pipeline:
  /scan (LaserScan)
    → Geçerli nokta filtresi
    → XY dönüşümü
    → Gap-detection ile O(n) kümeleme
    → Ön koni engel mesafesi analizi
    → Histerezis filtresi (kararlı STOP/SLOW/CLEAR)
    → /obstacle/state   (JSON → lane_controller)
    → /obstacle/markers (RViz silindir görselleştirme)
    → /obstacle/centroids (PoseArray)
"""

import math
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseArray, Pose
from builtin_interfaces.msg import Duration


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('static_obstacle_detector')

        # ─── PARAMETRELER ─────────────────────────────────────────────
        self.declare_parameter('lidar_topic',        '/scan')
        self.declare_parameter('base_frame',         'base_link')

        # Ön koni: robotun önü ±X derece
        self.declare_parameter('front_angle_deg',    40.0)
        # Dur eşiği (m) — bu mesafenin altında tam dur
        self.declare_parameter('stop_distance',       1.2)
        # Yavaşla eşiği (m) — bu mesafenin altında yavaş git
        self.declare_parameter('slow_distance',       3.0)
        # Temiz sayılmak için arka arkaya kaç frame gerekiyor
        self.declare_parameter('clear_frames_needed',  6)
        # Gap detection: iki bitişik nokta arası bu kadar sıçrama = farklı nesne
        self.declare_parameter('gap_threshold',       0.4)
        # Küme minimum genişliği (m) — gürültüyü at
        self.declare_parameter('min_cluster_width',   0.05)
        # Küme maximum genişliği (m) — duvarı engel sayma
        self.declare_parameter('max_cluster_width',   4.0)
        # Kendi gövdesi nedeniyle çok yakın ölçümleri at
        self.declare_parameter('min_range',           0.25)
        # Lidar maks mesafe (dinamik: msg.range_max den alınır ama fallback)
        self.declare_parameter('max_range',          20.0)

        # ─── QoS ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ─── SUBSCRIBER ───────────────────────────────────────────────
        self.sub = self.create_subscription(
            LaserScan,
            self._p('lidar_topic'),
            self._callback,
            sensor_qos
        )

        # ─── PUBLISHER'LAR ────────────────────────────────────────────
        self.pub_state    = self.create_publisher(String,      '/obstacle/state',    10)
        self.pub_markers  = self.create_publisher(MarkerArray, '/obstacle/markers',  10)
        self.pub_centroids = self.create_publisher(PoseArray,  '/obstacle/centroids', 10)

        # ─── HİSTEREZİS DURUMU ────────────────────────────────────────
        self.active_zone       = 'CLEAR'
        self.clear_frame_count = 0

        self.frame_count = 0
        self.get_logger().info(
            f'ObstacleDetector v2.0 başlatıldı | '
            f'dur={self._p("stop_distance")}m  yavaş={self._p("slow_distance")}m  '
            f'koni=±{self._p("front_angle_deg")}°'
        )

    # ─── YARDIMCI ─────────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    # ─── ANA CALLBACK ─────────────────────────────────────────────────

    def _callback(self, msg: LaserScan):
        self.frame_count += 1

        # 1. Açı ve mesafe dizileri oluştur
        n = len(msg.ranges)
        if n == 0:
            self._publish_state('CLEAR', 99.0, 0.0, [])
            return

        angles = (msg.angle_min
                  + np.arange(n, dtype=np.float32) * msg.angle_increment)
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 2. Geçersiz ölçümleri at
        min_r = max(self._p('min_range'), msg.range_min)
        max_r = min(self._p('max_range'), msg.range_max * 0.99)
        valid = np.isfinite(ranges) & (ranges >= min_r) & (ranges <= max_r)
        angles = angles[valid]
        ranges = ranges[valid]

        if len(ranges) == 0:
            self._publish_state('CLEAR', 99.0, 0.0, [])
            return

        # 3. Polar → Kartezyen
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # 4. Gap-detection kümeleme (O(n))
        clusters = self._gap_detect(angles, ranges, x, y)

        # 5. Ön koni minimum mesafe analizi
        front_rad  = math.radians(self._p('front_angle_deg'))
        front_mask = np.abs(angles) <= front_rad

        if np.any(front_mask):
            front_ranges = ranges[front_mask]
            front_angles = angles[front_mask]
            best          = int(np.argmin(front_ranges))
            min_dist      = float(front_ranges[best])
            closest_angle = float(front_angles[best])
        else:
            min_dist      = 99.0
            closest_angle = 0.0

        # 6. Histerezis ile zone belirle
        zone = self._classify_zone(min_dist)

        # 7. Yayınla
        self._publish_state(zone, min_dist, closest_angle, clusters)
        self._publish_markers(clusters, msg.header)
        self._publish_centroids(clusters, msg.header)

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'[Engel] Zone={zone} | '
                f'Ön min={min_dist:.2f}m açı={math.degrees(closest_angle):.1f}° | '
                f'{len(clusters)} küme'
            )

    # ─── GAP-DETECTION KÜMELEMESİ ─────────────────────────────────────

    def _gap_detect(self, angles, ranges, x, y):
        """
        Açısal sıralı LiDAR noktalarında ardışık noktalar arası
        büyük boşluklara (gap) bakarak kümeleme yapar.
        Karmaşıklık: O(n) — BFS'e göre çok daha hızlı.
        """
        if len(ranges) < 2:
            return []

        # Açıya göre sırala (LiDAR zaten sıralı gelir ama garantile)
        order    = np.argsort(angles)
        sx, sy   = x[order], y[order]
        sr       = ranges[order]

        # Ardışık nokta kartezyen mesafeleri
        dx = np.diff(sx)
        dy = np.diff(sy)
        pt_dist = np.sqrt(dx * dx + dy * dy)

        # Büyük sıçrama = yeni küme başlangıcı
        gap_thr  = self._p('gap_threshold')
        gap_idxs = np.where(pt_dist > gap_thr)[0] + 1
        splits   = np.concatenate([[0], gap_idxs, [len(sr)]])

        min_w = self._p('min_cluster_width')
        max_w = self._p('max_cluster_width')
        clusters = []

        for i in range(len(splits) - 1):
            s, e = splits[i], splits[i + 1]
            if e - s < 2:
                continue

            cx, cy, cr = sx[s:e], sy[s:e], sr[s:e]

            # Küme genişliği (bounding box diyagonali)
            width = math.hypot(
                float(cx.max() - cx.min()),
                float(cy.max() - cy.min())
            )
            if width < min_w or width > max_w:
                continue

            # Range-ağırlıklı centroid (yakın noktalar daha güvenilir)
            w       = 1.0 / (cr + 0.01)
            ctrd_x  = float(np.average(cx, weights=w))
            ctrd_y  = float(np.average(cy, weights=w))
            min_rng = float(cr.min())
            angle_c = math.atan2(ctrd_y, ctrd_x)

            clusters.append({
                'x':     ctrd_x,
                'y':     ctrd_y,
                'dist':  min_rng,
                'width': width,
                'angle': angle_c,
                'pts':   np.column_stack([cx, cy]),
            })

        return clusters

    # ─── HİSTEREZİS ───────────────────────────────────────────────────

    def _classify_zone(self, min_dist: float) -> str:
        """
        Kararlı zone geçişleri:
          • STOP / SLOW → anında geçiş (tehlike öncelikli)
          • → CLEAR → N ardışık temiz frame gerekir (yalancı clear'ı engelle)
        """
        stop_d        = self._p('stop_distance')
        slow_d        = self._p('slow_distance')
        clear_needed  = self._p('clear_frames_needed')

        if min_dist <= stop_d:
            self.clear_frame_count = 0
            self.active_zone = 'STOP'
        elif min_dist <= slow_d:
            self.clear_frame_count = 0
            self.active_zone = 'SLOW'
        else:
            self.clear_frame_count += 1
            if self.clear_frame_count >= clear_needed:
                self.active_zone = 'CLEAR'

        return self.active_zone

    # ─── YAYINLAR ─────────────────────────────────────────────────────

    def _publish_state(self, zone: str, distance: float,
                       angle_rad: float, clusters: list):
        payload = {
            'zone':           zone,
            'distance':       round(distance, 3),
            'angle_deg':      round(math.degrees(angle_rad), 1),
            'obstacle_count': len(clusters),
        }
        self.pub_state.publish(String(data=json.dumps(payload)))

    def _publish_markers(self, clusters, header):
        ma = MarkerArray()

        # Önceki marker'ları sil
        del_m        = Marker()
        del_m.header = header
        del_m.ns     = 'obstacles'
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        slow_d = self._p('slow_distance')
        stop_d = self._p('stop_distance')

        for i, cl in enumerate(clusters):
            m               = Marker()
            m.header        = header
            m.ns            = 'obstacles'
            m.id            = i
            m.type          = Marker.CYLINDER
            m.action        = Marker.ADD
            m.pose.position.x = cl['x']
            m.pose.position.y = cl['y']
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0

            r        = max(cl['width'] / 2.0, 0.08)
            m.scale.x = r * 2
            m.scale.y = r * 2
            m.scale.z = 1.0

            # Turuncu sabit renk
            m.color.r, m.color.g, m.color.b = 1.0, 0.50, 0.0

            m.color.a   = 0.80
            m.lifetime  = Duration(sec=0, nanosec=300_000_000)
            ma.markers.append(m)

        self.pub_markers.publish(ma)

    def _publish_centroids(self, clusters, header):
        pa        = PoseArray()
        pa.header = header
        for cl in clusters:
            p               = Pose()
            p.position.x    = cl['x']
            p.position.y    = cl['y']
            p.position.z    = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_centroids.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
