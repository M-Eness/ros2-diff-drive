#!/usr/bin/env python3
"""
GPS Takipçi – Robotaksi 2026
=============================
GPS ile waypoint'e yönlendirme, durakta durma ve bekleme,
IMU tabanlı yönelim ve lokal ENU koordinat sistemi ile
hassas ve kararlı takip algoritması.
"""

import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

# Dünya merkezli referans (robotaksi_world başlangıcı)
REF_LAT = 41.015137
REF_LON = 28.979530
EARTH_RADIUS = 6378137.0  # metre

class GPSFollower(Node):
    def __init__(self):
        super().__init__('gps_follower')
        
        # Parametreler
        self.declare_parameter('arrival_radius', 1.0)  # metre
        self.declare_parameter('max_speed', 0.5)      # m/s (simülasyonda güvenli hız)
        self.declare_parameter('steer_gain', 1.5)     # orantısal direksiyon kazancı
        self.declare_parameter('ref_lat', REF_LAT)
        self.declare_parameter('ref_lon', REF_LON)

        self.ref_lat = self.get_parameter('ref_lat').value
        self.ref_lon = self.get_parameter('ref_lon').value

        # GPS ve IMU abonelikleri
        self.sub_gps = self.create_subscription(NavSatFix, '/fix', self.gps_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        
        # Waypoint aboneliği
        self.sub_wp = self.create_subscription(String, '/map/waypoints', self.wp_cb, 10)
        
        # Şerit offset/angle (ileride hibrit kontrol için)
        self.sub_offset = self.create_subscription(Float32, '/perception/lane_offset', self.offset_cb, 10)
        self.sub_angle  = self.create_subscription(Float32, '/perception/target_angle', self.angle_cb, 10)
        self.sub_lane_cmd = self.create_subscription(Twist, '/cmd_vel_lane', self.lane_cmd_cb, 10)

        # Yayıncılar
        # Simülasyonu kontrol etmek için /cmd_vel_gps konusuna yazıyoruz
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_gps', 10)
        self.pub_wp_reached = self.create_publisher(String, '/map/wp_reached', 10)

        # Durum değişkenleri
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = 0.0      # radyan (ENU frame, 0 = Doğu)
        self.target_wp = None       # {'lat':..., 'lon':..., 'type':..., 'dur':...}
        self.target_reached = False
        
        # Yolcu Bekleme Durumları
        self.waiting = False
        self.wait_until = None

        self.lane_offset = 0.0
        self.lane_angle_deg = 0.0
        self.lane_cmd = None
        self.lane_cmd_time = None

        # Kontrol döngüsü (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("GPS Follower Node başlatıldı.")

    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def imu_cb(self, msg):
        # Quaternion -> Euler Yaw dönüşümü (ENU frame)
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def wp_cb(self, msg):
        try:
            if msg.data == 'FINISH':
                self.get_logger().info('FINISH sinyali alındı. Görev tamamlandı.')
                self.target_wp = None
                return

            wp = json.loads(msg.data)

            # START noktasına GPS gerektirmeden anında geç
            if wp.get('type') == 'START':
                if self.target_wp is None or self.target_wp.get('type') != 'START':
                    self.get_logger().info('START waypointi atlandı, DURAK1\'e geçiliyor')
                    ok = String()
                    ok.data = 'OK'
                    self.pub_wp_reached.publish(ok)
                return

            # Eğer yeni bir waypoint geldiyse durumları sıfırla
            if self.target_wp is None or self.target_wp['lat'] != wp['lat'] or self.target_wp['lon'] != wp['lon']:
                self.target_wp = wp
                self.target_reached = False
                self.waiting = False
                self.get_logger().info(f'Yeni hedef: {wp["type"]} ({wp["lat"]:.6f}, {wp["lon"]:.6f}) - Açıklama: {wp["desc"]}')
        except Exception as e:
            self.get_logger().error(f'Waypoint okunamadı: {e}')

    def offset_cb(self, msg):
        self.lane_offset = msg.data

    def angle_cb(self, msg):
        self.lane_angle_deg = msg.data

    def lane_cmd_cb(self, msg):
        self.lane_cmd = msg
        self.lane_cmd_time = self.get_clock().now()

    def wgs84_to_enu(self, lat, lon):
        """WGS84 (derece) -> ENU (metre) dönüşümü."""
        dlat = math.radians(lat - self.ref_lat)
        dlon = math.radians(lon - self.ref_lon)
        y = dlat * EARTH_RADIUS
        x = dlon * EARTH_RADIUS * math.cos(math.radians(self.ref_lat))
        return x, y

    def control_loop(self):
        # Veri veya hedef yoksa hiçbir şey yayınlama (bt_decision normal_speed kullanır)
        if self.current_lat is None or self.target_wp is None:
            return

        # Hedefe olan ENU koordinat farkını bul
        curr_x, curr_y = self.wgs84_to_enu(self.current_lat, self.current_lon)
        wp_x, wp_y = self.wgs84_to_enu(self.target_wp['lat'], self.target_wp['lon'])

        # Mesafe ve hedef açısı
        dist = math.sqrt((wp_x - curr_x)**2 + (wp_y - curr_y)**2)
        target_angle_enu = math.atan2(wp_y - curr_y, wp_x - curr_x)

        # Direksiyon hatası
        angle_error = target_angle_enu - self.current_yaw
        # [-pi, pi] aralığına normalize et
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        arrival_r = self.get_parameter('arrival_radius').value
        # START noktası için başlangıçtaki gecikmelerden dolayı daha büyük bir tolerans (örn: 15.0m) kullanıyoruz
        wp_arrival_radius = 15.0 if self.target_wp['type'] == 'START' else arrival_r

        # Hedefe ulaştık mı?
        if dist < wp_arrival_radius:
            if not self.target_reached:
                self.get_logger().info(f'Hedef yarıçapına girildi: {self.target_wp["type"]}')
                self.target_reached = True

            # DURAK ve PARK tiplerinde durma/bekleme bt_decision_node'a bırakılır
            if self.target_wp['type'] in ('DURAK', 'PARK'):
                pass

            # Diğer tipler (START, MOVE vb.) direkt OK yayınla
            else:
                self.get_logger().info(f'Noktaya ulaşıldı: {self.target_wp["type"]}. Sonraki hedefe geçiliyor.')
                msg_reached = String()
                msg_reached.data = 'OK'
                self.pub_wp_reached.publish(msg_reached)
                self.target_wp = None
                self.pub_cmd.publish(Twist())
                return

        # Normal sürüş kontrolü
        # Orantısal direksiyon kontrolü (Pure Pursuit mantığında açısal hata düzeltme)
        gps_steer = angle_error * self.get_parameter('steer_gain').value
        # Limit direksiyon açısı (Ackermann limitlerine göre)
        gps_steer = max(-0.6, min(0.6, gps_steer))

        # Hız kontrolü
        speed = self.get_parameter('max_speed').value
        if dist < 3.0:
            # Hedefe yaklaşırken hızı yavaşlat
            speed = min(speed, 0.2)

        # Şerit taze mi?
        now = self.get_clock().now()
        lane_fresh = (self.lane_cmd is not None and self.lane_cmd_time is not None and
                      (now.nanoseconds - self.lane_cmd_time.nanoseconds) / 1e9 < 0.5)

        if lane_fresh and self.target_wp['type'] in ('DURAK', 'PARK'):
            blend_start = 4.0
            blend_end = arrival_r
            
            if dist > blend_start:
                # Uzaktayken şeritte kal
                steer = self.lane_cmd.angular.z
                speed = min(self.lane_cmd.linear.x, speed)
            else:
                # Yakındayken harmanla
                gps_weight = (blend_start - dist) / (blend_start - blend_end)
                gps_weight = max(0.0, min(1.0, gps_weight))
                steer = (1.0 - gps_weight) * self.lane_cmd.angular.z + gps_weight * gps_steer
        else:
            # Şerit yoksa veya durak dışı bir waypoint ise tamamen GPS
            steer = gps_steer

        # Komutu gönder
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steer
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GPSFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
