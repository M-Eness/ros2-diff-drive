#!/usr/bin/env python3
"""
GPS Simülatörü – Robotaksi 2026
================================
Ignition /world/robotaksi_world/dynamic_pose/info'dan robot pozisyonunu alır,
referans enlem/boylam'a göre WGS84'e çevirir,
sensor_msgs/NavSatFix olarak yayınlar.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
import math

REF_LAT = 41.015137
REF_LON = 28.979530
EARTH_RADIUS = 6378137.0

ROBOT_NAMES = {'my_robot', 'base_link', 'base_footprint'}

class GPSSimulator(Node):
    def __init__(self):
        super().__init__('gps_simulator')
        self.declare_parameter('ref_lat', REF_LAT)
        self.declare_parameter('ref_lon', REF_LON)
        self.ref_lat = self.get_parameter('ref_lat').value
        self.ref_lon = self.get_parameter('ref_lon').value

        self._logged_ids = False

        self.sub = self.create_subscription(
            TFMessage,
            '/gz/world_poses',
            self.world_poses_cb,
            10
        )
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.get_logger().info('GPS Simülatörü başlatıldı. /gz/world_poses bekleniyor...')

    def enu_to_wgs84(self, x, y):
        dlat = y / EARTH_RADIUS
        dlon = x / (EARTH_RADIUS * math.cos(math.radians(self.ref_lat)))
        return self.ref_lat + math.degrees(dlat), self.ref_lon + math.degrees(dlon)

    def world_poses_cb(self, msg: TFMessage):
        # İlk mesajda mevcut frame ID'leri logla (debug için)
        if not self._logged_ids:
            ids = [t.child_frame_id for t in msg.transforms]
            self.get_logger().info(f'TFMessage child_frame_ids: {ids}')
            self._logged_ids = True

        for t in msg.transforms:
            cid = t.child_frame_id
            # Unscoped name kontrolü (örn: "my_robot" veya "robotaksi_world/my_robot")
            base_name = cid.split('/')[-1].split('::')[-1]
            if base_name not in ROBOT_NAMES:
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y

            # Sanity check: (0,0) pozisyonu geçersiz (henüz başlamadı)
            if abs(x) < 0.001 and abs(y) < 0.001:
                continue

            lat, lon = self.enu_to_wgs84(x, y)

            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'gps'
            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = 0.0
            fix.status.status = 0
            fix.position_covariance_type = 0
            self.pub.publish(fix)
            return  # İlk eşleşen yeterli

def main(args=None):
    rclpy.init(args=args)
    node = GPSSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
