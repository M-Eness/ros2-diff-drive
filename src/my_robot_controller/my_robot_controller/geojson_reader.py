#!/usr/bin/env python3
"""
GEOJSON Okuyucu – Robotaksi 2026
=================================
/map/waypoints topic'ine sırayla görev noktalarını yayınlar.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GeoJSONReader(Node):
    def __init__(self):
        super().__init__('geojson_reader')
        self.declare_parameter('geojson_path', '')
        path = self.get_parameter('geojson_path').value
        if not path:
            self.get_logger().error('GEOJSON dosya yolu belirtilmedi!')
            return
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            self.features = data.get('features', [])
            self.get_logger().info(f'{len(self.features)} görev noktası yüklendi')
        except Exception as e:
            self.get_logger().error(f'GEOJSON yüklenemedi: {e}')
            self.features = []

        self.pub_waypoints = self.create_publisher(String, '/map/waypoints', 10)
        self.sub_done = self.create_subscription(String, '/map/wp_reached', self.wp_reached_cb, 10)

        self.current_idx = 0
        # Periyodik olarak hedefin yayınlanması için 1 Hz zamanlayıcı (subscribers hazır değilse kaçırmasın)
        self.timer = self.create_timer(1.0, self.publish_next_waypoint)

    def publish_next_waypoint(self):
        if self.current_idx < len(self.features):
            feat = self.features[self.current_idx]
            coords = feat['geometry']['coordinates']
            props = feat.get('properties', {})
            wp = {
                'lat': coords[1],
                'lon': coords[0],
                'type': props.get('gorev_tipi', 'MOVE'),
                'desc': props.get('description', ''),
                'dur': props.get('bekleme_suresi', 0)
            }
            msg = String()
            msg.data = json.dumps(wp)
            self.pub_waypoints.publish(msg)
        else:
            # Tüm görevler bitince FINISH yayınla
            msg = String()
            msg.data = 'FINISH'
            self.pub_waypoints.publish(msg)

    def wp_reached_cb(self, msg):
        if msg.data == 'OK':
            self.current_idx += 1
            if self.current_idx < len(self.features):
                self.get_logger().info(f'Hedef tamamlandı. Sıradaki hedefe geçiliyor: {self.current_idx+1}')
                self.publish_next_waypoint()
            else:
                self.get_logger().info('Tüm görev noktaları tamamlandı.')
                self.timer.destroy()
                msg_finish = String()
                msg_finish.data = 'FINISH'
                self.pub_waypoints.publish(msg_finish)

def main(args=None):
    rclpy.init(args=args)
    node = GeoJSONReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
