#!/usr/bin/env python3
"""
Mission Manager v2.0 - Robotaksi TEKNOFEST 2026
================================================
Nav2 bağımlılığı olmadan çalışır.
BT Decision Node'a /mission/command topic üzerinden komut gönderir.

Görev sırası mission.json'dan okunur:
  MOVE    → şerit takibi devam (lane_controller yönetir), bekle
  DURAK   → BT durak levhasını görünce durduruyor; fallback timer
  ISIK    → trafik ışığını BT node yönetir (no-op)
  PARK    → PARK_ZONE → 15s bekle → PARK komutu (levha görülmezse)
"""

import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('mission_file',    '')
        self.declare_parameter('step_interval_s', 30.0)
        self.declare_parameter('auto_start',      True)

        self.cmd_pub = self.create_publisher(String, '/mission/command', 10)
        self.create_subscription(String, '/bt/status', self._cb_status, 10)
        self.create_subscription(String, '/mission/command', self._cb_ext_cmd, 10)

        self.mission           = self._load_mission()
        self.step              = 0
        self.started           = False
        self.waiting_park_done = False
        self.state_hint        = ''

        if self.get_parameter('auto_start').value:
            self._start_timer = self.create_timer(3.0, self._auto_start_cb)

        self.get_logger().info(
            f'MissionManager v2.0 başlatıldı — {len(self.mission)} görev yüklendi'
        )

    # ─── Yükleme ──────────────────────────────────────────────────────────────

    def _load_mission(self):
        param = self.get_parameter('mission_file').value
        if param:
            path = param
        else:
            pkg  = get_package_share_directory('my_robot_controller')
            path = os.path.join(pkg, 'config', 'mission.json')
        try:
            with open(path) as f:
                data = json.load(f)
            self.get_logger().info(f'mission.json yüklendi: {path}')
            return data.get('features', [])
        except Exception as e:
            self.get_logger().error(f'mission.json okunamadı: {e}')
            return []

    # ─── UMS Go / Dış START komutu ───────────────────────────────────────────

    def _cb_ext_cmd(self, msg: String):
        if msg.data == "START" and not self.started:
            self.started = True
            self.get_logger().info("START sinyali alındı → görev başlıyor.")
            self._execute_next()

    # ─── Başlatma ─────────────────────────────────────────────────────────────

    def _auto_start_cb(self):
        self._start_timer.cancel()
        if not self.started:
            self.started = True
            self.get_logger().info('Görev sekansı başlıyor.')
            self._execute_next()

    # ─── BT status dinleyicisi ────────────────────────────────────────────────

    def _cb_status(self, msg: String):
        s = msg.data
        if s == 'PARK_TAMAM' and self.waiting_park_done:
            self.waiting_park_done = False
            self.get_logger().info('Park tamamlandı — sonraki göreve geç.')
            if hasattr(self, '_park_fallback_timer'):
                self._park_fallback_timer.cancel()
            self._schedule_next(5.0)

    # ─── Görev yürütücü ───────────────────────────────────────────────────────

    def _execute_next(self):
        if self.step >= len(self.mission):
            self.get_logger().info('Tüm görevler tamamlandı.')
            return

        feat  = self.mission[self.step]
        props = feat.get('properties', {})
        tip   = props.get('gorev_tipi', 'MOVE')
        desc  = props.get('description', f'Görev {self.step + 1}')
        self.state_hint = tip
        self.step      += 1

        self.get_logger().info(
            f'[{self.step}/{len(self.mission)}] {desc} ({tip})'
        )

        interval = self.get_parameter('step_interval_s').value

        if tip == 'MOVE':
            self._schedule_next(interval)

        elif tip == 'DURAK':
            wait = float(props.get('bekleme_suresi', 15))
            # BT levha görünce kendi durduruyor; biz sadece zaman aşımı ekleriz
            self._schedule_next(wait + 10.0)

        elif tip == 'ISIK':
            # BT node ışık durumunu kendi yönetiyor
            self._schedule_next(interval)

        elif tip == 'PARK':
            self._publish('PARK_ZONE')
            # 15s içinde levha görülmezse doğrudan PARK gönder
            self._park_fallback_timer = self.create_timer(
                15.0, self._park_fallback_cb
            )
            self.waiting_park_done = True

    def _park_fallback_cb(self):
        self._park_fallback_timer.cancel()
        self.get_logger().info('Park levhası bekleme süresi doldu → PARK komutu')
        self._publish('PARK')

    def _schedule_next(self, delay: float):
        self._next_timer = self.create_timer(delay, self._next_timer_cb)

    def _next_timer_cb(self):
        self._next_timer.cancel()
        self._execute_next()

    def _publish(self, cmd: str):
        msg      = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'/mission/command → {cmd}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
