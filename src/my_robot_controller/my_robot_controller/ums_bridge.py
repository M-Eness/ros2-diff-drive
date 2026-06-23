#!/usr/bin/env python3
"""
UMS Bridge v1.0 – Robotaksi TEKNOFEST 2026
===========================================
UMS-1 (Kill): /ums/kill (Bool) → araç anında durur (EMERGENCY_STOP)
UMS-2 (Go):   /ums/go   (Bool) → görev başlar (START)

Kill rising edge → bt_decision_node'a EMERGENCY_STOP komutu
Kill falling edge → EMERGENCY_CLEAR
Go rising edge → mission_manager'a START komutu (bir kez, bırakılınca reset)

Simülasyonda test:
  ros2 topic pub /ums/kill std_msgs/msg/Bool "data: true"
  ros2 topic pub /ums/go   std_msgs/msg/Bool "data: true"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class UMSBridge(Node):

    def __init__(self):
        super().__init__("ums_bridge")

        self.declare_parameter("kill_topic", "/ums/kill")
        self.declare_parameter("go_topic",   "/ums/go")

        self.create_subscription(
            Bool, self.get_parameter("kill_topic").value, self._cb_kill, 10)
        self.create_subscription(
            Bool, self.get_parameter("go_topic").value, self._cb_go, 10)

        self.pub_mission = self.create_publisher(String, "/mission/command", 10)

        self.prev_kill = False
        self.go_sent   = False

        self.get_logger().info("UMS Bridge hazır | kill=%s  go=%s" % (
            self.get_parameter("kill_topic").value,
            self.get_parameter("go_topic").value,
        ))

    def _cb_kill(self, msg: Bool):
        active = bool(msg.data)
        if active and not self.prev_kill:
            self.get_logger().error("UMS-1 KİLL AKTİF → EMERGENCY_STOP!")
            self._pub("EMERGENCY_STOP")
        elif not active and self.prev_kill:
            self.get_logger().info("UMS-1 Kill serbest → EMERGENCY_CLEAR")
            self._pub("EMERGENCY_CLEAR")
        self.prev_kill = active

    def _cb_go(self, msg: Bool):
        if bool(msg.data) and not self.go_sent:
            self.get_logger().info("UMS-2 GO basıldı → START")
            self._pub("START")
            self.go_sent = True
        elif not bool(msg.data):
            self.go_sent = False

    def _pub(self, cmd: str):
        m = String()
        m.data = cmd
        self.pub_mission.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = UMSBridge()
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
