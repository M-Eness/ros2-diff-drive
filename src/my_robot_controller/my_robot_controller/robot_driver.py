import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # angular ve linear hız değerleri için

class RobotDriver(Node):
    def __init__(self):
        super().__init__("robot_driver")

        # Standart robot hız kanalı: /cmd_vel (Command Velocity)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Robot Sürücüsü Başlatıldı! Dikkat, araç kalkıyor...")

    def timer_callback(self):

        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0 # saniyede bir radyan
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()