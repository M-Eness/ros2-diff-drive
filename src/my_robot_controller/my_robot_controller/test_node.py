import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleTalker(Node):
    def __init__(self):
        super().__init__('konusan_node')
        self.publisher_ = self.create_publisher(String, 'haberlesme_hatti', 10)
        timer_period = 1.0  # saniye
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Merhaba ROS Dünyası! SAYAÇ: %d' % self.counter
        self.counter += 1
        self.publisher_.publish(msg)
        self.get_logger().info('Yayınlanıyor: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
