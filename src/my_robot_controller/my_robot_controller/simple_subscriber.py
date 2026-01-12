import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("listener_node")# node listte gözükecek resmi isim
        self.subscription = self.create_subscription(
            String,
            'haberlesme_hatti', # dinlenilen topic ismi
            self.listener_callback, # çalıştırılacak fonksiyon (burada çalıştırmıyoruz () yok, sadece fonksiyonu gösteriyoruz)
            10 # kuyruk boyutu
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Duydum: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node) # Sonsuz döngü (dinlemeye devam et)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()