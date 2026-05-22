import rclpy
from rclpy.node import Node
import math

# ROS2 standart mesajları
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Araç için oluşturulmuş özel mesaj tiplerini (smart_can_msgs) projenize göre import etmelisiniz.
# Örnek: from smart_can_msgs.msg import RC_THRT_DATA, AUTONOMOUS_BrakePedalControl, vs.

class VehicleEmulator(Node):
    def __init__(self):
        super().__init__('vehicle_emulator')
        
        # --- PUBLISHERS (ÇIKTILAR) ---
        # Gazebo'daki sanal aracı hareket ettirmek için cmd_vel 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Asıl köprü kodunuzu (Bridge) kandırmak için sahte gerçek hız verisi 
        # Not: Mesaj tipini smart_can_msgs yapısına göre ayarlayın.
        self.speed_pub = self.create_publisher(Float32, '/beemobs/FB_VehicleSpeed', 10)
        
        # --- SUBSCRIBERS (GİRDİLER) ---
        # Bridge modülünden gelen pedallar ve direksiyon [cite: 737]
        self.create_subscription(Float32, '/beemobs/RC_THRT_DATA', self.throttle_callback, 10)
        self.create_subscription(Float32, '/beemobs/AUTONOMOUS_BrakePedalControl', self.brake_callback, 10)
        self.create_subscription(Float32, '/beemobs/steering_target_value', self.steering_callback, 10)
        
        # --- ARAÇ FİZİK DURUMLARI ---
        self.current_speed = 0.0
        self.throttle_pedal = 0.0 # 50-250 arası [cite: 602]
        self.brake_pedal = 0.0    # 0-100 arası [cite: 596]
        self.steering_angle = 0.0 # Radyan cinsinden tekerlek açısı
        self.wheelbase = 2.5      # Aracın dingil mesafesi (L) - Gerçek araç ölçüsüyle güncelleyin [cite: 751]
        
        # Fizik motorunu 20Hz (0.05 saniye) hızında çalıştır
        self.timer = self.create_timer(0.05, self.physics_loop)
        self.get_logger().info("Vehicle Emulator Başlatıldı! Gazebo simülasyonu bekleniyor...")

    def throttle_callback(self, msg):
        self.throttle_pedal = msg.data

    def brake_callback(self, msg):
        self.brake_pedal = msg.data

    def steering_callback(self, msg):
        self.steering_angle = msg.data

    def physics_loop(self):
        # 1. BOYlAMASINA DİNAMİKLER (Hızlanma ve Yavaşlama) [cite: 741]
        if self.throttle_pedal > 50.0:
            # Gaz pedalı 50'nin üzerindeyse aracı ivmelendir [cite: 742, 743]
            acceleration = (self.throttle_pedal - 50.0) * 0.015
            self.current_speed += acceleration
            
        if self.brake_pedal > 0.0:
            # Fren pedalına basıldıysa aracı yavaşlat [cite: 745, 746]
            deceleration = self.brake_pedal * 0.02
            self.current_speed -= deceleration
            
        # Sürtünme: Pedallara basılmıyorsa araç yavaşlayarak durmalı [cite: 748, 749]
        self.current_speed *= 0.99
        
        # Hızın negatif olmasını (geri vites durumu hariç) engelle
        if self.current_speed < 0.0:
            self.current_speed = 0.0

        # 2. YANAL DİNAMİKLER (Ackermann Ters Kinematiği) [cite: 750]
        # Formül: w = (v * tan(delta)) / L
        angular_z = (self.current_speed * math.tan(self.steering_angle)) / self.wheelbase 

        # 3. VERİLERİ YAYINLA
        # Gazebo'yu Sür (cmd_vel)
        twist_msg = Twist()
        twist_msg.linear.x = self.current_speed
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

        # Bridge Kodunu Kandır (FB_VehicleSpeed)
        speed_msg = Float32()
        speed_msg.data = self.current_speed
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    emulator_node = VehicleEmulator()
    rclpy.spin(emulator_node)
    emulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()