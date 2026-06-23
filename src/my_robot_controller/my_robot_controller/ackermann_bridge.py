import rclpy
import rclpy.parameter
from rclpy.node import Node
import math
import time

# ROS2 standart mesajları
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ÖNEMLİ: Gerçek araçta smart_can_msgs kullanılmalıdır!
# from smart_can_msgs.msg import RC_THRT_DATA, AUTONOMOUS_BrakePedalControl, rc_unittoOmux

class AckermannBridge(Node):
    def __init__(self):
        super().__init__('ackermann_bridge')

        # use_sim_time=True → get_clock().now() Gazebo saatini kullanır
        # Böylece PID'in dt hesabı fizik adımlarıyla senkronize olur
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL,
                True
            )
        ])
        
        # --- SUBSCRIBERS (GİRDİLER) ---
        # Nav2'den gelen hedef komutlar
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Araçtan (veya Emülatörden) gelen gerçek hız
        self.create_subscription(Float32, '/beemobs/FB_VehicleSpeed', self.speed_callback, 10)
        
        # --- PUBLISHERS (ÇIKTILAR) ---
        # Pedallar ve Direksiyon
        #sonuçları bu topiclere yayınlar
        self.throttle_pub = self.create_publisher(Float32, '/beemobs/RC_THRT_DATA', 10) #gaz(throttle) komutu
        self.brake_pub = self.create_publisher(Float32, '/beemobs/AUTONOMOUS_BrakePedalControl', 10) #fren (brake) komutu
        self.steer_pub = self.create_publisher(Float32, '/beemobs/steering_target_value', 10) #direksiyon hedefi
        
        # Durum Makinesi (Kontağı ve Vitesi açmak için)
        self.state_pub = self.create_publisher(Float32, '/beemobs/rc_unittoOmux', 10) #kontak/vites vb (şu an gerçek mesaj değil, placeholder)

        # --- PID PARAMETRELERİ VE DEĞİŞKENLER ---
        self.Kp = 0.5
        self.Ki = 0.01 #integral şişmesini engellemek için düşürüldü
        self.Kd = 0.005
        self.Kff = 0.05  # Feedforward kazancı: hedef hızla orantılı ön-gaz katkısı
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.target_angular = 0.0
        self.wheelbase = 2.5 # Dingil mesafesi (L)

        # --- GECİKME ANALİZİ DEĞİŞKENLERİ ---
        # Ölçülen şey: sim_time ne kadar hızlı ilerliyor?
        # oran = delta_sim / delta_wall
        # oran ≈ 1.0 → Gazebo gerçek zamanlı (ideal)
        # oran < 1.0 → Gazebo yavaş (örn: 0.35 → gerçek zamandan 3x yavaş)
        self.sim_time_sec = 0.0
        self.latency_log_counter = 0
        self.prev_sim_time = None
        self.prev_wall_time = None
        # /clock BEST_EFFORT QoS ile yayınlanır — subscriber da aynı olmalı
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.clock_callback, clock_qos)
        
        # 1. BAŞLANGIÇ RİTÜELİ (State Machine)
        self.startup_routine()
        
        # Kontrol Döngüsü (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Ackermann Bridge (Araç Sürücüsü) Başlatıldı!")

    def startup_routine(self):
        self.get_logger().info("Araç başlatılıyor: Kontak açılıyor, vites ileriye alınıyor...")
        # NOT: Gerçekte rc_unittoOmux mesaj tipini doldurmanız gerekecek.
        # RC_Ignition: 1 (Kontak Açık) [cite: 622]
        # RC_SelectionGear: 1 (İleri Vites) [cite: 625]
        # AUTONOMOUS_HB_MotState: 1 (El Frenini İndir) [cite: 624]
        time.sleep(1) # Sistemin kendine gelmesi için ufak bir bekleme

    def clock_callback(self, msg):
        # Gazebo'nun sim_time'ını kaydet
        self.sim_time_sec = msg.clock.sec + msg.clock.nanosec / 1e9

    def cmd_vel_callback(self, msg):
        self.target_speed = msg.linear.x #hedef ileri hız
        self.target_angular = msg.angular.z #hedef yaw dönüş hızı

    def speed_callback(self, msg): #gerçek hız
        self.current_speed = msg.data

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # GECİKME ANALİZİ: Her 100 döngüde bir (yaklaşık 5 saniyede bir) logla
        self.latency_log_counter += 1
        if self.latency_log_counter >= 100:
            self.latency_log_counter = 0
            wall_now = time.time()
            sim_now = self.sim_time_sec

            if self.prev_sim_time is not None and sim_now > 0.0:
                delta_sim  = sim_now  - self.prev_sim_time
                delta_wall = wall_now - self.prev_wall_time
                if delta_wall > 0:
                    oran = delta_sim / delta_wall  # 1.0 = gerçek zamanlı ideal
                    # use_sim_time=True olduğu için PID dt sim_time'a bağlı → her oran'da doğru çalışır
                    # Bu log sadece donanım bilgisi için, alarm değil
                    if oran >= 0.85:
                        self.get_logger().info(
                            f"[Sim Hızı] oran={oran:.2f}x (gerçek zamanlı)")
                    elif oran >= 0.2:
                        self.get_logger().info(
                            f"[Sim Hızı] oran={oran:.2f}x (Docker/ARM normal değeri, PID doğru çalışıyor)")
                    else:
                        self.get_logger().warn(
                            f"[Sim Duraksadı] oran={oran:.2f}x → CPU aşırı yüklü olabilir")

            self.prev_sim_time  = sim_now
            self.prev_wall_time = wall_now

        # 2. BOYLAMASINA KONTROL (PID - Gaz/Fren)
        error = self.target_speed - self.current_speed 
        
        throttle_val = 0.0
        brake_val = 0.0
        max_pid = 10.0
        
        # YENİ: Ölü Bölge (Araç tamamen durması gerekiyorsa sistemi rahat bırak)
        if abs(self.target_speed) < 0.01 and abs(error) < 0.05:
            output = 0.0
            self.integral = 0.0
            self.prev_error = 0.0
            throttle_val = 0.0
            brake_val = 0.0
        else:
            self.integral += error * dt 
            # YENİ: PID Anti-Windup Koruması (İntegral Kafesi)
            max_integral = 3.0  # İntegralin şişebileceği maksimum sınır
            min_integral = -3.0 # İntegralin düşebileceği minimum sınır
            self.integral = max(min(self.integral, max_integral), min_integral)
            derivative = (error - self.prev_error) / dt 
            
            # PID + Feedforward Çıktısı
            # Feedforward: PID hatayı beklemeden, hedef hıza göre ön-gaz verir.
            # PID bu katkının üstüne sadece küçük sapmaları düzeltir.
            ff_term = self.target_speed * self.Kff
            output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative) + ff_term
            
            # YENİ: Kesintisiz ve Pürüzsüz Pedal Kontrolü
            if output > 0.0:
                # İleri İvmelenme: 50 değeri zaten aracın 0 noktasıdır. 
                # Çıktı ne kadar artarsa 50'nin üzerine o kadar ekleriz.
                throttle_val = 50.0 + (output * 40.0) 
                throttle_val = min(max(throttle_val, 50.0), 120.0) # Sınır 200
                brake_val = 0.0 
            else:
                # Yavaşlama: Gazı tamamen kesip, eksi çıktı oranında frene bas
                throttle_val = 0.0
                brake_val = (abs(output) * 15.0) 
                brake_val = min(max(brake_val, 0.0), 50.0)
            
        self.prev_error = error
        self.last_time = current_time

        # 3. YANAL KONTROL (Ackermann Kinematiği - Direksiyon)
        steering_angle = 0.0
        # Düşük hızlarda veya dururken de yönlenebilmek için referans hız hesabı
        if abs(self.current_speed) > 0.05:
            ref_speed = self.current_speed
        elif abs(self.target_speed) > 0.05:
            ref_speed = self.target_speed
        else:
            # Sıfıra yakın durumlarda sıfıra bölmeyi önlemek için varsayılan minimum yönlenme referansı
            ref_speed = 0.15 if self.target_speed >= 0 else -0.15
        
        steering_angle = math.atan((self.wheelbase * self.target_angular) / ref_speed)
        # Direksiyon açısını fiziksel sınırlarla sınırla (-0.6 ile 0.6 radyan)
        steering_angle = max(min(steering_angle, 0.6), -0.6)
        
        # VERİLERİ YAYINLA
        t_msg = Float32()
        t_msg.data = float(throttle_val)
        self.throttle_pub.publish(t_msg)
        
        b_msg = Float32()
        b_msg.data = float(brake_val)
        self.brake_pub.publish(b_msg)
        
        s_msg = Float32()
        s_msg.data = float(steering_angle)
        self.steer_pub.publish(s_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()