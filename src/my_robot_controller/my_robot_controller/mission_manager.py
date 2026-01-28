import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String  # Trafik ışığı verisi için
import json
import time
import os
from ament_index_python.packages import get_package_share_directory

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # 1. Nav2 ile konuşacak "Komutan" nesnesini oluşturuyoruz.
        # Bu, arka planda Action Client kullanarak Nav2'ye emir verir.
        self.navigator = BasicNavigator()
        
        # 2. Kulaklarımızı açıyoruz: Trafik Işığı Dinleyicisi
        # Algılama ekibi hazır olana kadar buraya terminalden veri basacağız.
        self.current_light_status = "GREEN" # Varsayılan olarak yol açık
        self.create_subscription(
            String,
            '/traffic_light_state',
            self.light_callback,
            10
        )
        self.get_logger().info("Mission Manager Başlatıldı. Işık Durumu: GREEN")

    def light_callback(self, msg):
        """Trafik ışığı verisi geldiğinde bu fonksiyon çalışır."""
        previous_status = self.current_light_status
        self.current_light_status = msg.data
        
        if previous_status != self.current_light_status:
            self.get_logger().info(f"🚦 IŞIK DURUMU DEĞİŞTİ: {self.current_light_status}")

    def load_mission(self):
        """mission.json dosyasını okur ve hedefleri listeler."""
        try:
            pkg_share = get_package_share_directory('my_robot_controller')
            file_path = os.path.join(pkg_share, 'config', 'mission.json')
            with open(file_path, 'r') as f:
                data = json.load(f)
                return data['features']
        except Exception as e:
            self.get_logger().error(f"Dosya okuma hatası: {e}")
            return []

    def run_mission(self):
        # Nav2 sisteminin tamamen açılmasını bekle (Önemli!)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 Aktif! Görev Başlıyor...")
        
        gorevler = self.load_mission()

        for item in gorevler:
            coords = item['geometry']['coordinates']
            props = item['properties']
            gorev_tipi = props.get('gorev_tipi', 'MOVE')
            description = props.get('description', 'Bilinmeyen Hedef')
            
            self.get_logger().info(f"--- YENİ HEDEF: {description} ({gorev_tipi}) ---")
            
            # Hedef Pozisyonunu Oluştur
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(coords[0])
            goal_pose.pose.position.y = float(coords[1])
            goal_pose.pose.orientation.w = 1.0 # Şimdilik sabit yön
            
            # Hareketi Başlat
            self.navigator.goToPose(goal_pose)

            # --- SÜREKLİ KONTROL DÖNGÜSÜ (BEYİN) ---
            # Robot hedefe gidene kadar bu döngü sürekli döner
            while not self.navigator.isTaskComplete():
                
                # KURAL 1: Kırmızı Işık Kontrolü [cite: 133]
                # Eğer görev tipi "ISIK" ise ve ışık "RED" ise durmalıyız.
                if gorev_tipi == "ISIK" and self.current_light_status == "RED":
                    self.get_logger().warn("🛑 KIRMIZI IŞIK ALGILANDI! Robot Durduruluyor...")
                    
                    # 1. Mevcut görevi iptal et (Fren yap)
                    self.navigator.cancelTask()
                    
                    # 2. Yeşil olana kadar bekle (Blocking loop)
                    while self.current_light_status == "RED":
                        self.get_logger().info("⏳ Işık Kırmızı... Bekleniyor...", throttle_duration_sec=2)
                        time.sleep(0.5)
                        # ROS callback'lerinin çalışması için kısa bir uyuma
                        
                    self.get_logger().info("🟢 YEŞİL IŞIK! Yola Devam Ediliyor...")
                    # 3. Görevi tekrar gönder (Kaldığı yerden değil, baştan planlar)
                    goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    self.navigator.goToPose(goal_pose)

                # Feedback (Geri bildirim) alabiliriz (Kalan mesafe vs.)
                # feedback = self.navigator.getFeedback()

            # Döngüden çıktık, yani ya vardık ya hata aldık.
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("✅ Hedefe Varıldı.")
                
                # Senaryo Gereği Bekleme (Yolcu Alma vb.) [cite: 630]
                if gorev_tipi == "DURAK":
                    wait_time = props.get('bekleme_suresi', 5)
                    self.get_logger().info(f"⏳ Yolcu Alınıyor... ({wait_time} sn)")
                    time.sleep(wait_time)
                    
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("⚠️ Görev İptal Edildi!")
            elif result == TaskResult.FAILED:
                self.get_logger().error("❌ Navigasyon Başarısız Oldu!")

def main():
    rclpy.init()
    node = MissionManager()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()