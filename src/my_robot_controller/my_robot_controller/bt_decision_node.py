import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
import json
import time

# Levha grupları
STOP_SIGNS        = {"dur"}
NO_ENTRY_SIGNS    = {"girilmez"}
SLOW_SIGNS        = {"yaya_gecidi", "ada_etrafinda_don"}
TURN_RIGHT_SIGNS  = {"saga_mecburi_yon", "ileri_ve_saga_mecburi_yon", "ileriden_saga_mecburi_yon"}
TURN_LEFT_SIGNS   = {"sola_mecburi_yon", "ileri_ve_sola_mecburi_yon", "ileriden_sola_mecburi_yon"}
NO_RIGHT_SIGNS    = {"saga_donulmez"}
NO_LEFT_SIGNS     = {"sola_donulmez"}
PASSENGER_SIGNS   = {"durak"}
PARK_SIGNS        = {"park_yeri"}
TUNNEL_SIGNS      = {"tunel"}

NORMAL_SPEED  = 0.3   # m/s
SLOW_SPEED    = 0.15
TURN_SPEED    = 0.5   # rad/s
LANE_KP       = 1.2   # şerit merkezi PID oranı
TURN_DURATION = 2.5   # saniye
PASSENGER_WAIT_MIN = 15.0
PASSENGER_WAIT_MAX = 20.0
STOP_DURATION = 3.0
COOLDOWN      = 10.0  # aynı levhayı yoksay süresi

# Park manevra parametreleri
PARK_FORWARD_DURATION    = 1.5   # saniye — spot geçişi için ileri git
PARK_REVERSE_DURATION    = 3.0   # saniye — geri + dönüş
PARK_STRAIGHTEN_DURATION = 2.0   # saniye — düzleşme
PARK_REVERSE_SPEED       = -0.10 # m/s
PARK_TURN_SPEED          = 0.45  # rad/s

class BTDecisionNode(Node):
    def __init__(self):
        super().__init__('bt_decision_node')

        # Algı değişkenleri
        self.traffic_light_red    = False
        self.traffic_light_green  = True
        self.current_sign         = None
        self.lane_offset          = 0.0
        self.lane_left_detected   = False
        self.lane_right_detected  = False

        # Durum makinesi
        self.state = "NORMAL"  # NORMAL, STOP, NO_ENTRY, SLOW, TURN_RIGHT, TURN_LEFT, PASSENGER, PARK, TUNNEL

        # Zamanlayıcılar
        self.stop_until          = 0.0
        self.turn_until          = 0.0
        self.passenger_until     = 0.0
        self.sign_cooldown_until = 0.0
        self.green_light_time    = None  # yeşil ışık ne zaman yaktı

        # Park alt-durumu
        self.park_phase       = "IDLE"  # IDLE, ILERI, GERI_DONUS, DUZELME, TAMAM
        self.park_phase_until = 0.0

        # Nav2'nin cmd_vel'i (düşük öncelik — BT override etmediğinde relay edilir)
        self.nav2_cmd: Twist | None = None

        # Subscriptions
        self.create_subscription(Bool,   '/perception/flags/traffic_light_red',   self.cb_red,       10)
        self.create_subscription(Bool,   '/perception/flags/traffic_light_green',  self.cb_green,     10)
        self.create_subscription(String, '/traffic_sign_detections',               self.cb_sign,      10)
        self.create_subscription(Float32,'/perception/lane_offset',                self.cb_lane,      10)
        self.create_subscription(String, '/perception/lane_info',                  self.cb_lane_info, 10)
        self.create_subscription(Twist,  '/cmd_vel_nav2',                          self.cb_nav2,      10)

        # Publishers — /cmd_vel'e tek yazan bu node
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',    10)
        self.status_pub = self.create_publisher(String, '/bt/status',  10)

        # BT döngüsü 10Hz
        self.create_timer(0.1, self.bt_tick)
        self.get_logger().info("BT Decision Node başlatıldı.")

    # ── Callbacks ──────────────────────────────────────────────────────────
    def cb_nav2(self, msg):
        self.nav2_cmd = msg

    def cb_red(self, msg):
        self.traffic_light_red = msg.data
        if msg.data:
            self.green_light_time = None

    def cb_green(self, msg):
        prev = self.traffic_light_green
        self.traffic_light_green = msg.data
        if msg.data and not prev:
            self.green_light_time = time.time()

    def cb_lane(self, msg):
        self.lane_offset = msg.data

    def cb_lane_info(self, msg):
        try:
            info = json.loads(msg.data)
            self.lane_left_detected  = info.get('left', False)
            self.lane_right_detected = info.get('right', False)
        except Exception:
            pass

    def cb_sign(self, msg):
        if time.time() < self.sign_cooldown_until:
            return
        try:
            detections = json.loads(msg.data)
            if detections:
                best = max(detections, key=lambda d: d['confidence'])
                if best['confidence'] >= 0.6:
                    self.current_sign = best['class_name']
        except:
            pass

    # ── Yardımcı ───────────────────────────────────────────────────────────
    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _move(self, speed, angular=0.0):
        t = Twist()
        t.linear.x  = speed
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _set_cooldown(self):
        self.sign_cooldown_until = time.time() + COOLDOWN
        self.current_sign = None

    # ── Ana BT Tick ────────────────────────────────────────────────────────
    def bt_tick(self):
        now = time.time()

        # ── Öncelik 1: KIRMIZI IŞIK ───────────────────────────────────────
        if self.traffic_light_red:
            self._stop()
            self._publish_status("KIRMIZI_ISIK_DUR")
            self.state = "NORMAL"
            return

        # ── Öncelik 2: YEŞİL IŞIK kontrolü ──────────────────────────────
        # Yeşil yaktıktan sonra 5sn içinde hareket etmeli (+40 puan)
        if self.green_light_time and now - self.green_light_time > 5.0:
            self.green_light_time = None  # zaten hareket ediyoruz

        # ── Öncelik 3: Aktif durum makinesi ──────────────────────────────
        if self.state == "STOP":
            if now < self.stop_until:
                self._stop()
                self._publish_status("DUR_LEVHASI_BEKLE")
                return
            else:
                self.state = "NORMAL"

        if self.state == "NO_ENTRY":
            # Girilmez — geri git ve dur
            if now < self.stop_until:
                self._move(-0.1)
                self._publish_status("GIRILMEZ_GERI")
                return
            else:
                self.state = "NORMAL"

        if self.state == "TURN_RIGHT":
            if now < self.turn_until:
                self._move(NORMAL_SPEED, -TURN_SPEED)
                self._publish_status("SAGA_DONUS")
                return
            else:
                self.state = "NORMAL"

        if self.state == "TURN_LEFT":
            if now < self.turn_until:
                self._move(NORMAL_SPEED, TURN_SPEED)
                self._publish_status("SOLA_DONUS")
                return
            else:
                self.state = "NORMAL"

        if self.state == "PASSENGER":
            if now < self.passenger_until:
                self._stop()
                remaining = self.passenger_until - now
                self._publish_status(f"YOLCU_BEKLENIYOR_{remaining:.0f}sn")
                return
            else:
                self.get_logger().info("Yolcu alındı, devam ediliyor.")
                self.state = "NORMAL"

        if self.state == "PARK":
            if self.park_phase == "IDLE":
                self.park_phase = "ILERI"
                self.park_phase_until = now + PARK_FORWARD_DURATION
                self.get_logger().info("Park: ileri hizalama başlıyor.")

            if self.park_phase == "ILERI":
                if now < self.park_phase_until:
                    self._move(NORMAL_SPEED * 0.5)
                    self._publish_status("PARK_HIZALAMA")
                    return
                self.park_phase = "GERI_DONUS"
                self.park_phase_until = now + PARK_REVERSE_DURATION
                self.get_logger().info("Park: geri dönüş başlıyor.")

            if self.park_phase == "GERI_DONUS":
                if now < self.park_phase_until:
                    self._move(PARK_REVERSE_SPEED, -PARK_TURN_SPEED)
                    self._publish_status("PARK_GERI_DONUS")
                    return
                self.park_phase = "DUZELME"
                self.park_phase_until = now + PARK_STRAIGHTEN_DURATION
                self.get_logger().info("Park: düzleşme başlıyor.")

            if self.park_phase == "DUZELME":
                if now < self.park_phase_until:
                    self._move(PARK_REVERSE_SPEED, PARK_TURN_SPEED)
                    self._publish_status("PARK_DUZELME")
                    return
                self.park_phase = "TAMAM"
                self.get_logger().info("Park: tamamlandı.")

            # TAMAM — spotta dur
            self._stop()
            self._publish_status("PARK_TAMAM")
            return

        if self.state == "TUNNEL":
            # Tünel modunda normal hızda ilerle
            angular = -self.lane_offset * 0.8
            self._move(NORMAL_SPEED, angular)
            self._publish_status("TUNEL_GECISI")
            return

        # ── Öncelik 4: Yeni levha kontrolü ───────────────────────────────
        sign = self.current_sign

        if sign in STOP_SIGNS:
            self.get_logger().warn("DUR levhası — 3 saniye duruyorum.")
            self.stop_until = now + STOP_DURATION
            self.state = "STOP"
            self._set_cooldown()
            self._stop()
            self._publish_status("DUR_LEVHASI")
            return

        if sign in NO_ENTRY_SIGNS:
            self.get_logger().warn("GİRİLMEZ — geri gidiyorum.")
            self.stop_until = now + 2.0
            self.state = "NO_ENTRY"
            self._set_cooldown()
            self._publish_status("GIRILMEZ")
            return

        if sign in TURN_RIGHT_SIGNS:
            self.get_logger().info("SAĞA MECBURİ YÖN")
            self.turn_until = now + TURN_DURATION
            self.state = "TURN_RIGHT"
            self._set_cooldown()
            self._publish_status("SAGA_DONUS_BASLADI")
            return

        if sign in TURN_LEFT_SIGNS:
            self.get_logger().info("SOLA MECBURİ YÖN")
            self.turn_until = now + TURN_DURATION
            self.state = "TURN_LEFT"
            self._set_cooldown()
            self._publish_status("SOLA_DONUS_BASLADI")
            return

        if sign in PASSENGER_SIGNS:
            self.get_logger().info("DURAK — yolcu alınıyor (15-20sn).")
            self.passenger_until = now + PASSENGER_WAIT_MIN
            self.state = "PASSENGER"
            self._set_cooldown()
            self._stop()
            self._publish_status("DURAK_YOLCU_ALINIYOR")
            return

        if sign in PARK_SIGNS:
            self.get_logger().info("PARK YERİ — park moduna geçiliyor.")
            self.state = "PARK"
            self.park_phase = "IDLE"
            self._set_cooldown()
            self._publish_status("PARK_BASLADI")
            return

        if sign in TUNNEL_SIGNS:
            self.get_logger().info("TÜNEL — tünel geçişi başlıyor.")
            self.state = "TUNNEL"
            self._set_cooldown()
            self._publish_status("TUNEL_BASLADI")
            return

        # ── Öncelik 5: Normal sürüş — Nav2'yi relay et ───────────────────
        # BT override yoksa Nav2'nin yolunu takip et; nav2 bağlı değilse
        # lane-offset ile fallback yap.
        lane_visible = self.lane_left_detected or self.lane_right_detected

        if sign in SLOW_SIGNS:
            self._move(SLOW_SPEED, -self.lane_offset * 0.8)
            self._publish_status(f"YAVAS_SUR_{sign}")
        elif self.nav2_cmd is not None:
            t = Twist()
            t.linear.x = self.nav2_cmd.linear.x
            if lane_visible:
                # GPS hızını koru, direksiyon şeritten gelsin
                t.angular.z = -self.lane_offset * LANE_KP
                self._publish_status("NORMAL_SURUS_LANE_CENTERED")
            else:
                # Şerit görünmüyor — Nav2'nin angular'ına güven
                t.angular.z = self.nav2_cmd.angular.z
                self._publish_status("NORMAL_SURUS_NAV2")
            self.cmd_pub.publish(t)
        else:
            self._move(NORMAL_SPEED, -self.lane_offset * 0.8)
            self._publish_status("NORMAL_SURUS_LANE")


def main(args=None):
    rclpy.init(args=args)
    node = BTDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
