#!/usr/bin/env bash
# =============================================================================
# 02 - LIDAR: robotun gozu neye benziyor?
# =============================================================================
#     bash ogren/02_lidar_gor.sh
#
# Gazebo'yu PENCERESIZ aciyoruz (cok daha hizli). Robot duracak, biz lidar
# verisine bakacagiz. Sonra robotu elle birkac metre surup on mesafenin
# nasil azaldigini izleyecegiz.
#
# Sakinma dugumu KAPALI -- robot kendi kendine karar vermiyor, sadece biz
# bakiyoruz.
# =============================================================================

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "HATA: Bu terminalde ROS ortami yok. Yeni bir terminal ac."
  exit 1
fi

# Asili Gazebo en sik sorun; sessizce temizle.
if pgrep -x gzserver > /dev/null 2>&1; then
  echo "Asili kalan Gazebo temizleniyor..."
  pkill -x gzserver; pkill -x gzclient; sleep 2
  pkill -9 -x gzserver 2>/dev/null; sleep 1
fi

cleanup() {
  echo
  echo "Kapatiliyor..."
  kill ${LAUNCH_PID:-0} 2>/dev/null
  sleep 2
  pkill -x gzserver 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null
  exit 0
}
trap cleanup INT TERM

echo "=============================================================="
echo " LIDAR - robotun gozu"
echo "=============================================================="
echo
echo "Gazebo penceresiz aciliyor (15 sn)..."

ros2 launch mergen_avoidance avoid.launch.py \
  world:=w2_cone_slalom gui:=false rviz:=false metrics:=false avoider:=false \
  > /tmp/ogren_lidar.log 2>&1 &
LAUNCH_PID=$!
sleep 15

if ! ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
  echo "HATA: /scan topic'i yok, simulasyon acilmadi."
  echo "Log: /tmp/ogren_lidar.log"
  cleanup
fi

echo "[OK] Simulasyon hazir."
echo
echo "--------------------------------------------------------------"
echo " 1) /scan MESAJI NEYE BENZIYOR?"
echo "--------------------------------------------------------------"
ros2 topic echo /scan --once 2>/dev/null | head -12 | sed 's/^/     /'
echo "     ... (ranges: 360 tane sayi)"
echo
echo "Onemli alanlar:"
echo "     angle_min       : ilk isinin acisi (radyan)"
echo "     angle_increment : isinlar arasi aci farki"
echo "     range_min/max   : olcebildigi en yakin/en uzak mesafe"
echo "     ranges[]        : 360 mesafe degeri, her biri bir aci"
echo
echo "ranges[0]   = tam ONDEKI mesafe"
echo "ranges[90]  = SOLDAKI mesafe"
echo "ranges[270] = SAGDAKI mesafe"
echo
sleep 6

echo "--------------------------------------------------------------"
echo " 2) 'inf' NE DEMEK?"
echo "--------------------------------------------------------------"
echo "Bazi degerler 'inf' (sonsuz) gorunur. Bu HATA DEGIL:"
echo "o yonde 3.5 m icinde hicbir sey YOK demek -- yani BOS ALAN."
echo
echo "Bizim kodumuzda (perception.py) en kritik kural bu:"
echo "     inf  -> 'bos alan' olarak YAZILIR, atilmaz"
echo "     nan  -> atilir (sensor o isini olcemedi)"
echo
echo "Eger inf'leri atarsan robot bos koridoru DUVAR sanir ve kilitlenir."
echo "Yeni baslayanlarin en sik yaptigi hata budur."
echo
sleep 6

echo "--------------------------------------------------------------"
echo " 3) ROBOTU SURELIM, ON MESAFEYI IZLEYELIM"
echo "--------------------------------------------------------------"
echo "Robot ileri gidecek, onundeki koniyi gorunce DURACAK."
echo "Dikkat et: on mesafe surekli azaliyor, sonra robot duruyor."
echo
sleep 2

# Bu kucuk dugum, otonom aracin tam ozeti:
#     /scan oku  ->  karar ver  ->  /cmd_vel yaz
# Acik dongu ("12 saniye ileri git") yerine KAPALI dongu kullaniyoruz:
# robot ne gordugune gore karar veriyor. Fark onemli -- acik dongude
# robot engeli gorse de durmaz.
python3 -u - <<'PYEOF'
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

DUR_MESAFESI = 0.7

class Okuyucu(Node):
    def __init__(self):
        super().__init__('ogren_lidar_okuyucu')
        self.create_subscription(LaserScan, 'scan', self.geldi, qos_profile_sensor_data)
        self.komut = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sayac = 0
        self.durdu = False

    def on_mesafesi(self, msg):
        """Tam onumuzdeki (+-10 derece) en yakin engel."""
        n = len(msg.ranges)
        degerler = []
        for d in range(-10, 11):
            r = msg.ranges[d % n]
            if r is None or math.isnan(r):
                continue                      # sensor olcemedi
            if math.isinf(r) or r >= msg.range_max:
                degerler.append(msg.range_max)  # BOS alan
            elif r >= msg.range_min:
                degerler.append(r)
        return min(degerler) if degerler else msg.range_max

    def geldi(self, msg):
        on = self.on_mesafesi(msg)
        self.sayac += 1

        hiz = Twist()
        if on > DUR_MESAFESI and not self.durdu:
            hiz.linear.x = 0.15
            durum = "GIT"
        else:
            hiz.linear.x = 0.0
            durum = "DUR"
            self.durdu = True
        self.komut.publish(hiz)

        if self.sayac % 3 == 0 or self.durdu:
            cubuk = "#" * max(0, int((3.5 - on) * 12))
            print(f"   {durum}  on={on:4.2f} m   {cubuk}", flush=True)

        if self.durdu and self.sayac % 3 == 0:
            raise SystemExit

rclpy.init()
d = Okuyucu()
try:
    rclpy.spin(d)
except (KeyboardInterrupt, SystemExit):
    pass
finally:
    d.komut.publish(Twist())      # araci durdur
    d.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PYEOF

echo
echo "Robot koniyi gorup durdu."
echo

echo "--------------------------------------------------------------"
echo " 4) BAGLANTIYI KUR"
echo "--------------------------------------------------------------"
echo "Yukarida gordugun o 'on mesafe' hesabi, bizim kodumuzda"
echo "     perception.py -> cone_distance()"
echo "fonksiyonunun yaptigi seyin basitlestirilmis hali."
echo
echo "Robot bu sayiya bakip karar veriyor:"
echo "     on > 2.0 m  -> CRUISE  (tam hiz)"
echo "     on < 1.4 m  -> AVOID   (sakin)"
echo "     on < 0.45 m -> BLOCKED (dur)"
echo "     on < 0.25 m -> EMERGENCY (ani fren)"
echo
echo "=============================================================="
echo " Sonraki adim -- kendi dugumunu yaz:"
echo "   python3 ogren/03_ilk_dugum.py"
echo " (bu simulasyon acikken, BASKA bir terminalde)"
echo "=============================================================="
echo
echo "Simulasyon acik kaliyor. Kapatmak icin Ctrl-C."
wait $LAUNCH_PID
