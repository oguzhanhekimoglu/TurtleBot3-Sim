#!/usr/bin/env bash
# =============================================================================
# 01 - KAPLUMBAGA: ROS'un tek fikrini 5 dakikada anla
# =============================================================================
#     bash ogren/01_kaplumbaga.sh
#
# Klavye KULLANMIYORUZ. Resmi egitimler burada `turtle_teleop_key` kullanir
# ve yeni baslayanlarin %90'i orada takilir: o program SADECE kendi terminal
# penceresi ODAKTAYKEN tuslari duyar. Baska bir pencereye tiklarsan
# kaplumbaga kipirdamaz ve "calismiyor" sanirsin.
#
# Onun yerine kaplumbagayi MESAJ gondererek surecegiz. Zaten ogrenmen
# gereken sey bu: ROS'ta her sey mesajdir.
# =============================================================================

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "HATA: Bu terminalde ROS ortami yok."
  echo "Cozum: yeni bir terminal ac (Ctrl+Alt+T) ve tekrar dene."
  exit 1
fi

cleanup() {
  echo
  echo "Kapatiliyor..."
  kill ${TURTLE_PID:-0} 2>/dev/null
  pkill -x turtlesim_node 2>/dev/null
  exit 0
}
trap cleanup INT TERM

echo "=============================================================="
echo " KAPLUMBAGA - ROS'un temel fikri"
echo "=============================================================="
echo
echo "Birazdan bir pencere acilacak ve icinde bir kaplumbaga olacak."
echo "Sonra ona mesaj gonderip hareket ettirecegiz."
echo
sleep 2

ros2 run turtlesim turtlesim_node > /dev/null 2>&1 &
TURTLE_PID=$!
sleep 4

if ! kill -0 $TURTLE_PID 2>/dev/null; then
  echo "HATA: turtlesim penceresi acilamadi."
  echo "Once sunu calistir:  bash ogren/00_kontrol.sh"
  exit 1
fi

echo "--------------------------------------------------------------"
echo " 1) DUGUM (node)"
echo "--------------------------------------------------------------"
echo "Acilan pencere bir DUGUM. Dugum = bir is yapan program."
echo "Su an calisan dugumler:"
echo
ros2 node list | sed 's/^/     /'
echo
sleep 4

echo "--------------------------------------------------------------"
echo " 2) TOPIC (konu)"
echo "--------------------------------------------------------------"
echo "Dugumler birbirine TOPIC uzerinden mesaj yollar."
echo "Kaplumbaganin dinledigi/yayinladigi topic'ler:"
echo
ros2 topic list | sed 's/^/     /'
echo
echo "  /turtle1/cmd_vel  -> kaplumbaganin DINLEDIGI topic (komut)"
echo "  /turtle1/pose     -> kaplumbaganin YAYINLADIGI topic (konum)"
echo
sleep 5

echo "--------------------------------------------------------------"
echo " 3) MESAJ GONDERELIM"
echo "--------------------------------------------------------------"
echo "Simdi /turtle1/cmd_vel topic'ine mesaj yolluyoruz."
echo "Pencereye bak -- kaplumbaga bir kare cizecek."
echo
sleep 2

for i in 1 2 3 4; do
  echo "  [$i/4] ileri git..."
  timeout 2.2 ros2 topic pub -r 10 /turtle1/cmd_vel \
    geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}" >/dev/null 2>&1
  echo "  [$i/4] sola don..."
  timeout 1.7 ros2 topic pub -r 10 /turtle1/cmd_vel \
    geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.95}}" >/dev/null 2>&1
done

echo
echo "--------------------------------------------------------------"
echo " 4) NE OLDU?"
echo "--------------------------------------------------------------"
echo "Gonderdigimiz mesajin tipi: geometry_msgs/msg/Twist"
echo "Icinde iki sey var:"
echo "     linear.x   -> ileri hiz   (m/s)"
echo "     angular.z  -> donme hizi  (rad/s)"
echo
echo "ISTE OTONOM ARACIN TAMAMI BUNUN UZERINE KURULU."
echo "Bizim robotumuz da tam olarak ayni mesaji yolluyor:"
echo "     kaplumbaga -> /turtle1/cmd_vel"
echo "     robot      -> /cmd_vel"
echo
echo "Fark su: kaplumbagaya komutu BIZ yazdik."
echo "Robotta komutu LIDAR VERISINE BAKARAK KOD uretiyor."
echo "Ogrenecegin tek sey, o kodun nasil karar verdigi."
echo
sleep 3

echo "--------------------------------------------------------------"
echo " 5) KAPLUMBAGANIN KONUMUNU OKUYALIM"
echo "--------------------------------------------------------------"
echo "Simdi de kaplumbaganin YAYINLADIGI mesaji okuyoruz:"
echo
timeout 3 ros2 topic echo /turtle1/pose 2>/dev/null | head -8 | sed 's/^/     /'
echo
echo "Bu, bir dugumun veri YAYINLAMASI. Robotta bunun karsiligi:"
echo "     /scan  -> lidar mesafeleri"
echo "     /odom  -> robotun konumu ve hizi"
echo

echo "=============================================================="
echo " BITTI. Ogrendiklerin:"
echo "   dugum   = is yapan program"
echo "   topic   = mesaj kanali"
echo "   yayinci = yazan,  abone = okuyan"
echo "   Twist   = hiz komutu mesaji (linear.x, angular.z)"
echo
echo " Sonraki adim:"
echo "   bash ogren/02_lidar_gor.sh"
echo "=============================================================="
echo
echo "Kapatmak icin Ctrl-C."
wait $TURTLE_PID
