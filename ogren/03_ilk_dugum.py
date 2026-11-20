#!/usr/bin/env python3
# =============================================================================
# 03 - ILK KENDI DUGUMUN
# =============================================================================
# Calistirmak icin:
#   1) Bir terminalde simulasyonu ac:   bash ogren/02_lidar_gor.sh
#   2) BASKA bir terminalde:            python3 ogren/03_ilk_dugum.py
#
# colcon, paket, derleme YOK. Duz bir Python dosyasi. ROS dugumleri
# boyle de yazilabilir; paket yapisi sadece dagitim ve launch icin gerekir.
#
# Bu dosya 40 satir. Bizim `avoider_node.py` 300 satir ama TEMEL FIKIR AYNI:
#     /scan dinle  ->  karar ver  ->  /cmd_vel yayinla
#
# Degistirerek ogren. Asagida "DENE:" diye isaretli yerler var.
# =============================================================================

import math

import rclpy                                    # ROS 2'nin Python kutuphanesi
from rclpy.node import Node                     # her dugum bundan turer
from rclpy.qos import qos_profile_sensor_data   # sensor verisi icin teslim politikasi
from geometry_msgs.msg import Twist             # hiz komutu mesaji
from sensor_msgs.msg import LaserScan           # lidar mesaji


class BasitDur(Node):
    """Onunde engel gorunce duran, gorunmeyince ilerleyen robot.

    Bu, bir otonom aracin yapabilecegi EN BASIT seydir. Bizim asil
    planlayicimizdan farki: bu sadece 'dur / git' der, oysa gercek
    planlayici 'su yone sap' da diyebilir.
    """

    DUR_MESAFESI = 0.6      # DENE: 0.3 yap -> robot engele daha cok yaklasir
    HIZ = 0.15              # DENE: 0.3 yap -> daha hizli gider

    def __init__(self):
        # Dugume bir isim veriyoruz. `ros2 node list` ciktisinda bu gorunur.
        super().__init__('ogren_basit_dur')

        # ABONE: /scan topic'ini dinle, her mesaj gelince self.tarama_geldi cagrilsin
        self.create_subscription(
            LaserScan, 'scan', self.tarama_geldi, qos_profile_sensor_data)

        # YAYINCI: /cmd_vel topic'ine hiz komutu yazacagiz
        self.komut = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Basladi. Ctrl-C ile durdur.')

    def on_mesafesi(self, msg: LaserScan) -> float:
        """Tam onumuzdeki (+-10 derece) en yakin engelin mesafesi."""
        n = len(msg.ranges)
        yakin = msg.range_max

        for derece in range(-10, 11):
            r = msg.ranges[derece % n]

            # nan  = sensor olcemedi  -> bilgi yok, ATLA
            if r is None or math.isnan(r):
                continue
            # inf  = donus yok        -> o yonde bir sey YOK, yani BOS
            if math.isinf(r) or r >= msg.range_max:
                continue
            # govdeye carpan isin     -> gecersiz
            if r < msg.range_min:
                continue

            yakin = min(yakin, r)

        return yakin

    def tarama_geldi(self, msg: LaserScan) -> None:
        """Her lidar taramasinda bir kez calisir (saniyede ~5 kez)."""
        on = self.on_mesafesi(msg)

        hiz = Twist()
        if on > self.DUR_MESAFESI:
            hiz.linear.x = self.HIZ
            durum = 'GIT'
        else:
            hiz.linear.x = 0.0
            durum = 'DUR'
            # DENE: asagidaki satirin basindaki # isaretini kaldir.
            # Robot durmak yerine saga donmeye baslar -- ve boylece
            # "engelden kacan" bir robota donusur.
            # hiz.angular.z = -0.5

        self.komut.publish(hiz)

        # throttle_duration_sec: ayni mesaji saniyede en fazla 1 kez yaz,
        # yoksa ekran akar.
        self.get_logger().info(
            f'{durum}  on={on:.2f} m  hiz={hiz.linear.x:.2f} m/s',
            throttle_duration_sec=1.0)

    def dur(self) -> None:
        """Kapanirken araci durdur. Bunu unutursan robot son komutla
        ilerlemeye DEVAM eder -- gercek bir araçta tehlikeli."""
        self.komut.publish(Twist())


def main() -> None:
    rclpy.init()
    dugum = BasitDur()
    try:
        rclpy.spin(dugum)          # mesajlar gelene kadar bekle, geldikce isle
    except KeyboardInterrupt:
        pass
    finally:
        dugum.dur()
        dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
