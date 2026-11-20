#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS 1 (Noetic) dugumu -- ROS 2 surumunun BIREBIR karsiligi.

NEDEN BU DOSYA VAR
------------------
Takim Ubuntu 20.04 + ROS Noetic kullaniyor; bu depo ise ROS 2 Humble ile
yazildi. Ilk bakista "yanlis surumle yazilmis" gibi gorunur. Degil:

    Karar mantiginin TAMAMI (1555 satir) ROS'tan bagimsiz saf Python.
    Tasinmasi gereken tek sey DUGUM KABUGU -- yani bu dosya.

Asagidaki kod, `avoider_node.py` ile ayni isi yapar ve ayni
`perception / planner / control` modullerini cagirir. Fark yalnizca
ROS API'sidir.

ROS 2  ->  ROS 1 KARSILIK TABLOSU
---------------------------------
    rclpy.init()                      rospy.init_node('ad')
    class X(Node)                     sade sinif, taban sinif yok
    create_subscription(T, 'top', cb) rospy.Subscriber('top', T, cb)
    create_publisher(T, 'top', 10)    rospy.Publisher('top', T, queue_size=10)
    create_timer(0.05, cb)            rospy.Timer(rospy.Duration(0.05), cb)
    get_logger().info(...)            rospy.loginfo(...)
    get_clock().now()                 rospy.Time.now()
    declare_parameter / get_parameter rospy.get_param('~ad', varsayilan)
    QoS profilleri                    yok -- yerine queue_size
    rclpy.spin(node)                  rospy.spin()
    finally: temizlik                 rospy.on_shutdown(cb)

Mesaj tipleri DEGISMIYOR: sensor_msgs/LaserScan, geometry_msgs/Twist,
nav_msgs/Odometry ROS 1 ve ROS 2'de ayni isimlerle var.

CALISTIRMA (Ubuntu 20.04 + Noetic makinede)
-------------------------------------------
    # catkin paketi icine koyduysan:
    rosrun mergen_avoidance avoider_node_rospy.py

    # ya da dogrudan:
    python ros1_noetic/avoider_node_rospy.py

DURUST NOT
----------
Bu dosya BU MAKINEDE CALISTIRILMADI. Noetic Ubuntu 20.04 ister, buradaki
sistem 22.04. Sozdizimi kontrol edildi ve ROS 1 API'si satir satir
karsilastirildi, ama gercek bir Noetic kurulumunda denenmedi.
Denemeden "calisiyor" demiyoruz -- projenin kurali bu.
"""
from __future__ import print_function

import math
import os
import sys

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

# --- saf (ROS'suz) modulleri bul --------------------------------------------
# catkin paketi icine tasidiginda bu satirlar gereksizlesir; burada depo
# yapisini bozmadan calisabilsin diye yol ekliyoruz.
_BURASI = os.path.dirname(os.path.abspath(__file__))
_SAF = os.path.join(_BURASI, os.pardir, "ros2_ws", "src", "mergen_avoidance")
if _SAF not in sys.path:
    sys.path.insert(0, _SAF)

from mergen_avoidance.geometry import normalize_angle          # noqa: E402
from mergen_avoidance.params import AvoidParams                # noqa: E402
from mergen_avoidance.perception import ScanGeometry, clean_scan  # noqa: E402
from mergen_avoidance.planner import AvoidancePlanner, DriveState  # noqa: E402


class AvoiderNodeROS1(object):
    """Lidar tabanli sakinma dugumu, ROS 1 surumu.

    ROS 2 surumuyle AYNI mantigi kullanir; tek fark ROS API cagrilaridir.
    """

    def __init__(self):
        rospy.init_node("mergen_avoider", anonymous=False)

        # --- parametreler ----------------------------------------------------
        # ROS 1'de parametreler dugum baslamadan once parameter server'a
        # yuklenir (launch dosyasindan veya rosparam ile). '~' onegi
        # "bu dugume ozel" demektir: ~max_linear_speed -> /mergen_avoider/max_linear_speed
        varsayilan = AvoidParams()
        degerler = {}
        for alan in varsayilan.__dataclass_fields__:
            degerler[alan] = rospy.get_param("~" + alan, getattr(varsayilan, alan))

        try:
            self.params = AvoidParams.from_dict(degerler)
            self.params.validate()
        except ValueError as hata:
            rospy.logfatal("Parametre hatasi: %s", hata)
            raise

        self.world_goal_heading = float(rospy.get_param("~world_goal_heading", 0.0))
        self.use_odom_for_goal = bool(rospy.get_param("~use_odom_for_goal", True))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.5))
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))
        self.log_period = float(rospy.get_param("~log_period", 1.0))
        self.goal_x = float(rospy.get_param("~goal_x", 0.0))

        self.planner = AvoidancePlanner(self.params)

        # --- durum -----------------------------------------------------------
        self._scan = None
        self._scan_stamp = None
        self._measured_linear = 0.0
        self._measured_angular = 0.0
        self._x = 0.0
        self._yaw = 0.0
        self._have_odom = False
        self._last_tick = None
        self._last_log = 0.0
        self._halted_for_timeout = False
        self._goal_reached = False

        # --- arayuz ----------------------------------------------------------
        # ROS 1'de QoS profili YOKTUR. Sensor verisi icin karsiligi
        # queue_size=1: kuyrukta bekleyen eski taramayi tutma, en yenisini al.
        # ROS 2'deki "best effort" mantiginin pratik esdegeri budur.
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub_state = rospy.Publisher("mergen/state", String, queue_size=10)
        self.pub_sectors = rospy.Publisher("mergen/sectors", Float32MultiArray,
                                           queue_size=10)
        self.pub_markers = rospy.Publisher("mergen/markers", MarkerArray,
                                           queue_size=10)

        rospy.Subscriber("scan", LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber("odom", Odometry, self._on_odom, queue_size=1)

        # Kapanista araci durdur. ROS 2'deki `runner.py`nin isini burada
        # rospy kendisi yapiyor -- on_shutdown geri cagrisi context hala
        # gecerliyken calisir.
        rospy.on_shutdown(self.halt)

        rospy.Timer(rospy.Duration(self.params.control_period), self._tick)

        rospy.loginfo(
            "mergen_avoider (ROS1) hazir | %.0f Hz | kinematik=%s | hedef yon=%.0f derece | %s",
            1.0 / self.params.control_period,
            "Ackermann" if self.params.bicycle_mode else "diferansiyel",
            math.degrees(self.world_goal_heading),
            ("bitis x=%.1f m" % self.goal_x) if self.goal_x > 0 else "bitis noktasi yok",
        )

    # ------------------------------------------------------------------ girdi
    def _on_scan(self, msg):
        self._scan = msg
        self._scan_stamp = rospy.Time.now().to_sec()

    def _on_odom(self, msg):
        self._x = msg.pose.pose.position.x
        self._measured_linear = msg.twist.twist.linear.x
        self._measured_angular = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._have_odom = True

    # -------------------------------------------------------------- ana dongu
    def _tick(self, _event):
        """rospy.Timer geri cagrisi. ROS 2'den tek farki: bir TimerEvent
        argumani alir (biz kullanmiyoruz, zamani kendimiz olcuyoruz)."""
        simdi = rospy.Time.now().to_sec()
        dt = self.params.control_period if self._last_tick is None else (simdi - self._last_tick)
        self._last_tick = simdi

        # --- GUVENLIK BEKCISI ---
        # Lidar sustuysa KORU DEVAM ETME. Son taramayla surmeye devam etmek,
        # otonom bir aracta en tehlikeli hatadir.
        eski = (self._scan is None or self._scan_stamp is None
                or (simdi - self._scan_stamp) > self.scan_timeout)
        if eski:
            self._publish(Twist())
            if not self._halted_for_timeout:
                rospy.logwarn("lidar verisi yok / eski -> arac durduruldu (esik %.2f s)",
                              self.scan_timeout)
                self._halted_for_timeout = True
            self._publish_state("NO_SCAN", "lidar verisi yok veya eski -> guvenli durus")
            return
        if self._halted_for_timeout:
            rospy.loginfo("lidar verisi geri geldi")
            self._halted_for_timeout = False

        # --- GOREV TAMAM MI? ---
        if self.goal_x > 0.0 and self._have_odom and self._x >= self.goal_x:
            self._publish(Twist())
            if not self._goal_reached:
                self._goal_reached = True
                rospy.loginfo("GOREV TAMAMLANDI -- hedef x=%.1f m'ye ulasildi "
                              "(konum %.2f m). Arac durduruldu.", self.goal_x, self._x)
            self._publish_state("GOREV_TAMAM", "hedefe ulasildi (x=%.2f m)" % self._x)
            return

        msg = self._scan
        geom = ScanGeometry(
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )
        scan = clean_scan(msg.ranges, geom, self.params.horizon)
        if len(scan) == 0:
            self._publish(Twist())
            self._publish_state("EMPTY_SCAN", "taramada gecerli isin yok -> guvenli durus")
            return

        hedef_govde = None
        if self.use_odom_for_goal and self._have_odom:
            hedef_govde = normalize_angle(self.world_goal_heading - self._yaw)

        karar = self.planner.step(
            scan,
            measured_linear=self._measured_linear,
            measured_angular=self._measured_angular,
            dt=dt,
            goal_heading=hedef_govde,
        )

        komut = Twist()
        komut.linear.x = float(karar.linear)
        komut.angular.z = float(karar.angular)
        self._publish(komut)
        self._publish_state(str(karar.state), karar.reason)
        self._publish_sectors(karar)
        if self.publish_markers:
            self._publish_markers(karar, msg.header.frame_id or "base_scan")

        if simdi - self._last_log >= self.log_period:
            self._last_log = simdi
            r = karar.report
            rospy.loginfo("[%s] v=%+.2f w=%+.2f | on=%.2f sol=%.2f sag=%.2f "
                          "en_yakin=%.2f@%+.0f | %s",
                          karar.state, karar.linear, karar.angular,
                          r.front, r.left, r.right,
                          r.nearest, math.degrees(r.nearest_angle), karar.reason)

    # ------------------------------------------------------------------ cikti
    def _publish(self, komut):
        self.pub_cmd.publish(komut)

    def _publish_state(self, durum, gerekce):
        self.pub_state.publish(String(data="%s|%s" % (durum, gerekce)))

    def _publish_sectors(self, karar):
        r = karar.report
        mesaj = Float32MultiArray()
        mesaj.data = [float(v) for v in (
            r.front, r.front_left, r.front_right, r.left, r.right,
            r.nearest, r.nearest_angle, karar.chosen_heading,
            karar.linear, karar.angular,
        )]
        self.pub_sectors.publish(mesaj)

    def _publish_markers(self, karar, frame_id):
        """RViz gorselleri. ROS 1'de RViz ayni Marker tiplerini kullanir."""
        dizi = MarkerArray()
        zaman = rospy.Time.now()

        renkler = {
            DriveState.CRUISE: (0.1, 0.9, 0.2),
            DriveState.SLOW: (0.9, 0.9, 0.1),
            DriveState.AVOID: (1.0, 0.5, 0.0),
            DriveState.BLOCKED: (1.0, 0.1, 0.1),
            DriveState.EMERGENCY: (1.0, 0.0, 0.6),
            DriveState.RECOVER: (0.6, 0.2, 1.0),
        }
        cr, cg, cb = renkler.get(karar.state, (1.0, 1.0, 1.0))

        ok = Marker()
        ok.header.frame_id = frame_id
        ok.header.stamp = zaman
        ok.ns = "chosen_heading"
        ok.id = 0
        ok.type = Marker.ARROW
        ok.action = Marker.ADD
        uzunluk = max(0.3, karar.report.front)
        ok.points = [
            Point(0.0, 0.0, 0.05),
            Point(uzunluk * math.cos(karar.chosen_heading),
                  uzunluk * math.sin(karar.chosen_heading), 0.05),
        ]
        # ARROW iki noktayla tanimlandiginda scale.z = baslik uzunlugu.
        # 0 birakmak RViz'de "sifir olcek" hatasi verir (ROS 2'de olculdu).
        ok.scale.x, ok.scale.y, ok.scale.z = 0.05, 0.10, 0.12
        ok.color = ColorRGBA(cr, cg, cb, 1.0)
        dizi.markers.append(ok)

        etiket = Marker()
        etiket.header.frame_id = frame_id
        etiket.header.stamp = zaman
        etiket.ns = "state"
        etiket.id = 1
        etiket.type = Marker.TEXT_VIEW_FACING
        etiket.action = Marker.ADD
        etiket.pose.position.z = 0.6
        etiket.pose.orientation.w = 1.0
        etiket.scale.z = 0.18
        etiket.color = ColorRGBA(cr, cg, cb, 1.0)
        etiket.text = "%s v=%.2f w=%+.2f" % (karar.state, karar.linear, karar.angular)
        dizi.markers.append(etiket)

        kapali = Marker()
        kapali.header.frame_id = frame_id
        kapali.header.stamp = zaman
        kapali.ns = "blocked_arcs"
        kapali.id = 2
        kapali.type = Marker.LINE_LIST
        kapali.action = Marker.ADD
        kapali.scale.x = 0.02
        kapali.color = ColorRGBA(1.0, 0.2, 0.2, 0.8)
        yaricap = self.params.stop_distance
        for lo, hi in karar.blocked:
            lo = max(lo, -self.params.max_steer_angle)
            hi = min(hi, +self.params.max_steer_angle)
            if hi <= lo:
                continue
            adim = max(2, int((hi - lo) / 0.05))
            for i in range(adim):
                a0 = lo + (hi - lo) * i / float(adim)
                a1 = lo + (hi - lo) * (i + 1) / float(adim)
                kapali.points.append(Point(yaricap * math.cos(a0),
                                           yaricap * math.sin(a0), 0.02))
                kapali.points.append(Point(yaricap * math.cos(a1),
                                           yaricap * math.sin(a1), 0.02))
        dizi.markers.append(kapali)

        self.pub_markers.publish(dizi)

    # ----------------------------------------------------------------- kapanis
    def halt(self):
        """Kapanirken araci durdur.

        ROS 2'de bunu basarmak icin sinyal isleyicisini devralmak gerekmisti
        (bkz. runner.py): rclpy context'i `finally` blogundan ONCE kapatiyordu
        ve durdurma komutu hic gitmiyordu. ROS 1'de `on_shutdown` geri cagrisi
        context hala gecerliyken calisir, yani sorun kendiliginden yok.
        """
        dur = Twist()
        for _ in range(3):          # tek mesaj kaybolabilir
            try:
                self.pub_cmd.publish(dur)
            except Exception:       # noqa: BLE001 -- kapanista sessiz kal
                break


def main():
    try:
        AvoiderNodeROS1()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
