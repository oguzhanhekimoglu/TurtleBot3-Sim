"""ROS KATMANI entegrasyon testi -- Gazebo GEREKTIRMEZ.

`test_algorithm.py` karar mantigini test eder ve ROS'a hic dokunmaz.
Bu dosya tam tersini yapar: mantigi degil, KABUGU test eder.

    sahte /scan + /odom yayinla  ->  dugum calissin  ->  /cmd_vel geldi mi?

Neden ayri ve neden degerli: aradaki butun ROS'a ozgu seyler ancak burada
kirilir ve hicbiri birim testinde gorunmez --
  * parametre tanimlama (declare_parameter) ve YAML'dan okuma
  * QoS uyusmazligi (best-effort abone / reliable yayinci)
  * topic isim cozumlemesi (relative "scan" -> "/scan")
  * timer periyodu ve saat kaynagi
  * mesaj alan adlari ve tipleri (Marker.points, ColorRGBA...)
  * guvenlik bekcisi (lidar susunca duruyor mu?)

Kosmak icin:
    source /opt/ros/humble/setup.bash
    python3 test/test_ros_node.py
veya ROS ortaminda:  pytest test/test_ros_node.py
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import MarkerArray

from mergen_avoidance.avoider_node import AvoiderNode

N_BEAMS = 360


def make_scan(blocked_degrees=(), distance=0.9, stamp=None) -> LaserScan:
    """TurtleBot3 LDS-01 ile ayni duzende bir tarama uretir.

    `blocked_degrees`: engelin bulundugu aci indeksleri (0 = ileri, CCW).
    Geri kalan isinlar `inf` -- yani BOS alan.
    """
    msg = LaserScan()
    msg.header.frame_id = "base_scan"
    if stamp is not None:
        msg.header.stamp = stamp
    msg.angle_min = 0.0
    msg.angle_max = 2.0 * math.pi
    msg.angle_increment = 2.0 * math.pi / N_BEAMS
    msg.range_min = 0.12
    msg.range_max = 3.5
    blocked = {int(d) % N_BEAMS for d in blocked_degrees}
    msg.ranges = [distance if i in blocked else float("inf") for i in range(N_BEAMS)]
    return msg


def make_odom(vx=0.0, wz=0.0) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.pose.pose.orientation.w = 1.0          # yaw = 0
    msg.twist.twist.linear.x = vx
    msg.twist.twist.angular.z = wz
    return msg


class Harness(Node):
    """Dugumu besleyen ve ciktilarini toplayan test kosum takimi."""

    def __init__(self) -> None:
        super().__init__("mergen_test_harness")
        self.pub_scan = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, "odom", qos_profile_sensor_data)

        self.cmds: list[Twist] = []
        self.states: list[str] = []
        self.sectors: list[list[float]] = []
        self.markers: list[MarkerArray] = []

        self.create_subscription(Twist, "cmd_vel", self.cmds.append, 10)
        self.create_subscription(String, "mergen/state",
                                 lambda m: self.states.append(m.data), 10)
        self.create_subscription(Float32MultiArray, "mergen/sectors",
                                 lambda m: self.sectors.append(list(m.data)), 10)
        self.create_subscription(MarkerArray, "mergen/markers", self.markers.append, 10)

    def clear(self) -> None:
        self.cmds.clear()
        self.states.clear()
        self.sectors.clear()
        self.markers.clear()

    def feed(self, executor, scan_factory, seconds: float, hz: float = 20.0,
             vx: float = 0.0, wz: float = 0.0) -> None:
        """`seconds` boyunca tarama + odometri yayinlayip dugume isletir."""
        period = 1.0 / hz
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.pub_scan.publish(scan_factory())
            self.pub_odom.publish(make_odom(vx, wz))
            end = time.time() + period
            while time.time() < end:
                executor.spin_once(timeout_sec=0.005)

    def silence(self, executor, seconds: float) -> None:
        """Hicbir sey yayinlamadan sadece dugumu isletir (bekci testi)."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.01)


def run() -> int:
    rclpy.init()
    node = AvoiderNode()
    harness = Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)

    failures: list[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        if condition:
            print(f"  OK   {name}")
        else:
            failures.append(f"{name}{(' -- ' + detail) if detail else ''}")
            print(f"  HATA {name} {detail}")

    try:
        # ---------------------------------------------------------------
        print("=== 1) parametreler dogru tanimlandi mi? ===")
        check("41 algoritma parametresi + dugum parametreleri tanimli",
              node.has_parameter("max_linear_speed")
              and node.has_parameter("bicycle_mode")
              and node.has_parameter("scan_timeout")
              and node.has_parameter("world_goal_heading"))
        check("varsayilanlar AvoidParams ile tutarli",
              abs(node.params.max_linear_speed - 0.22) < 1e-9,
              f"max_linear_speed={node.params.max_linear_speed}")

        # ---------------------------------------------------------------
        print("\n=== 2) bos dunya -> ileri gitmeli ===")
        harness.clear()
        harness.feed(executor, lambda: make_scan(), seconds=1.5, vx=0.1)
        check("/cmd_vel yayinlandi", len(harness.cmds) > 10,
              f"{len(harness.cmds)} mesaj")
        check("/mergen/state yayinlandi", len(harness.states) > 10,
              f"{len(harness.states)} mesaj")
        check("/mergen/sectors yayinlandi", len(harness.sectors) > 10,
              f"{len(harness.sectors)} mesaj")
        check("/mergen/markers yayinlandi", len(harness.markers) > 10,
              f"{len(harness.markers)} mesaj")
        if harness.cmds:
            last = harness.cmds[-1]
            check("ileri hiz pozitif", last.linear.x > 0.15,
                  f"v={last.linear.x:.3f}")
            check("acisal hiz ~0 (duz gidiyor)", abs(last.angular.z) < 0.1,
                  f"w={last.angular.z:.3f}")
        if harness.states:
            check("durum CRUISE", harness.states[-1].startswith("CRUISE"),
                  harness.states[-1][:60])
        if harness.markers:
            ns = {m.ns for m in harness.markers[-1].markers}
            check("RViz isaretleri eksiksiz",
                  {"chosen_heading", "state", "blocked_arcs"} <= ns, str(ns))

        # ---------------------------------------------------------------
        print("\n=== 3) sagda engel -> sola donmeli ===")
        harness.clear()
        right = list(range(270, 360)) + list(range(0, 15))   # sag yari + biraz on
        harness.feed(executor, lambda: make_scan(right, 0.9), seconds=1.5, vx=0.1)
        if harness.cmds:
            last = harness.cmds[-1]
            check("acisal hiz pozitif (sola)", last.angular.z > 0.05,
                  f"w={last.angular.z:.3f}")
        if harness.sectors:
            s = harness.sectors[-1]
            # sira: front, front_left, front_right, left, right, nearest, ...
            check("sag sektor sol sektordan yakin", s[4] < s[3],
                  f"sag={s[4]:.2f} sol={s[3]:.2f}")
            check("secilen yon pozitif", s[7] > 0.0, f"heading={s[7]:.3f}")

        # ---------------------------------------------------------------
        print("\n=== 4) cok yakin duvar -> acil fren ===")
        harness.clear()
        wall = list(range(330, 360)) + list(range(0, 31))     # +-30 derece on
        harness.feed(executor, lambda: make_scan(wall, 0.20), seconds=2.0, vx=0.22)
        if harness.cmds:
            check("hiz sifira indi", abs(harness.cmds[-1].linear.x) < 0.02,
                  f"v={harness.cmds[-1].linear.x:.3f}")
        if harness.states:
            seen = {s.split("|")[0] for s in harness.states}
            check("EMERGENCY durumuna girdi", "EMERGENCY" in seen, str(seen))

        # ---------------------------------------------------------------
        print("\n=== 5) GUVENLIK BEKCISI: lidar susarsa dursun ===")
        harness.clear()
        harness.feed(executor, lambda: make_scan(), seconds=0.8, vx=0.2)
        moving = harness.cmds[-1].linear.x if harness.cmds else 0.0
        check("bekci testi oncesinde arac hareket halinde", moving > 0.15,
              f"v={moving:.3f}")

        # Lidar'i sustur ve DURMA ANINI olc.
        # Dikkat: ilk `scan_timeout` saniyesinde komutlarin sifir olmasini
        # BEKLEMIYORUZ -- o pencerede veri hala "taze" sayilir ve dugum son
        # taramayla surmeye devam eder. Dogru beklenti: zaman asimi dolduktan
        # sonra komutlar sifir olmali ve sifir KALMALI.
        harness.clear()
        t_silent = time.time()
        t_stopped = None
        while time.time() - t_silent < 1.5:
            executor.spin_once(timeout_sec=0.01)
            if t_stopped is None and harness.cmds:
                last = harness.cmds[-1]
                if abs(last.linear.x) < 1e-9 and abs(last.angular.z) < 1e-9:
                    t_stopped = time.time() - t_silent

        check("zaman asimindan sonra durdu", t_stopped is not None,
              "hic sifir komut gelmedi")
        if t_stopped is not None:
            # Rampali duruş da var, bu yüzden üst sınır cömert.
            check("durma suresi scan_timeout ile tutarli",
                  node.scan_timeout - 0.1 <= t_stopped <= node.scan_timeout + 0.9,
                  f"{t_stopped:.2f} s (scan_timeout={node.scan_timeout:.2f} s)")
            print(f"       -> lidar sustuktan {t_stopped:.2f} s sonra sifir komut "
                  f"(0.22 m/s'de ~{0.22 * t_stopped * 100:.0f} cm yol)")

        # Kararli durum: son 10 komut sifir olmali
        tail = harness.cmds[-10:]
        check("sifirda KALDI (son 10 komut)",
              len(tail) == 10 and all(abs(c.linear.x) < 1e-9 and abs(c.angular.z) < 1e-9
                                      for c in tail),
              f"son 10: {[round(c.linear.x, 3) for c in tail]}")
        if harness.states:
            check("durum NO_SCAN", harness.states[-1].startswith("NO_SCAN"),
                  harness.states[-1][:60])

        # ---------------------------------------------------------------
        print("\n=== 6) bozuk tarama (hepsi nan) -> guvenli durus ===")
        harness.clear()

        def nan_scan():
            m = make_scan()
            m.ranges = [float("nan")] * N_BEAMS
            return m

        harness.feed(executor, nan_scan, seconds=1.0, vx=0.1)
        check("bozuk taramada cokmedi ve sifir komut verdi",
              bool(harness.cmds) and all(abs(c.linear.x) < 1e-9 for c in harness.cmds),
              f"{len(harness.cmds)} mesaj")
        if harness.states:
            check("durum EMPTY_SCAN", harness.states[-1].startswith("EMPTY_SCAN"),
                  harness.states[-1][:60])

        # ---------------------------------------------------------------
        print("\n=== 7) canli parametre degisimi ===")
        harness.clear()
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter("max_linear_speed",
                                       Parameter.Type.DOUBLE, 0.08)])
        # Not: dugum parametreleri baslangicta okuyor; bu test parametre
        # ARAYUZUNUN calistigini dogrular (ros2 param set ile ayni yol).
        check("parametre yazilabildi",
              abs(node.get_parameter("max_linear_speed").value - 0.08) < 1e-9)
        node.set_parameters([Parameter("max_linear_speed",
                                       Parameter.Type.DOUBLE, 0.22)])

        # ---------------------------------------------------------------
        print("\n=== 8) kapanista arac durduruluyor mu? ===")
        harness.clear()
        node.halt()
        for _ in range(40):
            executor.spin_once(timeout_sec=0.01)
        check("halt() sifir komut yayinladi",
              bool(harness.cmds) and all(c.linear.x == 0.0 and c.angular.z == 0.0
                                         for c in harness.cmds),
              f"{len(harness.cmds)} mesaj")

    finally:
        executor.remove_node(node)
        executor.remove_node(harness)
        node.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()

    print()
    if failures:
        print(f"{len(failures)} KONTROL BASARISIZ:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("ROS katmani: tum kontroller gecti")
    return 0


# pytest ile de kosabilmesi icin
def test_ros_layer():
    assert run() == 0


if __name__ == "__main__":
    sys.exit(run())
