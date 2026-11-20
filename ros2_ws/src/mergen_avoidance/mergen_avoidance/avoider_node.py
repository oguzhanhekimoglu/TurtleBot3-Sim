"""ROS 2 dugumu: lidar tabanli engelden sakinma.

Bu dosya KASITLI OLARAK INCE. Tek isi:
    ROS mesaji  ->  saf Python veri yapisi  ->  planlayici  ->  ROS mesaji

Tum karar mantigi perception.py / planner.py / control.py icinde ve ROS'tan
habersiz. Faydasi somut:
  * mantigi test etmek icin ROS/Gazebo baslatmak gerekmiyor (33 birim test)
  * ayni planlayici gercek araca baglanirken sadece BU dosya degisiyor
    (topic isimleri /scan -> /velodyne_points, /cmd_vel -> /beemobs/...)

Abone:   /scan   (sensor_msgs/LaserScan)
         /odom   (nav_msgs/Odometry)       -- olculen hiz, kilitlenme tespiti
Yayin:   /cmd_vel                (geometry_msgs/Twist)
         /mergen/state           (std_msgs/String)        -- durum + gerekce
         /mergen/sectors         (std_msgs/Float32MultiArray)
         /mergen/markers         (visualization_msgs/MarkerArray)  -- RViz
"""
from __future__ import annotations

import math
from dataclasses import fields
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import normalize_angle
from .params import AvoidParams
from .perception import ScanGeometry, clean_scan
from .planner import AvoidancePlanner, DriveState
from .runner import run


class AvoiderNode(Node):
    def __init__(self) -> None:
        super().__init__("mergen_avoider")

        # -- parametreler: AvoidParams alanlarini otomatik ROS parametresi yap --
        # Elle tek tek yazmak yerine dataclass'tan ureterek, parametre eklerken
        # iki yeri guncelleme zorunlulugunu (ve kacinilmaz tutarsizligi) kaldiriyoruz.
        defaults = AvoidParams()
        for f in fields(AvoidParams):
            self.declare_parameter(f.name, getattr(defaults, f.name))
        values = {f.name: self.get_parameter(f.name).value for f in fields(AvoidParams)}
        try:
            self.params = AvoidParams.from_dict(values)
            self.params.validate()
        except ValueError as exc:
            self.get_logger().fatal(f"Parametre hatasi: {exc}")
            raise

        # Dugume ozel (algoritmaya ait olmayan) ayarlar
        self.declare_parameter("world_goal_heading", 0.0)
        self.declare_parameter("use_odom_for_goal", True)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("log_period", 1.0)
        self.declare_parameter("goal_x", 0.0)
        """Gorev bitis noktasi [m, odom x]. 0 veya altinda ise KAPALI.

        Bu olmadan arac "+x yonune git" der ve engeller bitince sonsuza
        kadar ilerler -- ilk bakista hata gibi gorunur ama degil: salt
        tepkisel bir planlayicinin HEDEFI YOKTUR. Gercek sistemde hedef
        GEOJSON gorev noktalarindan gelir. Burada tek sayilik basit hali."""
        self.world_goal_heading = float(self.get_parameter("world_goal_heading").value)
        self.use_odom_for_goal = bool(self.get_parameter("use_odom_for_goal").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.goal_x = float(self.get_parameter("goal_x").value)

        self.planner = AvoidancePlanner(self.params)

        # -- durum --
        self._scan: Optional[LaserScan] = None
        self._scan_stamp: Optional[float] = None
        self._measured_linear = 0.0
        self._measured_angular = 0.0
        self._x = 0.0
        self._yaw = 0.0
        self._goal_reached = False
        self._have_odom = False
        self._last_tick: Optional[float] = None
        self._last_log = 0.0
        self._halted_for_timeout = False

        # -- arayuz --
        # /scan icin sensor QoS (best-effort): tek kare kaybetmek, geciken kareyi
        # beklemekten iyidir. Gazebo'nun yayincisi reliable; best-effort abone
        # reliable yayinciyla uyumludur.
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "mergen/state", 10)
        self.pub_sectors = self.create_publisher(Float32MultiArray, "mergen/sectors", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "mergen/markers", 10)

        self.create_timer(self.params.control_period, self._tick)
        self.get_logger().info(
            f"mergen_avoider hazir | {1.0 / self.params.control_period:.0f} Hz | "
            f"kinematik={'Ackermann' if self.params.bicycle_mode else 'diferansiyel'} | "
            f"hedef yon={math.degrees(self.world_goal_heading):.0f} derece | "
            + (f"bitis x={self.goal_x:.1f} m" if self.goal_x > 0 else "bitis noktasi yok")
        )

    # ---------------------------------------------------------------- girdi
    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_stamp = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._measured_linear = msg.twist.twist.linear.x
        self._measured_angular = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        # Kuaterniyondan yaw. Tam donusum yerine sadece z ekseni bileseni:
        #   yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._have_odom = True

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ ana dongu
    def _tick(self) -> None:
        now = self._now()
        dt = self.params.control_period if self._last_tick is None else (now - self._last_tick)
        self._last_tick = now

        # --- GUVENLIK BEKCISI ---------------------------------------------
        # Lidar sustuysa KORU DEVAM ETME. Son taramayla surmeye devam etmek,
        # otonom bir aracta en tehlikeli hatadir: dunya degisirken arac eski
        # resme gore gaz verir.
        stale = (self._scan is None or self._scan_stamp is None
                 or (now - self._scan_stamp) > self.scan_timeout)
        if stale:
            self._publish(Twist())
            if not self._halted_for_timeout:
                self.get_logger().warn(
                    "lidar verisi yok / eski -> arac durduruldu "
                    f"(esik {self.scan_timeout:.2f} s)"
                )
                self._halted_for_timeout = True
            self._publish_state("NO_SCAN", "lidar verisi yok veya eski -> guvenli durus")
            return
        if self._halted_for_timeout:
            self.get_logger().info("lidar verisi geri geldi")
            self._halted_for_timeout = False

        # --- GOREV TAMAM MI? ---
        # Guvenlik bekcisinden SONRA bakiyoruz: lidar susmussa zaten durduk,
        # "gorev bitti" demek yanlis olurdu.
        if self.goal_x > 0.0 and self._have_odom and self._x >= self.goal_x:
            self._publish(Twist())
            if not self._goal_reached:
                self._goal_reached = True
                self.get_logger().info(
                    f"GOREV TAMAMLANDI -- hedef x={self.goal_x:.1f} m'ye ulasildi "
                    f"(konum {self._x:.2f} m). Arac durduruldu."
                )
            self._publish_state("GOREV_TAMAM",
                                f"hedefe ulasildi (x={self._x:.2f} m)")
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

        # Hedef yon: dunya cercevesinde tanimli, govdeye cevrilerek verilir.
        goal_body = None
        if self.use_odom_for_goal and self._have_odom:
            goal_body = normalize_angle(self.world_goal_heading - self._yaw)

        decision = self.planner.step(
            scan,
            measured_linear=self._measured_linear,
            measured_angular=self._measured_angular,
            dt=dt,
            goal_heading=goal_body,
        )

        cmd = Twist()
        cmd.linear.x = float(decision.linear)
        cmd.angular.z = float(decision.angular)
        self._publish(cmd)
        self._publish_state(str(decision.state), decision.reason)
        self._publish_sectors(decision)
        if self.publish_markers:
            self._publish_markers(decision, msg.header.frame_id or "base_scan")

        if now - self._last_log >= self.log_period:
            self._last_log = now
            r = decision.report
            self.get_logger().info(
                f"[{decision.state}] v={decision.linear:+.2f} w={decision.angular:+.2f} | "
                f"on={r.front:.2f} sol={r.left:.2f} sag={r.right:.2f} "
                f"en_yakin={r.nearest:.2f}@{math.degrees(r.nearest_angle):+.0f} | "
                f"{decision.reason}"
            )

    # --------------------------------------------------------------- cikti
    def _publish(self, cmd: Twist) -> None:
        self.pub_cmd.publish(cmd)

    def _publish_state(self, state: str, reason: str) -> None:
        self.pub_state.publish(String(data=f"{state}|{reason}"))

    def _publish_sectors(self, decision) -> None:
        r = decision.report
        out = Float32MultiArray()
        # Sira sabit: metrik kaydedici ve grafikler bu siraya guveniyor.
        out.data = [float(x) for x in (
            r.front, r.front_left, r.front_right, r.left, r.right,
            r.nearest, r.nearest_angle, decision.chosen_heading,
            decision.linear, decision.angular,
        )]
        self.pub_sectors.publish(out)

    def _publish_markers(self, decision, frame_id: str) -> None:
        """RViz gorselleri: secilen yon (yesil ok) + kapali aci araliklari (kirmizi).

        Bunu yazmanin maliyeti 30 satir, getirisi buyuk: 'robot neden sola
        dondu?' sorusunu loglardan okumak yerine GORUYORSUN.
        """
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        colours = {
            DriveState.CRUISE: (0.1, 0.9, 0.2),
            DriveState.SLOW: (0.9, 0.9, 0.1),
            DriveState.AVOID: (1.0, 0.5, 0.0),
            DriveState.BLOCKED: (1.0, 0.1, 0.1),
            DriveState.EMERGENCY: (1.0, 0.0, 0.6),
            DriveState.RECOVER: (0.6, 0.2, 1.0),
        }
        cr, cg, cb = colours.get(decision.state, (1.0, 1.0, 1.0))

        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = stamp
        arrow.ns = "chosen_heading"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        length = max(0.3, decision.report.front)
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.05),
            Point(x=length * math.cos(decision.chosen_heading),
                  y=length * math.sin(decision.chosen_heading), z=0.05),
        ]
        # ARROW iki noktayla tanimlandiginda:
        #   scale.x = govde capi, scale.y = baslik capi,
        #   scale.z = baslik uzunlugu.
        # scale.z'yi 0 birakmak RViz'de 'sifir olcek' hatasi verir ve
        # tum MarkerArray goruntusu kirmizi yanar (olculdu).
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.10, 0.12
        arrow.color = ColorRGBA(r=cr, g=cg, b=cb, a=1.0)
        arr.markers.append(arrow)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "state"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = 0.6
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color = ColorRGBA(r=cr, g=cg, b=cb, a=1.0)
        label.text = f"{decision.state} v={decision.linear:.2f} w={decision.angular:+.2f}"
        arr.markers.append(label)

        blocked = Marker()
        blocked.header.frame_id = frame_id
        blocked.header.stamp = stamp
        blocked.ns = "blocked_arcs"
        blocked.id = 2
        blocked.type = Marker.LINE_LIST
        blocked.action = Marker.ADD
        blocked.scale.x = 0.02
        blocked.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.8)
        radius = self.params.stop_distance
        for lo, hi in decision.blocked:
            lo = max(lo, -self.params.max_steer_angle)
            hi = min(hi, +self.params.max_steer_angle)
            if hi <= lo:
                continue
            steps = max(2, int((hi - lo) / 0.05))
            for i in range(steps):
                a0 = lo + (hi - lo) * i / steps
                a1 = lo + (hi - lo) * (i + 1) / steps
                blocked.points.append(Point(x=radius * math.cos(a0), y=radius * math.sin(a0), z=0.02))
                blocked.points.append(Point(x=radius * math.cos(a1), y=radius * math.sin(a1), z=0.02))
        arr.markers.append(blocked)

        self.pub_markers.publish(arr)

    # ------------------------------------------------------------ kapanis
    def halt(self) -> None:
        """Dugum kapanirken araci durdur. Bunu atlamak, Gazebo'da robotun
        son komutla duvara dogru surmeye devam etmesi demek."""
        stop = Twist()
        for _ in range(3):           # UDP/DDS'te tek mesaj kaybolabilir
            self.pub_cmd.publish(stop)


def main(args=None) -> None:
    # Kapanis mantigi `runner.run` icinde: Ctrl-C'de araci GERCEKTEN durdurmak
    # icin sinyal isleyicisini rclpy'ye birakmiyoruz. Sebebi runner.py'de.
    run(AvoiderNode, on_stop="halt", args=args)


if __name__ == "__main__":
    main()
