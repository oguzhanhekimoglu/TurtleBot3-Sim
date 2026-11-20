"""Dinamik engel surucusu -- "aniden onune cikan yaya".

Sartname 2026, "Dinamik Engeller":
  "aracin hareketi esnasinda ani ve rastgele bir sekilde yol uzerinde hareketli
   bulunan engellerdir... turlar esnasinda aracin onune aniden cikabilir."
Puanlama: carpmadan sakindi / bekledi / hizini ayarladi -> +50
          carpti -> TURDAN DISKALIFIYE

Gazebo tarafinda engel, `gazebo_ros_planar_move` eklentili bir kutu modeli.
Bu dugum ona /pedestrian/cmd_vel yazarak hareket ettirir.

Iki mod:
  periodic  : iki nokta arasinda surekli gidip gelir (temel test)
  triggered : robot belirlenen x'e gelince karsiya gecmeye BASLAR
              -> gercek "aniden onune cikma" senaryosu. Ani fren ve TTC
                 mantigini test eden tek durum budur; periyodik hareket
                 tesadufe baglidir, tekrarlanabilir degildir.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data

from .runner import run


class PedestrianNode(Node):
    def __init__(self) -> None:
        super().__init__("mergen_pedestrian")

        self.declare_parameter("mode", "triggered")        # periodic | triggered
        self.declare_parameter("speed", 0.45)              # [m/s] yaya hizi
        # DIKKAT -- YAML tuzagi. Bu parametreyi `dynamic_typing` ile tanimliyoruz
        # ve degerini kendimiz normalize ediyoruz. Sebebi olculdu:
        # launch dosyasindan `axis: "y"` gecirince launch_ros bunu bir YAML
        # parametre dosyasina `axis: y` olarak yaziyor; YAML 1.1'de ciplak `y`
        # BOOLEAN'dir (y/yes/n/no/true/false). Yani string bekleyen parametreye
        # `True` geliyor ve dugum acilista `InvalidParameterTypeException` ile
        # coküyor -- Gazebo kosusunda yaya hic yola cikmadi, sebebi buydu.
        # ("Norway problem" olarak bilinen hatanin tek harfli hali.)
        self.declare_parameter(
            "axis", "lateral",
            ParameterDescriptor(
                dynamic_typing=True,
                description="lateral (yolu enine gec) | longitudinal (yol boyunca)",
            ),
        )
        self.declare_parameter("travel", 2.4)              # gidis mesafesi [m]
        self.declare_parameter("trigger_x", 4.0)           # robot bu x'i gecince basla
        self.declare_parameter("trigger_once", True)
        self.declare_parameter("period", 0.05)
        self.declare_parameter("start_delay", 0.0)

        self.mode = str(self.get_parameter("mode").value)
        self.speed = float(self.get_parameter("speed").value)
        self.axis = self._normalize_axis(self.get_parameter("axis").value)
        self.travel = float(self.get_parameter("travel").value)
        self.trigger_x = float(self.get_parameter("trigger_x").value)
        self.trigger_once = bool(self.get_parameter("trigger_once").value)
        self.period = float(self.get_parameter("period").value)
        self.start_delay = float(self.get_parameter("start_delay").value)

        if self.mode not in ("periodic", "triggered"):
            raise ValueError(f"mode 'periodic' veya 'triggered' olmali, verilen: {self.mode}")

        if self.speed <= 0.0 or self.travel <= 0.0:
            raise ValueError("speed ve travel > 0 olmali")

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_robot_odom, qos_profile_sensor_data)

        self._t0: Optional[float] = None
        self._active = (self.mode == "periodic")
        self._fired = False
        self._travelled = 0.0
        self._direction = 1.0

        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f"yaya hazir | mod={self.mode} hiz={self.speed} m/s eksen={self.axis} "
            + (f"tetik x={self.trigger_x} m" if self.mode == "triggered" else
               f"mesafe={self.travel} m")
        )

    def _normalize_axis(self, raw) -> str:
        """`axis` degerini "lateral" / "longitudinal"a indirger.

        YAML'in `y` -> True, `n` -> False cevirmesine karsi dayanikli; boolean
        geldiginde sessizce kabul etmek yerine NE OLDUGUNU soyleyen bir uyari
        basiyoruz, cunku bu hatayi bir kez sessizce yedik.
        """
        if isinstance(raw, bool):
            value = "lateral" if raw else "longitudinal"
            self.get_logger().warn(
                f"'axis' parametresi boolean olarak geldi ({raw}). YAML 1.1 ciplak "
                f"'y'/'n' degerlerini boolean'a cevirir; '{value}' varsayildi. "
                "Karisikligi onlemek icin 'lateral' veya 'longitudinal' yaz."
            )
            return value

        text = str(raw).strip().lower()
        if text in ("lateral", "y", "yolu_enine"):
            return "lateral"
        if text in ("longitudinal", "x", "yol_boyunca"):
            return "longitudinal"
        raise ValueError(
            f"'axis' icin gecersiz deger: {raw!r}. "
            "Kabul edilenler: lateral | longitudinal (kisa hali: y | x)"
        )

    def _on_robot_odom(self, msg: Odometry) -> None:
        if self.mode != "triggered":
            return
        if self._fired and self.trigger_once:
            return
        if msg.pose.pose.position.x >= self.trigger_x and not self._active:
            self._active = True
            self._fired = True
            self._travelled = 0.0
            self.get_logger().warn(
                f"YAYA YOLA CIKTI (robot x={msg.pose.pose.position.x:.2f} m)"
            )

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        if (now - self._t0) < self.start_delay:
            self.pub.publish(Twist())
            return

        cmd = Twist()
        if not self._active:
            self.pub.publish(cmd)
            return

        v = self._direction * self.speed
        if self.axis == "lateral":
            cmd.linear.y = v          # yolu enine gec (yaya gecidi)
        else:
            cmd.linear.x = v          # yol boyunca ilerle (onde giden arac)
        self.pub.publish(cmd)

        self._travelled += abs(v) * self.period
        if self._travelled >= self.travel:
            self._travelled = 0.0
            self._direction *= -1.0           # karsi yone don
            if self.mode == "triggered" and self.trigger_once:
                self._active = False          # tek gecis yapti, dursun
                self.pub.publish(Twist())
                self.get_logger().info("yaya karsiya gecti, durdu")


    def halt(self) -> None:
        """Yayayi durdur. Kapanista cagrilir -- yoksa engel Gazebo'da son
        komutla yurumeye devam eder ve bir sonraki kosuyu kirletir."""
        stop = Twist()
        for _ in range(3):             # DDS'te tek mesaj kaybolabilir
            self.pub.publish(stop)


def main(args=None) -> None:
    run(PedestrianNode, on_stop="halt", args=args)


if __name__ == "__main__":
    main()
