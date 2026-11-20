"""Kosu metriklerini CSV'ye yazar ve kapanista ozet basar.

Neden ayri bir dugum? Cunku Robotaksi simulasyon raporu sablonu (2026) acikca
su ikisini istiyor:
    8.2. Test Sonuclari (Tablo ve Grafikler)
    Veri Kayitlari (Logs): zaman damgasi, sensor verisi, karar asamalari
Ayrica sartname: "itirazlar durumunda araclarin temel verileri (hiz, konum,
sensor okumalari) kaydetmesi talep edilebilir."

Yani loglama bir lüks degil, puanlanan bir gereksinim.

CSV kolonlari:  t, x, y, yaw, v_cmd, w_cmd, v_odom, w_odom,
                front, front_left, front_right, left, right,
                nearest, nearest_angle, chosen_heading, state
"""
from __future__ import annotations

import csv
import math
import os
from datetime import datetime
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from .runner import run

HEADER = [
    "t", "x", "y", "yaw", "v_cmd", "w_cmd", "v_odom", "w_odom",
    "front", "front_left", "front_right", "left", "right",
    "nearest", "nearest_angle", "chosen_heading", "state",
]


class MetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("mergen_metrics_logger")

        self.declare_parameter("output_dir", os.path.expanduser("~/mergen_results"))
        self.declare_parameter("scenario", "unnamed")
        self.declare_parameter("period", 0.05)
        self.declare_parameter("collision_distance", 0.20)
        """Govde yuzeyine bu kadar yaklasma 'temas/surtme' sayilir [m].
        Sartnamede 'surterek sakindi -> -10 puan', 'carpti -> diskalifiye'."""

        out_dir = str(self.get_parameter("output_dir").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.period = float(self.get_parameter("period").value)
        self.collision_distance = float(self.get_parameter("collision_distance").value)

        # Senaryo adini temizle. Savunma amacli, cunku bu bir kez kirildi:
        # launch'tan `scenario` olarak tam dosya yolu geldi
        # (".../worlds/w2_cone_slalom.world") ve `os.path.join` ikinci parca
        # ABSOLUTE oldugu icin `out_dir`'i SESSIZCE yuttu -- CSV paket kurulum
        # dizinine yazildi. Bir dugum, parametre ne gelirse gelsin kendi cikti
        # dizininin disina yazmamali.
        self.scenario = self._sanitize(self.scenario)

        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(out_dir, f"{self.scenario}_{stamp}.csv")
        self._fh = open(self.path, "w", newline="", encoding="utf8")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(HEADER)

        # -- toplanan durum --
        self.x = self.y = self.yaw = 0.0
        self.v_odom = self.w_odom = 0.0
        self.v_cmd = self.w_cmd = 0.0
        self.sectors: Optional[list] = None
        self.state = "?"
        self.t0: Optional[float] = None

        # -- ozet istatistikleri --
        self.min_nearest = math.inf
        self.distance = 0.0
        self.contacts = 0          # temas BASLANGICI sayisi (surekli degil)
        self._in_contact = False
        self.rows = 0
        self.state_ticks: dict[str, int] = {}
        self._last_xy: Optional[tuple] = None

        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_subscription(Float32MultiArray, "mergen/sectors", self._on_sectors, 10)
        self.create_subscription(String, "mergen/state", self._on_state, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)

        self.create_timer(self.period, self._tick)
        self.get_logger().info(f"metrik kaydi: {self.path}")

    @staticmethod
    def _sanitize(scenario: str) -> str:
        """Senaryo adini guvenli bir dosya adi parcasina indirger.

        ".../worlds/w2_cone_slalom.world" -> "w2_cone_slalom"
        """
        name = os.path.basename(str(scenario).strip().rstrip("/"))
        if name.endswith(".world"):
            name = name[: -len(".world")]
        # Dosya adinda ise yaramayacak karakterleri temizle
        safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in name)
        return safe or "unnamed"

    # ------------------------------------------------------------- girdiler
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.v_odom = msg.twist.twist.linear.x
        self.w_odom = msg.twist.twist.angular.z

    def _on_cmd(self, msg: Twist) -> None:
        self.v_cmd, self.w_cmd = msg.linear.x, msg.angular.z

    def _on_sectors(self, msg: Float32MultiArray) -> None:
        self.sectors = list(msg.data)

    def _on_state(self, msg: String) -> None:
        self.state = msg.data.split("|", 1)[0]

    def _on_scan(self, msg: LaserScan) -> None:
        """Temas tespitini DOGRUDAN ham taramadan yapiyoruz.

        Neden planlayicinin raporuna guvenmiyoruz: planlayici `robust_k` ile
        tek isinlari eliyor (dogru karar). Ama OLCUM yaparken eleme yapmak
        istemeyiz -- gercekten surttuysek bunu gormemiz lazim. Olcum ile karar
        ayni filtreyi paylasmamali.
        """
        valid = [r for r in msg.ranges
                 if r is not None and not math.isnan(r) and not math.isinf(r)
                 and msg.range_min <= r <= msg.range_max]
        if not valid:
            return
        nearest = min(valid)
        self.min_nearest = min(self.min_nearest, nearest)
        touching = nearest <= self.collision_distance
        if touching and not self._in_contact:
            self.contacts += 1
            self.get_logger().warn(f"TEMAS #{self.contacts}: en yakin {nearest:.3f} m")
        self._in_contact = touching

    # ----------------------------------------------------------------- dongu
    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        if self._last_xy is not None:
            self.distance += math.dist(self._last_xy, (self.x, self.y))
        self._last_xy = (self.x, self.y)

        s = self.sectors or [float("nan")] * 10
        self._csv.writerow([
            f"{t:.3f}", f"{self.x:.4f}", f"{self.y:.4f}", f"{self.yaw:.4f}",
            f"{self.v_cmd:.4f}", f"{self.w_cmd:.4f}",
            f"{self.v_odom:.4f}", f"{self.w_odom:.4f}",
            *[f"{v:.4f}" for v in s[:7]],
            f"{s[7]:.4f}" if len(s) > 7 else "nan",
            self.state,
        ])
        self.rows += 1
        self.state_ticks[self.state] = self.state_ticks.get(self.state, 0) + 1

    # ---------------------------------------------------------------- ozet
    def finish(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:                     # noqa: BLE001
            pass

        duration = self.rows * self.period
        lines = [
            "",
            "=" * 66,
            f" KOSU OZETI -- senaryo: {self.scenario}",
            "=" * 66,
            f" sure                 : {duration:8.1f} s",
            f" kat edilen yol       : {self.distance:8.2f} m",
            f" ortalama hiz         : {(self.distance / duration if duration else 0):8.2f} m/s",
            f" en yakin yaklasma    : {self.min_nearest:8.2f} m"
            f"   (temas esigi {self.collision_distance:.2f} m)",
            f" temas sayisi         : {self.contacts:8d}"
            f"   {'<-- SARTNAME: surtme -10 / carpma DISKALIFIYE' if self.contacts else '(temiz)'}",
            f" kayit                : {self.path}",
            " durum dagilimi       :",
        ]
        total = max(1, sum(self.state_ticks.values()))
        for st, n in sorted(self.state_ticks.items(), key=lambda kv: -kv[1]):
            lines.append(f"   {st:<12s} {n * self.period:6.1f} s  ({100.0 * n / total:4.1f}%)")
        lines.append("=" * 66)
        self.get_logger().info("\n".join(lines))


def main(args=None) -> None:
    # `finish()` CSV'yi kapatip ozeti basar -- Ctrl-C'de de calismasi sart,
    # yoksa kosunun sonucunu kaybediyoruz.
    run(MetricsLogger, on_stop="finish", args=args)


if __name__ == "__main__":
    main()
