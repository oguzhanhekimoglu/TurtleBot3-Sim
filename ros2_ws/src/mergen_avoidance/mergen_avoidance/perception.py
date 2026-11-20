"""ALGILAMA (Perception): LaserScan -> kullanilabilir engel bilgisi.

Robotaksi karsiligi: aracta Velodyne VLP-16 var ve `sensor_msgs/PointCloud2`
yayinlar. Burada 2B `LaserScan` kullaniyoruz. Gecis yolu:
VLP-16 point cloud -> zemin ayristirma (RANSAC) -> yukseklik dilimi -> 360 kovaya
indirgeme -> ayni `PolarScan` yapisi. Yani bu dosyanin ALTINDAKI her sey aynen kalir.
Bu, kasitli bir tasarim karari: sensor degisimi tek bir dosyayi etkiliyor.

ROS'a bagimlilik yok -> test edilebilir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .geometry import Interval, merge_intervals, normalize_angle


@dataclass(frozen=True)
class ScanGeometry:
    """`sensor_msgs/LaserScan` basligindan gelen sabitler."""

    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass(frozen=True)
class PolarScan:
    """Temizlenmis tarama.

    `angles[i]` govde cercevesinde (-pi, pi], `ranges[i]` metre.
    Sira garantisi YOKTUR (gecersiz isinlar atildigi icin bosluklar olusur).
    """

    angles: Tuple[float, ...]
    ranges: Tuple[float, ...]

    def __len__(self) -> int:
        return len(self.angles)


def clean_scan(
    raw_ranges: Sequence[float],
    geom: ScanGeometry,
    horizon: float,
    mount_yaw: float = 0.0,
) -> PolarScan:
    """Ham `ranges` dizisini temiz bir `PolarScan`a cevirir.

    Kurallar (her biri gercek bir hata kaynagina karsilik gelir):
      * `nan`                 -> ATILIR. Sensor o isini olcemedi, bilgi yok.
      * `inf` / `>= range_max`-> `horizon` olarak YAZILIR. "Donus yok" demek
        "orada bir sey yok" demektir; bunu atmak bosluklari gormemize engel olur.
        (En sik yapilan hata burasi: inf'i atinca robot bos koridoru "duvar" sanir.)
      * `< range_min`         -> ATILIR. Govdenin kendisine carpan isin.
      * geri kalan            -> min(r, horizon).

    `mount_yaw`: lidar govdeye donuk monte edildiyse duzeltme (TurtleBot3'te 0).
    """
    if horizon <= 0.0:
        raise ValueError("horizon > 0 olmali")

    angles: List[float] = []
    ranges: List[float] = []
    for i, r in enumerate(raw_ranges):
        a = normalize_angle(geom.angle_min + i * geom.angle_increment + mount_yaw)
        if r is None:
            continue
        r = float(r)
        if math.isnan(r):
            continue
        if math.isinf(r) or r >= geom.range_max:
            angles.append(a)
            ranges.append(horizon)
            continue
        if r < geom.range_min or r <= 0.0:
            continue
        angles.append(a)
        ranges.append(min(r, horizon))
    return PolarScan(tuple(angles), tuple(ranges))


def cone_distance(
    scan: PolarScan,
    center: float,
    half_width: float,
    horizon: float,
    robust_k: int = 2,
) -> float:
    """`center` yonunde `+-half_width` koni icindeki engel mesafesi.

    `robust_k`: en kucuk k. degeri dondurur. k=1 saf minimumdur ve TEK bir
    hatali isin yuzunden robotu durdurur; k=2..3 tek-isin gurultusunu eler ama
    gercek engeli (onlarca isin dusurur) kacirmaz. Gazebo'da lidar gurultusu
    acikken bu fark onemli.
    """
    if robust_k < 1:
        raise ValueError("robust_k >= 1 olmali")
    picked: List[float] = []
    for a, r in zip(scan.angles, scan.ranges):
        d = a - center
        # koni dar oldugu icin sarma riski yok, yine de guvenli olalim:
        if abs(normalize_angle(d)) <= half_width:
            picked.append(r)
    if not picked:
        return horizon
    picked.sort()
    return picked[min(robust_k, len(picked)) - 1]


def blocked_intervals(
    scan: PolarScan,
    robot_radius: float,
    horizon: float,
    search_limit: float,
) -> List[Interval]:
    """Robotun GECEMEYECEGI yonleri aci araliklari olarak uretir.

    Klasik "konfigurasyon uzayina sisirme" (C-space inflation): `d` mesafesinde
    gorulen bir nokta engel, robotun yaricapi `R` ise, o yondeki
    `asin(R/d)` kadar aciyi da kapatir -- cunku robot bir nokta degil.
    Bunu yapmazsan robot "bosluk var" der, girer ve omzuyla siyirir.
    Sartnamedeki "surterek sakindi -> -10 puan" tam olarak bu hata.

    Sadece `search_limit` icindeki yonlerle ilgileniyoruz (arkayi bosuna isleme).
    """
    if robot_radius <= 0.0:
        raise ValueError("robot_radius > 0 olmali")
    raw: List[Interval] = []
    for a, r in zip(scan.angles, scan.ranges):
        if r >= horizon:
            continue  # bos yon
        if abs(a) > search_limit + math.pi / 2.0:
            continue  # arka taraf, aday rotalari etkilemez
        d = max(r, 1e-3)
        half = math.asin(min(1.0, robot_radius / d))
        raw.append((a - half, a + half))
    return merge_intervals(raw)


def angular_profile(
    scan: PolarScan,
    horizon: float,
    limit: float,
    buckets: int,
    robust_k: int = 2,
) -> Tuple[float, ...]:
    """[-limit, +limit] arasini `buckets` kovaya bolup her kovanin mesafesini dondurur.

    Neden ham `ranges` dizisini degil de kovalari karsilastiriyoruz: `clean_scan`
    gecersiz isinlari ATTIGI icin iki ardisik taramanin isin sayisi ve siralamasi
    ayni olmak zorunda degil -- indeks indeks karsilastirmak hatali olur. Sabit
    sayida kova, iki taramayi AYNI uzayda kiyaslanabilir hale getirir.
    """
    if buckets < 1:
        raise ValueError("buckets >= 1 olmali")
    half = limit / buckets
    return tuple(
        cone_distance(scan, -limit + (2 * i + 1) * half, half, horizon, robust_k)
        for i in range(buckets)
    )


def profile_activity(
    previous: Optional[Tuple[float, ...]],
    current: Tuple[float, ...],
    dt: float,
) -> float:
    """Sahne ne kadar hizli degisiyor? [m/s] -- kova basina ortalama degisim.

    Ne ise yarar: "yol kapali" durumunda iki soruyu ayirt etmek gerekir.
      * Onumu bir DUVAR kapatiyor      -> sahne donmus, yon aramaya basla
      * Onumden biri GECIYOR           -> sahne hareketli, BEKLE

    Kapanma hizi (`closing_rate`) bu ayrimi tek basina yapamaz: tam onunden
    dik gecen bir yaya mesafeyi neredeyse sabit tutar, kapanma ~0 olur ve
    "donmus" gibi gorunur. Profilin TAMAMINDAKI degisim ise yayayi yakalar.

    Bu ayrimi yapmamanin bedeli olculdu: yaya yolu kapatinca arac 180 derece
    donup koridorda ters yone gidiyordu (sartname: ters yon ihlali + beklemek
    yerine kacmak = puan kaybi).
    """
    if previous is None or dt <= 0.0 or len(previous) != len(current):
        return 0.0
    total = sum(abs(a - b) for a, b in zip(previous, current))
    return total / (len(current) * dt)


def closing_rate(
    previous: Optional["PolarScan"],
    current: PolarScan,
    dt: float,
    center: float,
    half_width: float,
    horizon: float,
    robust_k: int = 2,
) -> float:
    """Bir koni icindeki en yakin noktanin YAKLASMA hizi [m/s].

    Pozitif = yaklasiyor, negatif = uzaklasiyor.

        kapanma = (onceki_mesafe - simdiki_mesafe) / dt

    Neden bu kadar onemli: planlayicinin geri kalani her kareyi STATIK goruyor.
    Yanal yaklasan bir yayayi "su an onumde degil" diye gormezden gelir ve
    ancak carpismaya 0.3 m kalinca fark eder (olculdu: carpisma). Iki ardisik
    kareyi karsilastirmak, engelin HIZINI tam bir takip (tracking) altyapisi
    kurmadan tahmin etmenin en ucuz yoludur.

    Sinirlari durust olalim: bu yontem engelleri KIMLIKLENDIRMEZ. Koni icindeki
    en yakin nokta bir kareden digerine farkli bir cisme ait olabilir. Gercek
    cozum nesne takibi (en-yakin-komsu eslestirme + sabit hizli Kalman) ve
    yol kesisim tahminidir; bkz. docs/02-otonom-arac-nasil-yurur.md yol haritasi.
    """
    if previous is None or dt <= 0.0:
        return 0.0
    before = cone_distance(previous, center, half_width, horizon, robust_k)
    now = cone_distance(current, center, half_width, horizon, robust_k)
    # Ikisi de ufukta ise bilgi yok (ikisi de "bos")
    if before >= horizon and now >= horizon:
        return 0.0
    return (before - now) / dt


@dataclass(frozen=True)
class ObstacleReport:
    """Planlayiciya verilen ozet. Node katmani bunu loglar ve RViz'e basar."""

    front: float          # on koni mesafesi (m)
    front_left: float
    front_right: float
    left: float
    right: float
    nearest: float        # tum tarama icindeki en yakin nokta (m)
    nearest_angle: float  # o noktanin acisi (rad)

    @property
    def freer_side_sign(self) -> float:
        """+1 => sol daha bos, -1 => sag daha bos. Kilitlendiginde hangi yana donecegiz."""
        return 1.0 if (self.front_left + self.left) >= (self.front_right + self.right) else -1.0


def summarize(scan: PolarScan, horizon: float, front_half_width: float, robust_k: int = 2) -> ObstacleReport:
    """Insan tarafindan okunabilir sektor ozeti uretir (log / teshis / RViz icin)."""
    q = math.pi / 4.0
    nearest = horizon
    nearest_angle = 0.0
    for a, r in zip(scan.angles, scan.ranges):
        if r < nearest:
            nearest, nearest_angle = r, a
    return ObstacleReport(
        front=cone_distance(scan, 0.0, front_half_width, horizon, robust_k),
        front_left=cone_distance(scan, +q, q / 2.0, horizon, robust_k),
        front_right=cone_distance(scan, -q, q / 2.0, horizon, robust_k),
        left=cone_distance(scan, +math.pi / 2.0, q / 2.0, horizon, robust_k),
        right=cone_distance(scan, -math.pi / 2.0, q / 2.0, horizon, robust_k),
        nearest=nearest,
        nearest_angle=nearest_angle,
    )
