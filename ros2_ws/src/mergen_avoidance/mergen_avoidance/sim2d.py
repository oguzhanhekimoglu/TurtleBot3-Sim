"""Kucuk, bagimsiz 2B kinematik simulator (Gazebo'ya gerek yok).

Ne ise yarar?
  * Algoritmayi Gazebo acmadan saniyeler icinde kosturup dogrulamak.
  * Birim testlerde "bu parkurdan carpmadan gecti mi?" sorusunu sormak.
  * Parametre taramasi (w_clearance=1..4 arasinda hangisi daha iyi?) yapmak.

Gazebo'nun yerini ALMAZ: burada tekerlek kaymasi, sensor gurultusu, 3B govde,
kutle/surtunme yok. Isi, mantik hatasini Gazebo'dan once yakalamak.
Gazebo 60 saniyede acilir; bu 5 milisaniyede kosar. Gelistirme dongusu farki bu.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .params import AvoidParams
from .geometry import normalize_angle
from .perception import PolarScan, ScanGeometry, clean_scan
from .planner import AvoidancePlanner, DriveState


@dataclass(frozen=True)
class Circle:
    """Dairesel engel: trafik konisi, varil, direk, yaya."""

    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Box:
    """Eksene paralel dikdortgen engel: duvar, bariyer, yol blogu.

    `size_x`/`size_y` TAM kenar uzunlugudur (yaricap degil), Gazebo'nun
    `<box><size>` etiketiyle ayni konvansiyon -- boylece dunya ureticisi
    hicbir donusum yapmadan birebir kopyalayabiliyor.
    """

    x: float
    y: float
    size_x: float
    size_y: float

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        hx, hy = 0.5 * self.size_x, 0.5 * self.size_y
        return self.x - hx, self.y - hy, self.x + hx, self.y + hy


Obstacle = object  # Circle | Box  (Python 3.8 uyumu icin birlesim yazmiyoruz)


def _ray_circle(ox: float, oy: float, dx: float, dy: float, c: Circle) -> float:
    """Isin-daire kesisimi. Carpmazsa inf.

    Isin:   P(t) = O + t*d,  t >= 0,  |d| = 1
    Daire:  |P - C|^2 = r^2
    ->      t^2 - 2*t*(d.(C-O)) + |C-O|^2 - r^2 = 0
    """
    px, py = c.x - ox, c.y - oy
    proj = px * dx + py * dy
    if proj <= 0.0:
        return math.inf                       # daire arkada
    perp_sq = px * px + py * py - proj * proj
    gap = c.radius * c.radius - perp_sq
    if gap < 0.0:
        return math.inf                       # isin daireyi kaciriyor
    t = proj - math.sqrt(gap)
    return t if t > 0.0 else math.inf


def _ray_box(ox: float, oy: float, dx: float, dy: float, b: Box) -> float:
    """Isin-AABB kesisimi, klasik "slab" yontemi. Carpmazsa inf.

    Her eksen icin isinin kutu dilimine girdigi/ciktigi t degerlerini bulup
    kesisimlerini aliyoruz. t_near > t_far ise isin kutuyu kaciriyor.
    """
    min_x, min_y, max_x, max_y = b.bounds
    t_near, t_far = -math.inf, math.inf

    for origin, direction, lo, hi in ((ox, dx, min_x, max_x), (oy, dy, min_y, max_y)):
        if abs(direction) < 1e-12:
            # Isin bu eksene paralel: dilimin icinde degilse hic kesismez.
            if origin < lo or origin > hi:
                return math.inf
            continue
        t1 = (lo - origin) / direction
        t2 = (hi - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        t_near = max(t_near, t1)
        t_far = min(t_far, t2)
        if t_near > t_far:
            return math.inf

    if t_far < 0.0:
        return math.inf                       # kutu tamamen arkada
    t = t_near if t_near > 0.0 else t_far     # baslangic kutunun icindeyse t_far
    return t if t > 0.0 else math.inf


def surface_distance(obstacle, px: float, py: float) -> float:
    """(px, py) noktasindan engelin YUZEYINE en kisa mesafe. Icindeyse 0."""
    if isinstance(obstacle, Circle):
        return max(0.0, math.hypot(obstacle.x - px, obstacle.y - py) - obstacle.radius)
    min_x, min_y, max_x, max_y = obstacle.bounds
    dx = max(min_x - px, 0.0, px - max_x)
    dy = max(min_y - py, 0.0, py - max_y)
    return math.hypot(dx, dy)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def raycast(origin: Pose, beam_angle: float, obstacles: Sequence[Obstacle], max_range: float) -> float:
    """Tek bir lidar isininin ilk carptigi mesafe. Carpmazsa `inf`."""
    world_angle = origin.yaw + beam_angle
    dx, dy = math.cos(world_angle), math.sin(world_angle)

    best = math.inf
    for obstacle in obstacles:
        if isinstance(obstacle, Circle):
            t = _ray_circle(origin.x, origin.y, dx, dy, obstacle)
        else:
            t = _ray_box(origin.x, origin.y, dx, dy, obstacle)
        if t < best:
            best = t
    return best if best <= max_range else math.inf


def simulate_scan(
    pose: Pose,
    obstacles: Sequence[Obstacle],
    n_beams: int = 360,
    range_max: float = 3.5,
    range_min: float = 0.12,
) -> Tuple[List[float], ScanGeometry]:
    """TurtleBot3 LDS-01 ile ayni duzende (360 isin, 0'dan baslayan) tarama uretir."""
    geom = ScanGeometry(
        angle_min=0.0,
        angle_increment=2.0 * math.pi / n_beams,
        range_min=range_min,
        range_max=range_max,
    )
    ranges: List[float] = []
    for i in range(n_beams):
        a = geom.angle_min + i * geom.angle_increment
        d = raycast(pose, a, obstacles, range_max)
        ranges.append(d if math.isinf(d) else max(d, range_min))
    return ranges, geom


@dataclass
class SimResult:
    """Bir kosunun ozeti. Testler ve rapor grafikleri bunu kullanir."""

    collided: bool
    min_clearance: float
    distance_travelled: float
    duration: float
    reached_goal: bool
    trajectory: List[Tuple[float, float]] = field(default_factory=list)
    states: List[str] = field(default_factory=list)
    speeds: List[float] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"carpisma={'VAR' if self.collided else 'yok'}  "
            f"en_yakin={self.min_clearance:.2f} m  "
            f"yol={self.distance_travelled:.2f} m  "
            f"sure={self.duration:.1f} s  "
            f"hedef={'ULASILDI' if self.reached_goal else 'ulasilamadi'}"
        )


def run_episode(
    obstacles: Sequence[Obstacle],
    params: Optional[AvoidParams] = None,
    start: Optional[Pose] = None,
    goal_x: float = 8.0,
    max_time: float = 90.0,
    body_radius: float = 0.22,
    world_goal_heading: Optional[float] = 0.0,
    dynamic: Optional[Callable[[float], Sequence[Obstacle]]] = None,
) -> SimResult:
    """Robotu `goal_x`'e ulasana ya da sure bitene kadar surer.

    Hareket modeli: birim-cember (unicycle) ileri Euler entegrasyonu.
        x += v*cos(yaw)*dt ;  y += v*sin(yaw)*dt ;  yaw += w*dt
    Gazebo'nun diff_drive eklentisi de buna cok yakin davranir.
    """
    p = params or AvoidParams()
    planner = AvoidancePlanner(p)
    pose = start or Pose(0.0, 0.0, 0.0)
    dt = p.control_period

    result = SimResult(False, math.inf, 0.0, 0.0, False)
    v = w = 0.0
    t = 0.0
    while t < max_time:
        # Hareketli engeller zamanin fonksiyonu olarak her adimda yeniden uretilir.
        active = list(obstacles) + list(dynamic(t)) if dynamic else obstacles
        ranges, geom = simulate_scan(pose, active, range_max=max(3.5, p.horizon))
        scan = clean_scan(ranges, geom, p.horizon)
        # Hedef yonu DUNYA cercevesinde tutup her adimda govdeye ceviriyoruz.
        # `None` verilirse planlayici "burnunun dogrultusuna git" moduna duser.
        goal_body = (None if world_goal_heading is None
                     else normalize_angle(world_goal_heading - pose.yaw))
        decision = planner.step(scan, measured_linear=v, measured_angular=w,
                                dt=dt, goal_heading=goal_body)
        v, w = decision.linear, decision.angular

        pose.x += v * math.cos(pose.yaw) * dt
        pose.y += v * math.sin(pose.yaw) * dt
        pose.yaw += w * dt

        result.distance_travelled += abs(v) * dt
        result.trajectory.append((pose.x, pose.y))
        result.states.append(str(decision.state))
        result.speeds.append(v)

        # govde merkezinden engel yuzeyine gercek mesafe
        for obstacle in active:
            d = surface_distance(obstacle, pose.x, pose.y)
            if d < result.min_clearance:
                result.min_clearance = d
            if d <= body_radius:
                result.collided = True

        t += dt
        if pose.x >= goal_x:
            result.reached_goal = True
            break

    result.duration = t
    if math.isinf(result.min_clearance):
        result.min_clearance = float("nan")
    return result


# ---------------------------------------------------------------------------
# Hazir senaryolar -- Gazebo dunyalariyla AYNI yerlesimler (worlds/*.world)
# ---------------------------------------------------------------------------
def corridor_walls(length: float = 14.0, half_width: float = 1.8,
                   thickness: float = 0.2) -> List[Obstacle]:
    """Iki yanda uzun duvar -> tek seritli yol. Robotaksi'de "serit" karsiligi."""
    centre_x = 0.5 * length - 2.0            # robot x=0'da baslar, duvar -2'den baslasin
    return [
        Box(centre_x, +half_width, length, thickness),
        Box(centre_x, -half_width, length, thickness),
    ]


def scenario(name: str) -> List[Obstacle]:
    """Isimle engel yerlesimi dondurur.

    Bu fonksiyon TEK GERCEK KAYNAK (single source of truth):
    `scripts/generate_worlds.py` Gazebo `.world` dosyalarini buradan uretir.
    Boylece 2B simulatorde test ettigin yerlesim ile Gazebo'da gordugun
    yerlesim birbirinden AYRILAMAZ. Iki yerde elle tanimlanan parkurlar
    kacinilmaz olarak birbirinden uzaklasir ve "2B'de gecti, Gazebo'da
    carpti" durumuna yol acar.
    """
    if name == "w1_empty":
        return []

    if name == "w2_cone_slalom":
        # Slalom: art arda yon degistirmeye zorlar, histerezisi sinar.
        return [Circle(2.0, 0.0, 0.18), Circle(4.0, 0.9, 0.18),
                Circle(6.0, -0.9, 0.18), Circle(8.0, 0.4, 0.18)]

    if name == "w3_corridor":
        # Koridor + donusumlu variller: serit disiplini testi.
        return corridor_walls() + [Circle(3.0, -0.6, 0.30), Circle(6.0, 0.7, 0.30)]

    if name == "w4_lane_block":
        # STATIK ENGEL (sartname): seridi KISMEN kapatan blok.
        # Arac sakinmali ama koridordan cikmamali.
        return corridor_walls() + [Box(5.0, -1.0, 0.8, 1.4)]

    if name == "w5_dead_end":
        # Kisa cikmaz koridor: ici kapali, DISI acik -> etrafindan dolasilabilir.
        return [
            Box(3.25, +1.3, 2.5, 0.2),
            Box(3.25, -1.3, 2.5, 0.2),
            Box(4.6, 0.0, 0.2, 2.8),
        ]

    if name == "w6_dense_clutter":
        # Yogun dagilmis koniler: surekli yeniden planlama.
        pts = [(2.2, 0.3), (3.1, -0.8), (3.9, 0.9), (4.6, -0.2), (5.4, 0.7),
               (6.1, -0.9), (6.9, 0.2), (7.6, -0.7)]
        return [Circle(x, y, 0.22) for x, y in pts]

    if name == "w7_trap_pocket":
        # GERCEK kapan: iki yani + sonu kapali cep.
        # Amaci algoritmayi kirmak degil, SINIRINI gostermek: salt tepkisel
        # (reactive) bir planlayici yerel minimumdan kendi basina cikamaz.
        # Cozum global planlayicidir (Nav2 / GEOJSON rotasi).
        return [
            Box(3.0, +1.1, 7.0, 0.2),
            Box(3.0, -1.1, 7.0, 0.2),
            Box(6.6, 0.0, 0.2, 2.4),
        ]

    if name == "w9_parkur":
        # DOLU ve SONU OLAN parkur.
        # w2_cone_slalom'un iki eksigi vardi: sadece 4 engel, ve sonrasi
        # bos oldugu icin robot sonsuza kadar ilerliyordu (hedef yon sabit
        # +x). Burada 20 m'lik bir koridor, 10 engel ve SONDA bir duvar var:
        # arac butun engelleri gecip duvara varinca durup geri aliyor.
        # Bitis duvari YOK -- bilerek.
        # Once koridoru duvarla kapatmistim ki arac sonsuza gitmesin. Sonuc
        # kotuydu: duvar tum koridoru kapattigi icin arac onunde kilitlenip
        # salindi (BLOCKED %38). Cunku bu bir CIKMAZ SOKAK ve salt tepkisel
        # planlayici cikmaz sokagi cozemez.
        # Dogru cozum duvar degil HEDEF: `goal_x` parametresi. Arac gorev
        # noktasina varinca duruyor -- yarismadaki gorev noktasi mantigi.
        duvarlar = [
            Box(9.0, +2.0, 24.0, 0.2),     # sol duvar
            Box(9.0, -2.0, 24.0, 0.2),     # sag duvar
        ]
        # Engeller MERKEZE yakin: robotun yolunu gercekten kapatmali.
        # Ilk denemede engelleri kenara koymustum; robot neredeyse duz gidip
        # gecti (maks yan sapma 0.12 m) -- yani parkur hicbir sey test
        # etmiyordu. Engel "var" olmasi yetmez, YOLU KAPATMASI gerekir.
        engeller = [
            Circle(2.5, 0.0, 0.25),        # tam ortada: iki yandan birini sec
            Circle(4.5, +0.8, 0.25),
            Circle(6.5, -0.8, 0.25),
            Box(8.5, +1.3, 0.8, 1.2),      # soldan kismi blok
            Circle(10.0, -0.5, 0.25),
            Circle(11.8, +0.6, 0.25),
            Box(13.5, -1.3, 0.8, 1.2),     # sagdan kismi blok
            # Gecit: aciklik 2.2 m.
            # Olculerek bulundu. 1.2 m -> arac kilitlendi; 1.8 m -> hemen
            # onundeki koniyle birlesince yine kilitlendi. Yakin mesafede
            # C-space sisirmesi aci penceresini hizla kapatir, bu yuzden
            # gecit ile ondeki engel arasinda da BOSLUK birakmak gerekir.
            Box(16.5, +1.55, 0.3, 0.9),
            Box(16.5, -1.55, 0.3, 0.9),
        ]
        return duvarlar + engeller

    if name == "w8_pedestrian":
        # Dinamik engel senaryosunun STATIK iskeleti. Yayanin kendisi
        # `pedestrian_crossing()` ile zamana bagli olarak eklenir.
        return corridor_walls()

    raise ValueError(f"bilinmeyen senaryo: {name}")


def pedestrian_crossing(trigger_x: float = 4.0, speed: float = 0.45,
                        start_y: float = -2.0, radius: float = 0.25,
                        start_time: float = 14.0) -> Callable[[float], List[Obstacle]]:
    """Belirli bir anda karsiya gecmeye baslayan yaya uretir (2B simulator icin).

    Gazebo tarafindaki karsiligi `pedestrian_node.py` + `w8_pedestrian.world`.
    Burada zamanla tetikliyoruz (robotun konumuna degil) cunku 2B kosu
    TEKRARLANABILIR olmali: ayni parametrelerle ayni sonucu vermesi lazim.
    """
    def obstacles_at(t: float) -> List[Obstacle]:
        if t < start_time:
            return [Circle(trigger_x, start_y, radius)]      # kaldirimda bekliyor
        y = start_y + speed * (t - start_time)
        return [Circle(trigger_x, min(y, -start_y), radius)]  # karsiya geciyor
    return obstacles_at


SCENARIOS = ["w1_empty", "w2_cone_slalom", "w3_corridor", "w4_lane_block",
             "w5_dead_end", "w6_dense_clutter", "w7_trap_pocket", "w8_pedestrian",
             "w9_parkur"]
