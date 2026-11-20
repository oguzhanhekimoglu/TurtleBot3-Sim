"""Tum ayarlanabilir sayilar TEK yerde.

Kural: kodun icinde cikplak sayi (magic number) yok. Her esik burada, adiyla ve
birimiyle duruyor. Sebebi pratik: yarisma gunu parkurda parametre denemesi
yapacaksan, 8 dosyayi degil tek YAML'i degistirmen lazim.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields


@dataclass
class AvoidParams:
    # ---- Arac govdesi -------------------------------------------------------
    robot_radius: float = 0.26
    """Govdeyi cevreleyen daire yaricapi [m]. TurtleBot3 Waffle ~0.22 m;
    guvenlik payiyla 0.26 aliyoruz. Robotaksi'de bu ~1.0 m olur (en ~1.5 m)."""

    # ---- Lidar kullanimi ----------------------------------------------------
    horizon: float = 3.0
    """Bu mesafeden uzagi 'bos' kabul et [m]. Lidar 3.5 m goruyor; 3.0 m
    karar ufku, hem gurultulu uzak isinlari eler hem hesabi azaltir."""

    front_half_width: float = 0.35
    """On koninin yari genisligi [rad] (~20 derece). Fren karari bu koniye bakar."""

    clearance_half_width: float = 0.26
    """Aday bir rotanin 'ne kadar acik' oldugunu olcerken kullanilan koni [rad]."""

    robust_k: int = 2
    """`cone_distance` icin k. en kucuk. Tek-isin gurultusune karsi."""

    # ---- Mesafe esikleri ----------------------------------------------------
    emergency_distance: float = 0.25
    """Bu mesafenin altinda KOSULSUZ dur [m]. Son guvenlik agi."""

    stop_distance: float = 0.45
    """On mesafe bunun altina inerse ilerlemeyi kes, yerinde yon ara [m]."""

    slow_distance: float = 1.40
    """On mesafe bunun altina inerse hiz lineer olarak dusmeye baslar [m]."""

    avoid_trigger_distance: float = 2.00
    """Bu mesafede engel gorulurse AVOID durumuna gec (erken, yumusak manevra) [m]."""

    # ---- Hiz / donus limitleri ---------------------------------------------
    max_linear_speed: float = 0.22
    """[m/s]. TurtleBot3 Waffle siniri 0.26 m/s."""

    min_linear_speed: float = 0.05
    """AVOID sirasinda tamamen durmamak icin taban hiz [m/s]."""

    max_angular_speed: float = 1.30
    """[rad/s]. Waffle siniri 1.82 rad/s."""

    max_linear_accel: float = 0.35
    """Hiz degisim limiti [m/s^2]. Sartnamedeki 'ergonomi' / yolcu konforu.
    Bunu koymazsan arac her karede zipliyormus gibi davranir."""

    max_angular_accel: float = 2.50
    """[rad/s^2]."""

    # ---- Rota secimi (VFH benzeri maliyet) ---------------------------------
    max_steer_angle: float = 1.57
    """Aday rotalari +-bu aci icinde ara [rad]. 90 derece."""

    steer_resolution: float = 0.035
    """Aday rota tarama adimi [rad] (~2 derece)."""

    w_goal: float = 1.00
    """Hedef yonunden sapma cezasi."""

    w_clearance: float = 2.20
    """Rotanin kapaliligi cezasi. Buyurse robot daha korkak/daha genis kacar."""

    w_smooth: float = 0.70
    """Onceki karardan sapma cezasi = HISTEREZIS. Bunu 0 yaparsan robot iki
    bosluk arasinda saniyede 10 kez karar degistirir ve yerinde titrer."""

    heading_gain: float = 1.60
    """Secilen yon hatasini acisal hiza ceviren P kazanci [1/s]."""

    turn_speed_penalty: float = 0.80
    """Donus ne kadar keskinse ileri hizi o kadar kes (0..1)."""

    # ---- Kilitlenme / kurtarma ---------------------------------------------
    stuck_speed_threshold: float = 0.03
    """Olculen hiz bunun altindaysa 'ilerlemiyor' sayilir [m/s]."""

    stuck_angular_threshold: float = 0.15
    """Olculen acisal hiz bunun altindaysa 'donmuyor' sayilir [rad/s].
    'Sikisti' karari icin hem ilerleme hem donus olmamali: BLOCKED durumunda
    robot yerinde donerek yol ariyor olabilir, bu uretken bir davranistir ve
    kurtarma manevrasi ile kesilmemelidir."""

    stuck_time: float = 3.0
    """Bu kadar sure ilerlemezse RECOVER'a gec [s]."""

    profile_buckets: int = 24
    """Sahne degisimini olcmek icin kullanilan aci kovasi sayisi."""

    activity_threshold: float = 0.10
    """Kova basina ortalama degisim bunun uzerindeyse sahne HAREKETLI [m/s].
    Hareketli sahnede "yol kapali" demek "biri geciyor" demektir -> beklenir."""

    blocked_wait: float = 2.0
    """Yol kapaliyken, yon aramaya baslamadan once DURUP BEKLENECEK sure [s].

    Insan surucu de boyle yapar: once durur, bir an bekler, ancak yol gercekten
    kapaliysa manevraya baslar. Sahne hareketli oldugu surece bu sayac
    islemez -- yani gecen yaya bitene kadar beklemeye devam eder."""

    blocked_timeout: float = 4.0
    """BLOCKED durumunda bu kadar sure cozum bulunamazsa RECOVER'a tirman [s].
    Yerinde donmek ise yaramiyorsa geri almak gerekir; yoksa arac saatlerce
    ayni noktada yol arar."""

    rear_safe_distance: float = 0.40
    """Geri alirken arkada bu kadar bos olmali [m].

    Planlayici yalnizca ONE bakar (+-max_steer_angle). RECOVER durumunda ise
    arac GERI gider -- yani bakmadigi yone. Bu bir kez carpismaya yol acti:
    dar bir parkurda kilitlenen arac geri alirken arkasindaki koniye vurdu
    (olculdu: t=83 s, RECOVER, v=-0.08). Yarisma kurallarinda engele carpmak
    DISKALIFIYE sebebi; "geri giderken oldu" diye bir mazeret yok.

    Arka bu mesafeden yakinsa geri alinmaz; arac yerinde donmeye calisir."""

    recover_duration: float = 1.8
    """RECOVER'da geri + don suresi [s]."""

    recover_linear_speed: float = -0.08
    """RECOVER'da geri hiz [m/s] (negatif)."""

    recover_angular_speed: float = 0.9
    """RECOVER'da donus hizi [rad/s] (isaret bos tarafa gore secilir)."""

    # ---- Dinamik engel (kapanma hizi / TTC) --------------------------------
    wide_arc_half_width: float = 1.31
    """Dinamik engel gozetleme arkinin yari genisligi [rad] (~75 derece).

    Neden on koniden (0.35 rad) GENIS: yanal yaklasan bir yaya, on koniye
    girdigi anda zaten cok yakindir. Yolun kenarindaki hareketi onceden
    gormek icin genis ark gerekir -- gercek surucunun yaptigi da bu."""

    closure_min: float = 0.12
    """Bu degerin altindaki kapanma hizlari gurultu kabul edilir [m/s].
    Esik koymazsan lidar gurultusu surekli yanlis fren uretir."""

    ttc_warn: float = 3.0
    """Carpismaya kalan sure bunun altina inerse hiz kesmeye basla [s]."""

    ttc_brake: float = 1.5
    """Carpismaya kalan sure bunun altina inerse TAM fren [s]."""

    # ---- Ani fren (ego hiza gore) ------------------------------------------
    reaction_time: float = 0.45
    """Carpismaya kalan sure hesabinda kullanilan tepki suresi [s].
    2023 sartnamesindeki 'Ani Fren Gorevi'nin karsiligi."""

    brake_margin: float = 0.15
    """TTC freninde birakilacak ek pay [m]."""

    # ---- Arac kinematigi ----------------------------------------------------
    bicycle_mode: bool = False
    """True ise yerinde donmeyi yasakla ve |w| <= v / min_turn_radius zorla.
    TurtleBot3 diferansiyel surus -> yerinde donebilir. Robotaksi Ackermann ->
    DONEMEZ. Algoritmayi gercek araca tasimadan once bunu True yapip test et;
    yoksa simulasyonda calisan kod gercek araca hic oturmaz."""

    min_turn_radius: float = 4.0
    """Ackermann donus yaricapi [m]. Sadece bicycle_mode=True iken kullanilir."""

    # ---- Hedef --------------------------------------------------------------
    goal_heading: float = 0.0
    """Engel yoksa gidilecek yon [rad]. Gercek sistemde bunu global planlayici
    (GEOJSON noktalarindan uretilen rota) doldurur; burada sabit 'duz git'."""

    control_period: float = 0.05
    """Kontrol dongusu periyodu [s] -> 20 Hz."""

    def validate(self) -> None:
        """Mantiksiz parametre kombinasyonlarini erken yakala."""
        if not (0 < self.emergency_distance < self.stop_distance < self.slow_distance
                <= self.avoid_trigger_distance <= self.horizon):
            raise ValueError(
                "Mesafe esikleri sirali olmali: 0 < emergency < stop < slow "
                f"<= avoid_trigger <= horizon (su an: {self.emergency_distance}, "
                f"{self.stop_distance}, {self.slow_distance}, "
                f"{self.avoid_trigger_distance}, {self.horizon})"
            )
        if self.min_linear_speed >= self.max_linear_speed:
            raise ValueError("min_linear_speed < max_linear_speed olmali")
        if not 0.0 < self.max_steer_angle <= math.pi:
            raise ValueError("0 < max_steer_angle <= pi olmali")
        if self.steer_resolution <= 0.0:
            raise ValueError("steer_resolution > 0 olmali")
        if self.robot_radius <= 0.0 or self.horizon <= 0.0:
            raise ValueError("robot_radius ve horizon > 0 olmali")
        if self.bicycle_mode and self.min_turn_radius <= 0.0:
            raise ValueError("bicycle_mode icin min_turn_radius > 0 olmali")
        if not 0.0 <= self.turn_speed_penalty <= 1.0:
            raise ValueError("turn_speed_penalty 0..1 arasinda olmali")

    @classmethod
    def from_dict(cls, d: dict) -> "AvoidParams":
        """Bilinmeyen anahtarlari sessizce yutmaz -- yazim hatasini yakalar."""
        known = {f.name for f in fields(cls)}
        unknown = set(d) - known
        if unknown:
            raise ValueError(f"Bilinmeyen parametre(ler): {sorted(unknown)}")
        return cls(**d)

    def to_dict(self) -> dict:
        return asdict(self)
