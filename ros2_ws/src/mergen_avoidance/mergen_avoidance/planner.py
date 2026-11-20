"""PLANLAMA + KARAR VERME: nereye gidecegiz ve hangi moddayiz?

Yaklasim: VFH (Vector Field Histogram) ailesinden, sadelestirilmis bir
"aday rota + maliyet fonksiyonu" semasi.

  1. Lidar'dan robotun GECEMEYECEGI aci araliklarini cikar (C-space sisirme).
  2. +-max_steer_angle icinde, steer_resolution adimiyla ADAY YONLER uret.
  3. Her aday yonu uc terimle puanla:  hedefe yakinlik + acikli + yumusaklik.
  4. En ucuz adayi sec.
  5. Bir DURUM MAKINESI ile bu secimi davranisa cevir.

Neden "potansiyel alan" (potential field) degil? Klasik potansiyel alan yerel
minimumda (or. U seklinde cikmaz sokak) kilitlenir ve dar koridorda titrer.
Aday-rota + histerezis semasi her iki problemi de dogrudan ele aliyor:
kilitlenmeyi RECOVER durumu, titremeyi `w_smooth` terimi cozer.

ROS'a bagimlilik yok -> test edilebilir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from .control import (
    dynamic_speed_factor, emergency_brake_required, emergency_command, smooth_command,
    speed_profile, time_to_collision,
)
from .geometry import clamp, free_intervals, in_intervals, normalize_angle
from .params import AvoidParams
from .perception import (
    ObstacleReport, PolarScan, angular_profile, blocked_intervals, closing_rate,
    cone_distance, profile_activity, summarize,
)


class DriveState(Enum):
    """Aracin davranis modlari. Simulasyon raporundaki 'Durum Makinesi' bolumu."""

    CRUISE = "CRUISE"          # yol acik, hedef yonde tam hiz
    SLOW = "SLOW"              # engel yaklasiyor, hiz dusuyor, yon korunuyor
    AVOID = "AVOID"            # sakinma manevrasi, bosluga yoneliyor
    BLOCKED = "BLOCKED"        # gecilebilir yon yok, yerinde yon ariyor
    EMERGENCY = "EMERGENCY"    # ani fren
    RECOVER = "RECOVER"        # kilitlendi, geri al + don

    def __str__(self) -> str:  # log ciktisi icin
        return self.value


@dataclass
class Candidate:
    """Puanlanmis bir aday yon. RViz'de ve logda bunu gosteriyoruz."""

    heading: float
    clearance: float
    cost: float
    feasible: bool


@dataclass
class ControllerState:
    """Karelere YAYILAN hafiza. Saf fonksiyonlar disinda tutulan tek degisken durum."""

    state: DriveState = DriveState.CRUISE
    prev_heading: float = 0.0
    prev_linear: float = 0.0
    prev_angular: float = 0.0
    stuck_timer: float = 0.0
    recover_timer: float = 0.0
    recover_sign: float = 1.0
    elapsed: float = 0.0
    blocked_sign: float = 0.0
    """BLOCKED durumunda KILITLENMIS donus yonu (+1 sol / -1 sag, 0 = henuz yok).

    Neden kilitliyoruz: dar ve simetrik bir koridorda "hangi taraf daha bos"
    sorusunun cevabi her karede taraf degistirir. Yonu her karede yeniden
    secerseniz arac saga-sola birkac derece salinir, net donus sifir olur ve
    cikmaz sokaktan ASLA kurtulamaz (olculdu: -19 ile +2 derece arasinda
    sonsuz limit cevrimi). Karar bir kez verilir ve BLOCKED'dan cikana kadar
    korunur."""

    blocked_timer: float = 0.0
    """BLOCKED durumunda gecen toplam sure [s]. `blocked_timeout` asilirsa RECOVER."""

    blocked_static_timer: float = 0.0
    """BLOCKED durumunda sahnenin DONMUS gectigi sure [s].
    Hareketli sahnede sifirlanir -> gecen yaya bitene kadar beklenir."""


@dataclass
class Decision:
    """Bir kontrol adiminin TUM cikti ve gerekcesi. Loglanabilir, test edilebilir."""

    linear: float
    angular: float
    state: DriveState
    chosen_heading: float
    report: ObstacleReport
    reason: str
    closing_speed: float = 0.0
    """Genis on arkta olculen yaklasma hizi [m/s]. Pozitif = bir sey uzerimize geliyor."""
    dynamic_factor: float = 1.0
    """Dinamik engel yuzunden uygulanan hiz carpani (1 = serbest, 0 = tam fren)."""
    candidates: List[Candidate] = field(default_factory=list)
    blocked: List[Tuple[float, float]] = field(default_factory=list)


class AvoidancePlanner:
    """Durumsal planlayici. Her kontrol adiminda `step()` cagrilir.

    Kasitli olarak ROS'tan habersiz: girdisi `PolarScan` + olculen hizlar,
    cikti `Decision`. Boylece ayni sinifi (a) Gazebo'da, (b) gercek araca bagli,
    (c) kayitli bir rosbag uzerinde, (d) birim testte aynen kullanabiliyoruz.
    """

    def __init__(self, params: Optional[AvoidParams] = None) -> None:
        self.p = params or AvoidParams()
        self.p.validate()
        self.s = ControllerState()
        # O anki hedef yon, GOVDE cercevesinde. Her `step()` basinda guncellenir.
        self._goal: float = self.p.goal_heading
        # Onceki tarama: kapanma hizini (dolayisiyla dinamik engelleri) buradan cikariyoruz.
        self._prev_scan: Optional[PolarScan] = None
        # Onceki acisal profil: sahne hareketliligini buradan cikariyoruz.
        self._prev_profile: Optional[Tuple[float, ...]] = None

    # -- yardimcilar ------------------------------------------------------
    def _candidates(self, scan: PolarScan) -> Tuple[List[Candidate], List[Tuple[float, float]]]:
        """Aday yonleri uret ve puanla."""
        p = self.p
        blocked = blocked_intervals(scan, p.robot_radius, p.horizon, p.max_steer_angle)

        n_steps = int(round(2.0 * p.max_steer_angle / p.steer_resolution))
        out: List[Candidate] = []
        for i in range(n_steps + 1):
            heading = -p.max_steer_angle + i * p.steer_resolution
            heading = clamp(heading, -p.max_steer_angle, p.max_steer_angle)

            clearance = cone_distance(scan, heading, p.clearance_half_width, p.horizon, p.robust_k)

            geometrically_free = not in_intervals(heading, blocked)
            deep_enough = clearance >= p.stop_distance
            feasible = geometrically_free and deep_enough

            # --- maliyet terimleri (hepsi 0..1 civarinda normalize) ---
            goal_term = abs(normalize_angle(heading - self._goal)) / p.max_steer_angle
            clearance_term = 1.0 - clamp(clearance / p.horizon, 0.0, 1.0)
            smooth_term = abs(normalize_angle(heading - self.s.prev_heading)) / (2.0 * p.max_steer_angle)

            cost = (
                p.w_goal * goal_term
                + p.w_clearance * clearance_term
                + p.w_smooth * smooth_term
            )
            out.append(Candidate(heading, clearance, cost, feasible))
        return out, blocked

    def _widest_free_heading(self, blocked: List[Tuple[float, float]]) -> Optional[float]:
        """En GENIS bos aci araliginin ortasi. Aday yonlerin hepsi elendiginde
        son bir sans: belki cok dar ama geometrik olarak acik bir bosluk var."""
        p = self.p
        gaps = free_intervals(blocked, -p.max_steer_angle, p.max_steer_angle)
        if not gaps:
            return None
        lo, hi = max(gaps, key=lambda g: g[1] - g[0])
        return 0.5 * (lo + hi)

    def _update_stuck(self, measured_linear: float, measured_angular: float,
                      dt: float, dynamic_hold: bool) -> None:
        """Ne ilerliyor ne donuyorsa kilitlenme sayacini isletir.

        `dynamic_hold`: yaklasan bir dinamik engel yuzunden BILEREK bekliyoruz.
        Bu durumda sayaci isletmeyiz -- yoksa yaya geciti onunde kurallara uygun
        bekleyen arac 3 saniye sonra "sikistim" diyip GERI GIDER. Beklemek bir
        ariza degil, dogru davranistir (sartname: "engel tamamen geçene kadar
        bekledi -> +50 puan").

        Sonsuz bekleme riski yok: kapanma hizi ancak mesafe azaliyorsa pozitif
        kalir; engel gecip gittiginde isaret dondugu icin durum kendiliginden cozulur.
        """
        p = self.p
        if dynamic_hold:
            return
        idle = (
            abs(measured_linear) < p.stuck_speed_threshold
            and abs(measured_angular) < p.stuck_angular_threshold
        )
        if idle and self.s.state is not DriveState.RECOVER:
            self.s.stuck_timer += dt
        elif not idle:
            self.s.stuck_timer = 0.0

    def _recover_command(self, rear_clearance: float, dt: float) -> Tuple[float, float, str]:
        """Kurtarma komutu -- ARKAYI KONTROL EDEREK.

        Arka kapaliysa geri gitmeyiz. Diferansiyel suruste yerinde donmeye
        calisiriz (donus arkayi acabilir); Ackermann'da yerinde donus de
        mumkun olmadigi icin tek secenek durmaktir.
        """
        p, s = self.p, self.s
        if rear_clearance <= p.rear_safe_distance:
            if p.bicycle_mode:
                linear, angular = smooth_command(0.0, 0.0, s.prev_linear,
                                                 s.prev_angular, dt, p)
                return linear, angular, f"arka kapali ({rear_clearance:.2f} m) -> DUR"
            linear, angular = smooth_command(
                0.0, s.recover_sign * p.recover_angular_speed,
                s.prev_linear, s.prev_angular, dt, p)
            return linear, angular, f"arka kapali ({rear_clearance:.2f} m) -> yerinde don"

        linear, angular = smooth_command(
            p.recover_linear_speed, s.recover_sign * p.recover_angular_speed,
            s.prev_linear, s.prev_angular, dt, p)
        return linear, angular, f"geri al (arka {rear_clearance:.2f} m acik)"

    # -- ana dongu --------------------------------------------------------
    def step(
        self,
        scan: PolarScan,
        measured_linear: float = 0.0,
        measured_angular: float = 0.0,
        dt: Optional[float] = None,
        goal_heading: Optional[float] = None,
    ) -> Decision:
        """Bir kontrol adimi.

        `goal_heading`: gidilmek istenen yon, GOVDE cercevesinde [rad].
        Bunu cagiran taraf hesaplar:
            goal_heading = normalize_angle(dunya_hedef_yonu - aracin_yaw'i)
        Neden govde cercevesi? Cunku planlayici lidar'i govde cercevesinde
        goruyor; iki farkli cerceveyi ayni fonksiyonda karistirmak en sik
        gorulen isaret/aci hatasinin kaynagi.

        `None` verilirse `params.goal_heading` kullanilir -- yani "basini
        cevirdigin yone dogru git". Bu, kendi basina GLOBAL YON BILGISI
        OLMADIGI anlamina gelir: robot bir engelden kactiktan sonra eski
        yonune donmeyi bilmez, kactigi yone devam eder. Gercek sistemde bu
        deger GEOJSON rotasindan uretilen global planlayicidan gelir.
        """
        p = self.p
        s = self.s
        dt = p.control_period if dt is None else dt
        if dt <= 0.0:
            dt = p.control_period
        s.elapsed += dt
        self._goal = p.goal_heading if goal_heading is None else normalize_angle(goal_heading)

        report = summarize(scan, p.horizon, p.front_half_width, p.robust_k)

        # --- Dinamik engel degerlendirmesi (genis on ark) ---
        wide_distance = cone_distance(scan, 0.0, p.wide_arc_half_width, p.horizon, p.robust_k)
        closure = closing_rate(self._prev_scan, scan, dt, 0.0,
                               p.wide_arc_half_width, p.horizon, p.robust_k)
        dyn_factor = dynamic_speed_factor(wide_distance, closure, p)
        dynamic_hold = dyn_factor <= 0.0

        # Sahne ne kadar degisiyor? "Yol kapali" durumunda bekle-mi/ara-mi
        # kararini bu belirliyor.
        profile = angular_profile(scan, p.horizon, p.max_steer_angle,
                                  p.profile_buckets, p.robust_k)
        activity = profile_activity(self._prev_profile, profile, dt)
        scene_moving = activity > p.activity_threshold

        # ARKA ACIKLIK. Planlayici one bakar; RECOVER ise geri gider.
        # Bakmadigin yone gitmek, carpmanin en kisa yoludur.
        rear_clearance = cone_distance(scan, math.pi, p.front_half_width,
                                       p.horizon, p.robust_k)

        self._prev_scan = scan
        self._prev_profile = profile

        self._update_stuck(measured_linear, measured_angular, dt, dynamic_hold)

        # ---------------------------------------------------------------
        # 1) RECOVER zaten suruyorsa onu bitir. Kurtarma manevrasi
        #    BOLUNMEZ: yarim geri alinmis bir manevra, robotu girdigi
        #    kose icinde birakir ve sonsuz dongu uretir.
        # ---------------------------------------------------------------
        if s.state is DriveState.RECOVER and s.recover_timer > 0.0:
            s.recover_timer -= dt
            linear, angular, neden = self._recover_command(rear_clearance, dt)
            if s.recover_timer <= 0.0:
                s.state = DriveState.CRUISE
                s.stuck_timer = 0.0
            return self._finish(linear, angular, DriveState.RECOVER, s.prev_heading, report,
                                f"kurtarma: {neden} ({s.recover_timer:.1f}s kaldi)", [], [])

        # ---------------------------------------------------------------
        # 2) ACIL FREN -- her seyin onunde. Mutlak esik veya TTC.
        # ---------------------------------------------------------------
        if emergency_brake_required(report.front, abs(measured_linear), p):
            linear, angular = emergency_command(s.prev_linear, s.prev_angular, dt, p)
            return self._finish(linear, angular, DriveState.EMERGENCY, s.prev_heading, report,
                                f"acil fren: on={report.front:.2f} m, hiz={measured_linear:.2f} m/s",
                                [], [])

        # ---------------------------------------------------------------
        # 3) DINAMIK ENGEL: uzerimize gelen bir sey var ve TTC kritik -> dur, bekle.
        #    Ego hizina bakan acil frenden AYRI bir kontrol: orada "ben hizli
        #    gidiyorum" diye frene basiyoruz, burada "o bana dogru geliyor" diye.
        # ---------------------------------------------------------------
        if dynamic_hold:
            linear, angular = emergency_command(s.prev_linear, s.prev_angular, dt, p)
            ttc = time_to_collision(wide_distance, closure, p)
            return self._finish(linear, angular, DriveState.EMERGENCY, s.prev_heading, report,
                                f"dinamik engel yaklasiyor: mesafe={wide_distance:.2f} m, "
                                f"kapanma={closure:+.2f} m/s, TTC={ttc:.2f} s -> dur ve bekle",
                                [], [], closure, dyn_factor)

        # ---------------------------------------------------------------
        # 4) Kilitlenme -> kurtarma manevrasi baslat
        # ---------------------------------------------------------------
        if s.stuck_timer >= p.stuck_time:
            s.state = DriveState.RECOVER
            s.recover_timer = p.recover_duration
            s.recover_sign = report.freer_side_sign
            s.stuck_timer = 0.0
            linear, angular, neden = self._recover_command(rear_clearance, dt)
            side = "sol" if s.recover_sign > 0 else "sag"
            return self._finish(linear, angular, DriveState.RECOVER, s.prev_heading, report,
                                f"kilitlendi -> {neden} + {side}a don", [], [])

        # ---------------------------------------------------------------
        # 5) Aday yonler
        # ---------------------------------------------------------------
        candidates, blocked = self._candidates(scan)
        feasible = [c for c in candidates if c.feasible]

        if not feasible:
            # Gecilebilir yon yok.
            # BLOCKED'a ILK girisse donus yonunu bir kez sec ve KILITLE.
            if s.state is not DriveState.BLOCKED or s.blocked_sign == 0.0:
                fallback = self._widest_free_heading(blocked)
                if fallback is not None and abs(fallback) > 1e-3:
                    s.blocked_sign = 1.0 if fallback > 0.0 else -1.0
                else:
                    s.blocked_sign = report.freer_side_sign
                s.blocked_timer = 0.0
            s.blocked_timer += dt
            # Sahne hareketliyse statik sayaci SIFIRLA: onumden biri geciyor,
            # manevraya baslamak yerine beklemeye devam et.
            if scene_moving:
                s.blocked_static_timer = 0.0
            else:
                s.blocked_static_timer += dt

            # 1) Once DUR VE BEKLE. Yol gercekten kapali mi, yoksa biri mi geciyor?
            if s.blocked_static_timer < p.blocked_wait:
                linear, angular = smooth_command(0.0, 0.0, s.prev_linear,
                                                 s.prev_angular, dt, p)
                why = ("sahne hareketli, biri geciyor" if scene_moving
                       else f"bekliyorum ({s.blocked_static_timer:.1f}/{p.blocked_wait:.1f}s)")
                return self._finish(linear, angular, DriveState.BLOCKED, s.prev_heading,
                                    report, f"yol kapali -> dur ve bekle: {why}",
                                    candidates, blocked, closure, dyn_factor)

            # 2) Ackermann yerinde donemez; ayrica donmek ise yaramiyorsa (zaman
            #    asimi) geri almak tek cikis. Iki durumda da RECOVER.
            if p.bicycle_mode or s.blocked_timer >= p.blocked_timeout:
                s.state = DriveState.RECOVER
                s.recover_timer = p.recover_duration
                s.recover_sign = s.blocked_sign
                linear, angular, neden = self._recover_command(rear_clearance, dt)
                why = ("Ackermann yerinde donemez" if p.bicycle_mode
                       else f"{p.blocked_timeout:.0f}s donmeye ragmen cikis yok")
                return self._finish(linear, angular, DriveState.RECOVER, s.prev_heading, report,
                                    f"yol kapali + {why} -> {neden}", candidates, blocked,
                                    closure, dyn_factor)

            linear, angular = smooth_command(
                0.0, s.blocked_sign * p.max_angular_speed,
                s.prev_linear, s.prev_angular, dt, p,
            )
            side = "sol" if s.blocked_sign > 0 else "sag"
            return self._finish(linear, angular, DriveState.BLOCKED, s.prev_heading, report,
                                f"gecilebilir yon yok -> yerinde {side}a donup ariyorum "
                                f"({s.blocked_timer:.1f}s)",
                                candidates, blocked, closure, dyn_factor)

        best = min(feasible, key=lambda c: c.cost)
        heading = best.heading

        # ---------------------------------------------------------------
        # 6) Durum secimi + hiz
        # ---------------------------------------------------------------
        # Donus hatasi: secilen yon govde cercevesinde verildigi icin, aracin
        # ona donmesi gereken aci dogrudan `heading`in kendisidir (govde ileri = 0).
        heading_error = normalize_angle(heading)
        limiting_clearance = min(report.front, best.clearance)
        target_linear = speed_profile(limiting_clearance, heading_error, p)

        # Hedef yonden ne kadar saptik? Sakinma yapip yapmadigimizin olcusu bu.
        turning_away = abs(normalize_angle(heading - self._goal)) > 0.12  # ~7 derece
        if report.front <= p.stop_distance:
            state = DriveState.BLOCKED
            reason = f"on cok yakin ({report.front:.2f} m) -> dur, {heading:+.2f} rad yonune cevir"
            target_linear = 0.0
        elif report.front <= p.slow_distance or (turning_away and report.front <= p.avoid_trigger_distance):
            state = DriveState.AVOID
            target_linear = max(target_linear, p.min_linear_speed)
            reason = (f"sakinma: on={report.front:.2f} m, secilen yon={heading:+.2f} rad, "
                      f"acikli={best.clearance:.2f} m")
        elif report.front <= p.avoid_trigger_distance:
            state = DriveState.SLOW
            reason = f"engel ufukta ({report.front:.2f} m) -> yavasla"
        else:
            state = DriveState.CRUISE
            reason = f"yol acik ({report.front:.2f} m)"

        # Dinamik engel carpani: tam fren esigine gelmediyse bile yaklasan bir
        # cisim varsa hizi kademeli olarak kes.
        if dyn_factor < 1.0:
            target_linear *= dyn_factor
            reason += f" | yaklasan cisim: kapanma={closure:+.2f} m/s, carpan={dyn_factor:.2f}"
            if state is DriveState.CRUISE:
                state = DriveState.SLOW

        target_angular = p.heading_gain * heading_error
        linear, angular = smooth_command(target_linear, target_angular,
                                         s.prev_linear, s.prev_angular, dt, p)
        s.state = state
        return self._finish(linear, angular, state, heading, report, reason,
                            candidates, blocked, closure, dyn_factor)

    def _finish(self, linear, angular, state, heading, report, reason, candidates, blocked,
                closing_speed: float = 0.0, dynamic_factor: float = 1.0) -> Decision:
        """Durumu kaydet ve `Decision` uret. TEK cikis noktasi -- boylece
        `prev_*` guncellemesini unutma ihtimali yok."""
        self.s.prev_linear = linear
        self.s.prev_angular = angular
        self.s.prev_heading = heading
        # BLOCKED'dan ciktik mi? O zaman kilitli donus yonunu birak.
        if state is not DriveState.BLOCKED:
            self.s.blocked_sign = 0.0
            self.s.blocked_timer = 0.0
            self.s.blocked_static_timer = 0.0
        self.s.state = state
        return Decision(
            linear=linear, angular=angular, state=state, chosen_heading=heading,
            report=report, reason=reason, candidates=candidates, blocked=blocked,
            closing_speed=closing_speed, dynamic_factor=dynamic_factor,
        )
