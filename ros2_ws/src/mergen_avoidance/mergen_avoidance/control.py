"""KONTROL (Control): karar -> tekerleklere gidecek hiz.

Planlayici "su yone git" der; bu dosya onu FIZIKSEL OLARAK UYGULANABILIR bir
(linear, angular) ciftine cevirir. Uc is yapar:
  1. hiz profili   : ne kadar bos yol varsa o kadar hiz
  2. ivme limiti   : komutun zipladigini degil, suruldugunu hissettirir
  3. kinematik     : diferansiyel mi, Ackermann mi?

ROS'a bagimlilik yok -> test edilebilir.
"""
from __future__ import annotations

import math

from .geometry import clamp
from .params import AvoidParams


def rate_limit(target: float, previous: float, max_rate: float, dt: float) -> float:
    """`previous` degerinden `target`a, saniyede en fazla `max_rate` degisimle git.

    Bu bir "slew rate limiter". Robotaksi'de karsiligi dogrudan yolcu konforu:
    0'dan 0.22 m/s'ye tek karede ziplayan bir komut, gercek araca yazildiginda
    gaz/fren aktuatorunu doyurur ve tekerlek kaymasi uretir.
    """
    if dt <= 0.0:
        return previous
    if max_rate <= 0.0:
        raise ValueError("max_rate > 0 olmali")
    max_delta = max_rate * dt
    delta = clamp(target - previous, -max_delta, +max_delta)
    return previous + delta


def speed_profile(front_clearance: float, heading_error: float, p: AvoidParams) -> float:
    """Onumuzdeki bosluga ve donus keskinligine gore hedef ileri hiz [m/s].

    Iki carpan:
      * mesafe carpani : stop_distance'ta 0, slow_distance'ta 1 (lineer rampa)
      * donus carpani  : keskin donuste hizi kes (merkezkac + gorus alani)
    """
    span = p.slow_distance - p.stop_distance
    distance_factor = clamp((front_clearance - p.stop_distance) / span, 0.0, 1.0)

    turn_ratio = clamp(abs(heading_error) / p.max_steer_angle, 0.0, 1.0)
    turn_factor = 1.0 - p.turn_speed_penalty * turn_ratio

    return p.max_linear_speed * distance_factor * turn_factor


def stopping_distance(speed: float, p: AvoidParams) -> float:
    """Bu hizda, tepki suresi + pay dahil durmak icin gereken mesafe [m].

    2023 sartnamesindeki "Ani Fren Gorevi"nin matematigi. Sabit ivmeyle durma:
        d = v*t_tepki + v^2 / (2*a)
    """
    speed = max(speed, 0.0)
    braking = speed * speed / (2.0 * p.max_linear_accel) if p.max_linear_accel > 0 else 0.0
    return speed * p.reaction_time + braking + p.brake_margin


def emergency_brake_required(front_clearance: float, speed: float, p: AvoidParams) -> bool:
    """Ani fren sart mi?

    Iki ayri gerekce, ikisi de yeterli:
      * mutlak esik : engel `emergency_distance` icinde (hiz ne olursa olsun)
      * TTC         : mevcut hizla duramayacagimiz kadar yakin
    """
    if front_clearance <= p.emergency_distance:
        return True
    return front_clearance <= stopping_distance(speed, p)


def time_to_collision(distance: float, closing_speed: float, p: AvoidParams) -> float:
    """Mevcut kapanma hiziyla carpismaya kalan sure [s]. Yaklasmiyorsa `inf`."""
    if closing_speed <= p.closure_min:
        return math.inf
    return max(0.0, distance) / closing_speed


def dynamic_speed_factor(distance: float, closing_speed: float, p: AvoidParams) -> float:
    """Yaklasan cisme gore hiz carpani: 1.0 (serbest) .. 0.0 (tam fren).

    `ttc_warn` ile `ttc_brake` arasinda lineer iner. Kademeli olmasi kasitli:
    ani 0/1 anahtari, yayayi gorup gormeme sinirinda araci zipzip ettirir.
    Sartnamedeki "engele carpmayacak sekilde hizini ayarladi -> +50 puan"
    maddesinin dogrudan karsiligi.
    """
    ttc = time_to_collision(distance, closing_speed, p)
    if ttc >= p.ttc_warn:
        return 1.0
    if ttc <= p.ttc_brake:
        return 0.0
    span = p.ttc_warn - p.ttc_brake
    if span <= 0.0:
        return 0.0
    return (ttc - p.ttc_brake) / span


def apply_kinematics(linear: float, angular: float, p: AvoidParams) -> tuple[float, float]:
    """Arac kinematigine gore (linear, angular) ciftini kisitlar.

    Diferansiyel surus (TurtleBot3): ikisi bagimsiz, sadece doyma siniri var.

    Ackermann / bisiklet modeli (gercek Robotaksi): donus yaricapi `R = v / w`
    ve `R >= min_turn_radius`. Yani `|w| <= v / R_min`. Sonucu:
      * v = 0 iken w = 0  -> ARAC YERINDE DONEMEZ.
      * Yavasken keskin donus imkansizdir.
    Bu kisiti simulasyonda acip test etmezsen, TurtleBot3 uzerinde kusursuz
    calisan algoritman gercek araca tasindiginda ilk kavsakta kilitlenir.
    """
    linear = clamp(linear, -p.max_linear_speed, p.max_linear_speed)
    angular = clamp(angular, -p.max_angular_speed, p.max_angular_speed)

    if not p.bicycle_mode:
        return linear, angular

    if abs(linear) < 1e-6:
        return linear, 0.0

    max_angular_for_speed = abs(linear) / p.min_turn_radius
    angular = clamp(angular, -max_angular_for_speed, +max_angular_for_speed)
    return linear, angular


def smooth_command(
    target_linear: float,
    target_angular: float,
    prev_linear: float,
    prev_angular: float,
    dt: float,
    p: AvoidParams,
) -> tuple[float, float]:
    """Ivme limiti + kinematik kisit, dogru sirayla.

    Sira onemli: once rampa (fizik), sonra kinematik (geometri). Ters sirada
    yaparsan kinematigin urettigi degeri rampa tekrar bozar.
    """
    linear = rate_limit(target_linear, prev_linear, p.max_linear_accel, dt)
    angular = rate_limit(target_angular, prev_angular, p.max_angular_accel, dt)
    return apply_kinematics(linear, angular, p)


def emergency_command(prev_linear: float, prev_angular: float, dt: float, p: AvoidParams) -> tuple[float, float]:
    """Acil durus komutu.

    Hedef 0, ama yine de rampali: `max_linear_accel` ile sifira in. Tek karede
    0 yazmak gercek fren sisteminde uygulanamaz, simulasyonda ise tekerlegi
    kaydirip odometriyi bozar. Panik frende bile fizik gecerli.
    """
    linear = rate_limit(0.0, prev_linear, p.max_linear_accel, dt)
    angular = rate_limit(0.0, prev_angular, p.max_angular_accel, dt)
    # Sifira cok yakinsa tam sifirla (surunmeyi engelle)
    if abs(linear) < 0.01:
        linear = 0.0
    if abs(angular) < 0.02:
        angular = 0.0
    return linear, angular
