"""mergen_avoidance cekirdek mantik testleri.

ROS GEREKTIRMEZ. Iki sekilde kosar:
    pytest -q                      (ROS/colcon ortaminda)
    python3 test/run_tests.py      (ciplak Python, bagimlilik yok)

Buradaki her test ya bir TASARIM KARARINI ya da GERCEKTEN YASANMIS BIR HATAYI
kilitler. Test isimleri bu yuzden uzun: ne bekledigimizi anlatiyorlar.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mergen_avoidance.control import (
    apply_kinematics, dynamic_speed_factor, emergency_brake_required, rate_limit,
    speed_profile, stopping_distance, time_to_collision,
)
from mergen_avoidance.geometry import (
    clamp, free_intervals, in_intervals, merge_intervals, normalize_angle,
)
from mergen_avoidance.params import AvoidParams
from mergen_avoidance.perception import (
    ScanGeometry, angular_profile, blocked_intervals, clean_scan, closing_rate,
    cone_distance, profile_activity, summarize,
)
from mergen_avoidance.planner import AvoidancePlanner, DriveState
from mergen_avoidance.sim2d import (
    SCENARIOS, Box, Circle, Pose, pedestrian_crossing, run_episode, scenario,
    simulate_scan, surface_distance,
)

TB3_GEOM = ScanGeometry(angle_min=0.0, angle_increment=2.0 * math.pi / 360.0,
                        range_min=0.12, range_max=3.5)


def _scan_from(pairs, horizon=3.0):
    """(derece, mesafe) listesinden 360 isinlik tarama uretir; gerisi bos."""
    ranges = [math.inf] * 360
    for deg, dist in pairs:
        ranges[int(deg) % 360] = dist
    return clean_scan(ranges, TB3_GEOM, horizon)


# ===========================================================================
# geometry
# ===========================================================================
def test_normalize_angle_wraps_to_pi_interval():
    assert abs(normalize_angle(3 * math.pi / 2) - (-math.pi / 2)) < 1e-9
    assert abs(normalize_angle(-3 * math.pi / 2) - (+math.pi / 2)) < 1e-9
    assert abs(normalize_angle(math.pi) - math.pi) < 1e-9


def test_merge_intervals_joins_overlaps_and_keeps_gaps():
    assert merge_intervals([(0, 1), (0.5, 2), (3, 4)]) == [(0, 2), (3, 4)]
    assert merge_intervals([]) == []


def test_free_intervals_is_complement_of_blocked():
    gaps = free_intervals([(-0.2, 0.3), (1.0, 1.2)], -1.5, 1.5)
    assert gaps == [(-1.5, -0.2), (0.3, 1.0), (1.2, 1.5)]
    # Tamamen kapali -> hic bosluk yok
    assert free_intervals([(-2.0, 2.0)], -1.5, 1.5) == []


def test_clamp_rejects_inverted_bounds():
    """Sessiz bozulma yerine gurultulu hata: parametre karisikligini yakalar."""
    try:
        clamp(0.0, 1.0, -1.0)
    except ValueError:
        return
    raise AssertionError("ters sinirlarda ValueError beklenir")


# ===========================================================================
# perception
# ===========================================================================
def test_clean_scan_treats_infinity_as_free_space_not_as_wall():
    """EN KRITIK ALGILAMA KURALI.

    'Donus yok' (inf) = 'orada bir sey yok'. Bu isinlari ATARSAN, bos koridor
    taramada hic gorunmez ve planlayici onu duvar sanar.
    """
    scan = clean_scan([math.inf] * 360, TB3_GEOM, horizon=3.0)
    assert len(scan) == 360
    assert all(r == 3.0 for r in scan.ranges)


def test_clean_scan_drops_nan_and_self_hits():
    ranges = [math.nan] * 180 + [0.05] * 90 + [1.0] * 90   # nan, range_min alti, gecerli
    scan = clean_scan(ranges, TB3_GEOM, horizon=3.0)
    assert len(scan) == 90
    assert all(abs(r - 1.0) < 1e-9 for r in scan.ranges)


def test_clean_scan_clamps_beyond_horizon():
    scan = clean_scan([5.0] * 360, TB3_GEOM, horizon=3.0)   # 5.0 > range_max -> bos
    assert all(r == 3.0 for r in scan.ranges)


def test_cone_distance_robustness_rejects_single_noisy_ray():
    """Tek hatali isin robotu durdurmamali (robust_k=2)."""
    scan = _scan_from([(0, 0.15)])          # tam onde 1 isin cok yakin, gerisi bos
    assert cone_distance(scan, 0.0, 0.35, 3.0, robust_k=1) < 0.2   # saf min aldanir
    assert cone_distance(scan, 0.0, 0.35, 3.0, robust_k=2) == 3.0  # k=2 elemeyi yapar


def test_cone_distance_sees_real_obstacle_despite_robustness():
    """Gercek engel onlarca isin dusurur; robust_k onu KACIRMAMALI."""
    scan = _scan_from([(d, 0.8) for d in range(-15, 16)])
    assert abs(cone_distance(scan, 0.0, 0.35, 3.0, robust_k=2) - 0.8) < 1e-9


def test_blocked_intervals_inflate_more_when_obstacle_is_closer():
    """C-space sisirme: ayni engel yakinken daha cok aci kapatir."""
    far = blocked_intervals(_scan_from([(0, 2.0)]), 0.26, 3.0, 1.57)
    near = blocked_intervals(_scan_from([(0, 0.5)]), 0.26, 3.0, 1.57)
    width = lambda iv: sum(hi - lo for lo, hi in iv)
    assert width(near) > width(far) > 0.0


def test_blocked_intervals_empty_when_all_clear():
    assert blocked_intervals(clean_scan([math.inf] * 360, TB3_GEOM, 3.0), 0.26, 3.0, 1.57) == []


def test_summarize_reports_freer_side_correctly():
    left_blocked = _scan_from([(deg, 0.6) for deg in range(60, 121)])
    assert summarize(left_blocked, 3.0, 0.35).freer_side_sign == -1.0   # sag bos
    right_blocked = _scan_from([(deg, 0.6) for deg in range(240, 301)])
    assert summarize(right_blocked, 3.0, 0.35).freer_side_sign == +1.0  # sol bos


# ===========================================================================
# control
# ===========================================================================
def test_rate_limit_caps_change_per_step():
    assert abs(rate_limit(1.0, 0.0, 0.35, 0.05) - 0.0175) < 1e-9
    assert abs(rate_limit(-1.0, 0.0, 0.35, 0.05) - (-0.0175)) < 1e-9
    assert rate_limit(0.01, 0.0, 10.0, 0.05) == 0.01          # hedef yakinsa tam ulasir


def test_speed_profile_is_zero_at_stop_and_full_at_slow():
    p = AvoidParams()
    assert speed_profile(p.stop_distance, 0.0, p) == 0.0
    assert abs(speed_profile(p.slow_distance, 0.0, p) - p.max_linear_speed) < 1e-9
    assert abs(speed_profile(10.0, 0.0, p) - p.max_linear_speed) < 1e-9   # doyar


def test_speed_profile_slows_down_in_sharp_turns():
    p = AvoidParams()
    straight = speed_profile(p.slow_distance, 0.0, p)
    turning = speed_profile(p.slow_distance, p.max_steer_angle, p)
    assert turning < straight


def test_emergency_brake_scales_with_speed():
    """Hizliyken daha uzaktan fren: TTC mantigi."""
    p = AvoidParams()
    assert emergency_brake_required(0.20, 0.0, p) is True      # mutlak esik
    assert emergency_brake_required(1.50, 0.22, p) is False
    d = stopping_distance(0.22, p)
    assert emergency_brake_required(d - 0.01, 0.22, p) is True
    assert emergency_brake_required(d + 0.01, 0.22, p) is False


def test_ackermann_cannot_rotate_in_place():
    """Gercek Robotaksi kinematigi: v=0 iken donus yok."""
    p = AvoidParams(bicycle_mode=True, min_turn_radius=4.0)
    assert apply_kinematics(0.0, 1.0, p) == (0.0, 0.0)
    v, w = apply_kinematics(0.2, 1.0, p)
    assert abs(w - 0.2 / 4.0) < 1e-9          # |w| <= v/R
    # Diferansiyel suruste ayni cagri serbest
    assert apply_kinematics(0.0, 1.0, AvoidParams())[1] == 1.0


# ===========================================================================
# planner -- durum makinesi
# ===========================================================================
def test_cruises_straight_when_world_is_empty():
    pl = AvoidancePlanner()
    scan = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)
    d = pl.step(scan, measured_linear=0.0, dt=0.05)
    for _ in range(40):
        d = pl.step(scan, measured_linear=d.linear, dt=0.05)
    assert d.state is DriveState.CRUISE
    assert d.linear > 0.2
    assert abs(d.angular) < 0.05
    assert abs(d.chosen_heading) < 0.05


def test_emergency_brake_when_wall_is_very_close():
    pl = AvoidancePlanner()
    scan = _scan_from([(deg, 0.20) for deg in range(-30, 31)])
    d = pl.step(scan, measured_linear=0.22, dt=0.05)
    assert d.state is DriveState.EMERGENCY
    assert d.linear < 0.22          # frene basiyor
    for _ in range(60):             # birkac saniye sonra tam durmus olmali
        d = pl.step(scan, measured_linear=d.linear, dt=0.05)
    assert d.linear == 0.0


def test_steers_towards_the_open_side():
    """Sag kapali, sol acik -> sola donmeli (pozitif acisal hiz)."""
    pl = AvoidancePlanner()
    # Sadece SAG tarafi kapat: 270..360 derece (= -90..0) ve 0..20 derece.
    scan = _scan_from([(deg, 0.9) for deg in range(270, 360)]
                      + [(deg, 0.9) for deg in range(0, 20)])
    d = pl.step(scan, measured_linear=0.1, dt=0.05)
    for _ in range(10):
        d = pl.step(scan, measured_linear=d.linear, dt=0.05)
    assert d.chosen_heading > 0.0, "sol taraf acikken sola yonelmeli"
    assert d.state in (DriveState.AVOID, DriveState.SLOW, DriveState.BLOCKED)


def test_blocked_turn_direction_is_latched_and_does_not_chatter():
    """REGRESYON: simetrik cikmaz sokakta donus yonu taraf degistirmemeli.

    Yasanan hata: `freer_side_sign` her karede yeniden hesaplandigi icin
    simetrik koridorda isaret surekli flip etti; arac -19 ile +2 derece
    arasinda salinip cikmaz sokaktan hic kurtulamadi (sonsuz limit cevrimi).
    Cozum: BLOCKED'a girince yon BIR KEZ secilir ve kilitlenir.
    """
    pl = AvoidancePlanner()
    scan = _scan_from([(deg, 0.6) for deg in range(-100, 101)])   # on yarim kure kapali
    signs = set()
    for _ in range(60):
        d = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
        if d.state is DriveState.BLOCKED and abs(d.angular) > 1e-6:
            signs.add(1 if d.angular > 0 else -1)
    assert len(signs) <= 1, f"donus yonu taraf degistirdi: {signs}"


def test_blocked_escalates_to_recover_after_timeout():
    """Donmek ise yaramiyorsa geri al."""
    p = AvoidParams(blocked_timeout=1.0)
    pl = AvoidancePlanner(p)
    scan = _scan_from([(deg, 0.6) for deg in range(-100, 101)])
    states = []
    for _ in range(60):
        # Donuyormus gibi besle ki 'sikisti' sayaci devreye girmesin;
        # tirmanmayi tetikleyen sey blocked_timeout olsun.
        d = pl.step(scan, measured_linear=0.0, measured_angular=1.0, dt=0.05)
        states.append(d.state)
    assert DriveState.BLOCKED in states
    assert DriveState.RECOVER in states, "zaman asimindan sonra RECOVER beklenir"


def test_recover_triggers_when_vehicle_makes_no_progress():
    """Komut veriyoruz ama arac kipirdamiyor (tekerlek bosta / engele dayali)."""
    p = AvoidParams(stuck_time=0.5)
    pl = AvoidancePlanner(p)
    scan = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)    # yol acik gorunuyor
    d = None
    for _ in range(40):
        d = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
    assert d.state is DriveState.RECOVER
    assert d.linear < 0.0, "kurtarma geri gitmeli"


def test_recover_manoeuvre_is_not_interrupted():
    """Kurtarma BOLUNMEZ: yarim kalmis manevra robotu kosede birakir."""
    p = AvoidParams(stuck_time=0.5, recover_duration=1.0)
    pl = AvoidancePlanner(p)
    scan = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)

    # RECOVER'a girene kadar 'kipirdamiyor' diye besle (10 tick = 0.5 s yeter).
    d = None
    for _ in range(30):
        d = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
        if d.state is DriveState.RECOVER:
            break
    assert d.state is DriveState.RECOVER, "kilitlenme RECOVER'i tetiklemedi"

    # Artik 'duzgun hareket ediyorum' diye besle. Manevra 1.0 s surecek;
    # 10 tick = 0.5 s sonra HALA RECOVER olmali -- yarida kesilmemeli.
    seen = []
    for _ in range(10):
        d = pl.step(scan, measured_linear=-0.08, measured_angular=0.9, dt=0.05)
        seen.append(d.state)
    assert all(st is DriveState.RECOVER for st in seen), seen

    # Sure dolduktan sonra normale donmeli.
    for _ in range(15):
        d = pl.step(scan, measured_linear=-0.08, measured_angular=0.9, dt=0.05)
    assert d.state is not DriveState.RECOVER, "kurtarma hic bitmiyor"


def test_bicycle_mode_backs_up_instead_of_spinning_when_blocked():
    """Ackermann yerinde donemez -> tek cikis geri almak.

    Not: ILK karede degil. Yeni davranis once DURUP BEKLEMEK (sahne degisiyor
    olabilir); ancak sahne donmus oldugu anlasilinca manevra baslar.
    """
    p = AvoidParams(bicycle_mode=True, min_turn_radius=4.0, blocked_wait=0.5)
    pl = AvoidancePlanner(p)
    scan = _scan_from([(deg, 0.6) for deg in range(-100, 101)])

    first = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
    assert first.state is DriveState.BLOCKED, "once durup beklemeli"
    assert first.linear == 0.0

    d = first
    for _ in range(30):
        d = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
        if d.state is DriveState.RECOVER:
            break
    assert d.state is DriveState.RECOVER, "sahne donmusken geri almaya gecmeli"
    assert d.linear <= 0.0


def test_acceleration_limit_respected_on_first_tick():
    """Ilk karede 0'dan tam hiza ziplamamali (yolcu konforu / aktuator limiti)."""
    p = AvoidParams()
    pl = AvoidancePlanner(p)
    scan = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)
    d = pl.step(scan, 0.0, 0.0, dt=0.05)
    assert d.linear <= p.max_linear_accel * 0.05 + 1e-9


def test_heading_choice_is_stable_between_consecutive_ticks():
    """HISTEREZIS: iki esit bosluk arasinda karar titremesi olmamali."""
    pl = AvoidancePlanner()
    scan = _scan_from([(deg, 1.0) for deg in range(-8, 9)])   # tam onde dar engel
    headings = []
    for _ in range(40):
        d = pl.step(scan, measured_linear=0.1, dt=0.05)
        headings.append(d.chosen_heading)
    flips = sum(1 for a, b in zip(headings, headings[1:])
                if a * b < 0 and abs(a - b) > 0.2)
    assert flips <= 1, f"karar {flips} kez taraf degistirdi: titreme var"


def test_goal_heading_is_interpreted_in_body_frame():
    """Hedef govde cercevesinde verilir; planlayici o yone yonelmeli."""
    pl = AvoidancePlanner()
    scan = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)
    d = None
    for _ in range(20):
        d = pl.step(scan, measured_linear=0.1, dt=0.05, goal_heading=+0.5)
    assert d.chosen_heading > 0.3, d.chosen_heading


# ===========================================================================
# uctan uca -- 2B simulator (Gazebo'dan once calisan asil dogrulama)
# ===========================================================================
def test_every_scenario_is_traversed_without_collision():
    """ANA REGRESYON: tum parkurlarda carpisma YOK.

    Bu test bozulursa algoritma gerilemis demektir -- Gazebo acmaya gerek yok.
    """
    failures = []
    for name in SCENARIOS:
        r = run_episode(scenario(name), goal_x=9.0, max_time=60.0, world_goal_heading=0.0)
        if r.collided:
            failures.append(f"{name}: CARPISMA (en yakin {r.min_clearance:.2f} m)")
    assert not failures, failures


def test_open_scenarios_reach_the_goal():
    """Kapan olmayan parkurlarda hedefe VARMALI (sadece 'carpmadi' yetmez:
    duvara bakip hic hareket etmeyen bir robot da carpmaz)."""
    reachable = ["w1_empty", "w2_cone_slalom", "w3_corridor",
                 "w4_lane_block", "w6_dense_clutter"]
    for name in reachable:
        r = run_episode(scenario(name), goal_x=9.0, max_time=60.0, world_goal_heading=0.0)
        assert r.reached_goal, f"{name}: hedefe ulasilamadi ({r.summary()})"


def test_corridor_keeps_vehicle_near_centre_line():
    """Serit disciplini: koridorda yan sapma kucuk kalmali.
    Robotaksi'de bu dogrudan 'serit ihlali yapmadi -> +50 puan' demek."""
    r = run_episode(scenario("w3_corridor"), goal_x=9.0, max_time=60.0, world_goal_heading=0.0)
    max_abs_y = max(abs(y) for _, y in r.trajectory)
    assert max_abs_y < 0.8, f"yan sapma cok buyuk: {max_abs_y:.2f} m"


def test_trap_pocket_is_recognised_and_never_hit():
    """Kapan senaryosu: BILINEN SINIR, ama guvenli.

    Salt tepkisel bir planlayici yerel minimumdan (cikmaz cep) kendi basina
    cikamaz -- bu bir kusur degil, yontemin tanimli siniri. Beklentimiz:
      * HICBIR SEKILDE carpmamak
      * kapani FARK ETMEK (BLOCKED/RECOVER durumlarina girmek)
      * duvara dayanip gaz vermemek (saglikli bir mesafe birakmak)
    Hedefe varmasini BEKLEMIYORUZ; onun cozumu global planlayici.
    """
    r = run_episode(scenario("w7_trap_pocket"), goal_x=9.0, max_time=60.0,
                    world_goal_heading=0.0)
    assert not r.collided, f"kapanda carpti (en yakin {r.min_clearance:.2f} m)"
    assert {"BLOCKED", "RECOVER"} & set(r.states), "kapani hic fark etmedi"
    assert r.min_clearance > 0.30, f"duvara fazla dayandi: {r.min_clearance:.2f} m"


def test_raycast_geometry_is_correct():
    """Simulatorun kendisi de test edilir; yoksa yanlis sensorle dogru algoritma
    test etmis oluruz."""
    obs = [Circle(2.0, 0.0, 0.5)]
    assert abs(simulate_scan(Pose(0, 0, 0), obs)[0][0] - 1.5) < 1e-6      # on
    assert math.isinf(simulate_scan(Pose(0, 0, 0), obs)[0][180])          # arka bos
    # Robot donunce engel de 'donmeli'
    ranges, _ = simulate_scan(Pose(0, 0, math.pi / 2), obs)
    assert abs(ranges[270] - 1.5) < 1e-6


# ===========================================================================
# dinamik engel katmani (kapanma hizi + sahne hareketliligi)
# ===========================================================================
def test_closing_rate_sign_convention():
    """Pozitif = yaklasiyor. Isaret hatasi burada yakalanir."""
    # Gercek bir engel ONLARCA isin dusurur. Tek isinlik engel kullanmak
    # `robust_k=2` filtresini tetikler ve olcum 0 doner -- bu kasitli bir
    # tasarim, testin onu atlatmasi gerekir.
    far = _scan_from([(deg, 2.0) for deg in range(-20, 21)])
    near = _scan_from([(deg, 1.0) for deg in range(-20, 21)])
    approaching = closing_rate(far, near, dt=1.0, center=0.0, half_width=0.35, horizon=3.0)
    receding = closing_rate(near, far, dt=1.0, center=0.0, half_width=0.35, horizon=3.0)
    assert approaching > 0.0, "yaklasma pozitif olmali"
    assert receding < 0.0, "uzaklasma negatif olmali"
    assert abs(approaching - 1.0) < 1e-6


def test_closing_rate_is_zero_without_history():
    """Ilk karede gecmis yok -> 0 dondurmeli, cop deger uretmemeli."""
    assert closing_rate(None, _scan_from([(0, 1.0)]), 0.05, 0.0, 0.35, 3.0) == 0.0


def test_closing_rate_ignores_empty_scenes():
    """Iki karede de ufuk bos ise 'yaklasma' uydurmamali."""
    empty = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)
    assert closing_rate(empty, empty, 0.05, 0.0, 0.35, 3.0) == 0.0


def test_time_to_collision_and_speed_factor_are_consistent():
    p = AvoidParams()
    assert math.isinf(time_to_collision(2.0, 0.0, p))          # yaklasmiyor
    assert dynamic_speed_factor(2.0, 0.0, p) == 1.0            # serbest
    # ttc_brake esiginin altinda tam fren
    assert dynamic_speed_factor(0.5, 1.0, p) == 0.0
    # arada kademeli
    mid = dynamic_speed_factor(1.0, 0.45, p)
    assert 0.0 < mid < 1.0, mid


def test_angular_profile_has_requested_bucket_count():
    scan = clean_scan([math.inf] * 360, TB3_GEOM, 3.0)
    prof = angular_profile(scan, horizon=3.0, limit=1.57, buckets=24)
    assert len(prof) == 24
    assert all(v == 3.0 for v in prof)


def test_profile_activity_detects_motion_and_ignores_stillness():
    scan_a = _scan_from([(deg, 2.0) for deg in range(-20, 21)])
    scan_b = _scan_from([(deg, 1.0) for deg in range(-20, 21)])
    prof_a = angular_profile(scan_a, 3.0, 1.57, 24)
    prof_b = angular_profile(scan_b, 3.0, 1.57, 24)
    assert profile_activity(prof_a, prof_a, 0.05) == 0.0       # donmus sahne
    assert profile_activity(prof_a, prof_b, 0.05) > 0.10       # hareketli sahne
    assert profile_activity(None, prof_b, 0.05) == 0.0         # gecmis yok


def test_blocked_road_waits_before_searching_for_a_way_out():
    """REGRESYON: yolu kapatan bir sey varken HEMEN 180 derece donmemeli.

    Yasanan hata: yaya koridoru kapatinca arac yerinde donup ters yone gitti
    (sartname: ters yon ihlali; dogrusu "engel gecene kadar bekledi -> +50").
    """
    p = AvoidParams(blocked_wait=1.0)
    pl = AvoidancePlanner(p)
    scan = _scan_from([(deg, 0.6) for deg in range(-100, 101)])
    # Ilk 1 saniye (20 tick) boyunca durmali, donmemeli
    for _ in range(18):
        d = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
        assert d.state is DriveState.BLOCKED
        assert d.linear == 0.0
        assert abs(d.angular) < 1e-6, "beklemesi gerekirken donmeye basladi"


def test_moving_scene_extends_the_wait_indefinitely():
    """Sahne degismeye devam ettigi surece arac beklemeyi surdurmeli."""
    p = AvoidParams(blocked_wait=0.5)
    pl = AvoidancePlanner(p)
    # Her karede engel mesafesini oynatarak 'hareketli sahne' uretiyoruz.
    for i in range(60):
        dist = 0.6 + 0.25 * (i % 2)
        scan = _scan_from([(deg, dist) for deg in range(-100, 101)])
        d = pl.step(scan, measured_linear=0.0, measured_angular=0.0, dt=0.05)
    assert d.state is DriveState.BLOCKED, d.state
    assert abs(d.angular) < 1e-6, "sahne hareketliyken manevraya baslamamali"


def test_static_blocked_scene_eventually_starts_searching():
    """Sahne gercekten donmusse beklemeyi birakip yon aramaya gecmeli."""
    p = AvoidParams(blocked_wait=0.5, blocked_timeout=10.0)
    pl = AvoidancePlanner(p)
    scan = _scan_from([(deg, 0.6) for deg in range(-100, 101)])
    turned = False
    for _ in range(40):
        d = pl.step(scan, measured_linear=0.0, measured_angular=1.0, dt=0.05)
        if d.state is DriveState.BLOCKED and abs(d.angular) > 1e-6:
            turned = True
            break
    assert turned, "donmus sahnede hic yon aramaya baslamadi"


def test_pedestrian_crossing_is_survived_and_journey_continues():
    """UCTAN UCA: aniden karsiya gecen yaya.

    Sartname puanlamasi: sakindi / bekledi / hizini ayarladi -> +50,
    carpti -> TURDAN DISKALIFIYE. Yani iki kriter birlikte saglanmali:
      * carpmamak
      * yaya gectikten sonra GOREVE DEVAM etmek (durup kalmamak)
    """
    r = run_episode(scenario("w8_pedestrian"), goal_x=9.0, max_time=75.0,
                    world_goal_heading=0.0, dynamic=pedestrian_crossing())
    assert not r.collided, f"yayaya carpti (en yakin {r.min_clearance:.2f} m)"
    assert r.reached_goal, f"yaya gectikten sonra devam etmedi ({r.summary()})"
    assert "EMERGENCY" in r.states or "BLOCKED" in r.states, "hic tepki vermedi"


def test_pedestrian_scenario_costs_time_but_not_safety():
    """Beklemek sureyi uzatir -- ama bu DOGRU takas. Olculu olmali."""
    quiet = run_episode(scenario("w8_pedestrian"), goal_x=9.0, max_time=75.0,
                        world_goal_heading=0.0)
    busy = run_episode(scenario("w8_pedestrian"), goal_x=9.0, max_time=75.0,
                       world_goal_heading=0.0, dynamic=pedestrian_crossing())
    assert busy.duration > quiet.duration, "yaya hic etkilemediyse fren calismiyor"
    assert busy.duration - quiet.duration < 20.0, "gereginden fazla bekledi"


# ===========================================================================
# sim2d geometrisi -- kutu engeller
# ===========================================================================
def test_box_raycast_hits_front_face():
    obs = [Box(2.0, 0.0, 1.0, 1.0)]       # merkez (2,0), 1x1 -> on yuz x=1.5
    ranges, _ = simulate_scan(Pose(0, 0, 0), obs)
    assert abs(ranges[0] - 1.5) < 1e-6
    assert math.isinf(ranges[180])        # arka bos


def test_box_raycast_misses_when_offset():
    obs = [Box(2.0, 5.0, 1.0, 1.0)]       # yoldan uzak
    ranges, _ = simulate_scan(Pose(0, 0, 0), obs)
    assert math.isinf(ranges[0])


def test_surface_distance_matches_geometry():
    assert abs(surface_distance(Circle(3.0, 0.0, 0.5), 0.0, 0.0) - 2.5) < 1e-9
    assert surface_distance(Circle(0.0, 0.0, 0.5), 0.0, 0.0) == 0.0        # icinde
    assert abs(surface_distance(Box(2.0, 0.0, 1.0, 1.0), 0.0, 0.0) - 1.5) < 1e-9
    assert surface_distance(Box(0.0, 0.0, 1.0, 1.0), 0.0, 0.0) == 0.0      # icinde
    # kose mesafesi: (1.5,1.5) kosesinden (0,0)'a
    d = surface_distance(Box(2.0, 2.0, 1.0, 1.0), 0.0, 0.0)
    assert abs(d - math.hypot(1.5, 1.5)) < 1e-9


# ===========================================================================
# Uretilen Gazebo dunyalari -- 2B senaryolarla tutarlilik
# ===========================================================================
WORLDS_DIR = Path(__file__).resolve().parents[1] / "worlds"


def test_generated_worlds_are_valid_xml():
    """REGRESYON: uretilen .world dosyalari gecerli XML olmali.

    Yasanan hata: urettigimiz 8 dunyanin TAMAMI gecersiz XML'di, cunku bir
    yorumda "URETILDI -- elle duzenleme" yaziyordu ve XML 1.0'a gore yorum
    icinde '--' olamaz. Gazebo'nun TinyXML'i boyle durumlarda anlamsiz bir
    mesajla patliyor; hatayi ancak Gazebo'yu acinca gorursun. Burada saniyede
    yakaliyoruz.
    """
    import xml.etree.ElementTree as ET

    worlds = sorted(WORLDS_DIR.glob("*.world"))
    assert worlds, f"hic dunya dosyasi yok: {WORLDS_DIR}"
    for path in worlds:
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            raise AssertionError(f"{path.name}: gecersiz XML -- {exc}") from exc


def test_every_scenario_has_a_generated_world():
    """Her 2B senaryonun Gazebo karsiligi uretilmis olmali."""
    missing = [name for name in SCENARIOS
               if not (WORLDS_DIR / f"{name}.world").is_file()]
    assert not missing, (
        f"su senaryolarin dunyasi uretilmemis: {missing}\n"
        "Cozum: python3 scripts/generate_worlds.py"
    )


def test_world_model_count_matches_scenario_obstacles():
    """Dunya dosyasindaki model sayisi senaryodaki engel sayisiyla tutmali.

    Bu, 2B simulator ile Gazebo'nun AYRISMADIGINI dogrulayan test. Ayrisirsa
    "2B'de gecti, Gazebo'da carpti" durumuna duseriz.
    Model sayisi = engeller + zemin (+ w8'de yaya).
    """
    import xml.etree.ElementTree as ET

    for name in SCENARIOS:
        path = WORLDS_DIR / f"{name}.world"
        world = ET.parse(path).getroot().find("world")
        models = [m.get("name") for m in world.findall("model")]

        expected = len(scenario(name)) + 1          # +1 = ground_plane
        if name == "w8_pedestrian":
            expected += 1                            # + dinamik engel modeli
            assert "pedestrian" in models, f"{name}: yaya modeli yok"
        assert len(models) == expected, (
            f"{name}: dunyada {len(models)} model var, senaryoya gore "
            f"{expected} olmali -> dunyalar guncel degil "
            f"(python3 scripts/generate_worlds.py)"
        )
