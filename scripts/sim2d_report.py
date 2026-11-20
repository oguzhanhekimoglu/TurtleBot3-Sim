#!/usr/bin/env python3
"""Tum senaryolari 2B simulatorde kosturur ve Markdown sonuc tablosu uretir.

ROS/Gazebo GEREKTIRMEZ -- saf Python. Bu yuzden:
  * kurulum beklemeden algoritmanin durumunu gorebilirsin
  * her kod degisikliginden sonra saniyeler icinde gerileme kontrolu yapilir
  * Robotaksi simulasyon raporundaki "8.2 Test Sonuclari" tablosunun
    ilk hali dogrudan buradan cikar

Kullanim:
    python3 scripts/sim2d_report.py
    python3 scripts/sim2d_report.py --out docs/test-sonuclari.md
    python3 scripts/sim2d_report.py --ackermann      # Ackermann kinematigiyle
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "ros2_ws" / "src" / "mergen_avoidance"
sys.path.insert(0, str(PKG))

from mergen_avoidance.params import AvoidParams                      # noqa: E402
from mergen_avoidance.sim2d import (                                  # noqa: E402
    SCENARIOS, pedestrian_crossing, run_episode, scenario,
)

DESCRIPTIONS = {
    "w1_empty": "Bos alan -- taban cizgisi (hiz, duz gitme)",
    "w2_cone_slalom": "Koni slalomu -- art arda yon degistirme, histerezis",
    "w3_corridor": "Koridor + variller -- serit disiplini",
    "w4_lane_block": "Seridi kismen kapatan statik engel (sartname: statik engel)",
    "w5_dead_end": "Kisa cikmaz koridor -- etrafindan dolasma",
    "w6_dense_clutter": "Yogun dagilmis koniler -- surekli yeniden planlama",
    "w7_trap_pocket": "Kapali cep -- BILINEN SINIR (yerel minimum)",
    "w8_pedestrian": "Aniden karsiya gecen yaya (sartname: dinamik engel)",
}

GOAL_X = 9.0
MAX_TIME = 75.0


def verdict(name: str, r) -> str:
    if r.collided:
        return "BASARISIZ (carpisma)"
    if name == "w7_trap_pocket":
        # Hedefe varmasi beklenmiyor; guvenli kalmasi ve kapani fark etmesi yeterli.
        recognised = bool({"BLOCKED", "RECOVER"} & set(r.states))
        return "BEKLENEN (kapan farkedildi)" if recognised else "BASARISIZ (kapani gormedi)"
    return "BASARILI" if r.reached_goal else "KISMEN (hedefe varmadi)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=None, help="Markdown cikti dosyasi")
    ap.add_argument("--ackermann", action="store_true",
                    help="bicycle_mode=True ile kostur (gercek arac kinematigi)")
    ap.add_argument("--max-time", type=float, default=MAX_TIME)
    args = ap.parse_args()

    params = AvoidParams(bicycle_mode=True) if args.ackermann else AvoidParams()
    params.validate()

    rows = []
    print(f"{len(SCENARIOS)} senaryo kosuluyor "
          f"({'Ackermann' if args.ackermann else 'diferansiyel'} kinematik)...\n")

    for name in SCENARIOS:
        dynamic = pedestrian_crossing() if name == "w8_pedestrian" else None
        r = run_episode(scenario(name), params=params, goal_x=GOAL_X,
                        max_time=args.max_time, world_goal_heading=0.0, dynamic=dynamic)
        states = Counter(r.states)
        total = max(1, sum(states.values()))
        busiest = ", ".join(
            f"{st} %{100 * n / total:.0f}"
            for st, n in sorted(states.items(), key=lambda kv: -kv[1])[:3]
        )
        max_abs_y = max((abs(y) for _, y in r.trajectory), default=0.0)
        rows.append({
            "name": name,
            "desc": DESCRIPTIONS.get(name, ""),
            "verdict": verdict(name, r),
            "collided": "VAR" if r.collided else "yok",
            "min_clear": r.min_clearance,
            "dist": r.distance_travelled,
            "dur": r.duration,
            "max_y": max_abs_y,
            "states": busiest,
        })
        print(f"  {name:18s} {verdict(name, r):28s} {r.summary()}")

    # ---- Markdown ----
    kin = "Ackermann (bicycle_mode=True)" if args.ackermann else "diferansiyel surus"
    md = [
        "# 2B Simulator Test Sonuclari",
        "",
        f"Kinematik: **{kin}**  |  hedef: x >= {GOAL_X:.0f} m  |  "
        f"sure siniri: {args.max_time:.0f} s  |  kontrol: {1/params.control_period:.0f} Hz",
        "",
        "Bu tablo `python3 scripts/sim2d_report.py` ile uretildi. Gazebo gerektirmez;",
        "ayni engel yerlesimleri `worlds/*.world` dosyalarina da bu senaryolardan uretilir",
        "(`scripts/generate_worlds.py`), dolayisiyla 2B ve 3B testler ayni parkuru olcer.",
        "",
        "| Senaryo | Ne test ediyor | Sonuc | Carpisma | En yakin yaklasma | Yol | Sure | Maks yan sapma | Baskin durumlar |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        mc = "-" if r["min_clear"] != r["min_clear"] else f"{r['min_clear']:.2f} m"
        md.append(
            f"| `{r['name']}` | {r['desc']} | **{r['verdict']}** | {r['collided']} | "
            f"{mc} | {r['dist']:.2f} m | {r['dur']:.1f} s | {r['max_y']:.2f} m | {r['states']} |"
        )

    collisions = sum(1 for r in rows if r["collided"] == "VAR")
    md += [
        "",
        f"**Ozet:** {len(rows)} senaryo, {collisions} carpisma.",
        "",
        "Notlar:",
        "",
        "- `w7_trap_pocket` kasitli olarak cozulemez bir parkurdur. Salt tepkisel",
        "  (reactive) bir yerel planlayici yerel minimumdan kendi basina cikamaz; bu",
        "  yontemin tanimli sinirudir, kusuru degil. Beklentimiz carpmamak ve kapani",
        "  fark etmek. Cozum global planlayici (Nav2 / GEOJSON rotasi) eklemektir.",
        "- `w8_pedestrian` senaryosunda sure uzamasi BEKLENIR: arac yaya gecene kadar",
        "  durup bekler. Sartname puanlamasi tam olarak bunu odullendirir",
        "  ('engel tamamen gecene kadar bekledi -> +50').",
        "- 'En yakin yaklasma' robot MERKEZINDEN engel YUZEYINE olan mesafedir.",
        f"  Govde yaricapi ~0.22 m oldugundan {0.22:.2f} m'nin altindaki degerler temastir.",
    ]

    if args.ackermann:
        md += [
            "",
            "## Ackermann kosusu hakkinda -- bu testin ANLAMI",
            "",
            "Bu kosuda `bicycle_mode=True` ve `min_turn_radius="
            f"{params.min_turn_radius:.1f} m` kullanildi: yani TurtleBot3 govdesine",
            "**gercek Robotaksi aracinin donus kisiti** dayatildi. Arac artik yerinde",
            "donemez ve `|w| <= v / R_min` kisitina tabidir.",
            "",
            "Bu bilerek ZOR bir kurgudur ve amaci su dersi olculebilir hale getirmek:",
            "",
            "> Diferansiyel suruste kusursuz calisan bir algoritma, yerinde donmeye",
            "> dayaniyorsa Ackermann bir aracta calismaz.",
            "",
            "Basarisiz olan senaryolar bir ALGORITMA hatasi degil, **kinematik",
            "uyumsuzluktur**: 0.8 m araliklı konilerin arasindan 4 m donus yaricapiyla",
            "gecmek fiziksel olarak imkansizdir. Gercek yarismada parkur da araca gore",
            "olceklidir (serit genisligi ~3 m, engeller metrelerce arali).",
            "",
            "Dogru karsilastirma icin iki secenek var:",
            "",
            "1. Parkuru araca gore olcekle (mesafeleri ~4x buyut) -- gercege en yakini.",
            "2. TurtleBot3 olcegine uygun bir `min_turn_radius` kullan (~0.5 m).",
            "",
            "Buradaki tablo 1. veya 2. secenek UYGULANMADAN alinmistir; yani en kotu",
            "durumu gosterir. Yol haritasindaki is: parkuru olcekleyip Ackermann",
            "kosusunu yeniden almak ve gerekirse planlayiciya donus yaricapi farkindali",
            "aday rota uretimi (arc-based sampling) eklemek.",
        ]
    text = "\n".join(md) + "\n"

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf8")
        print(f"\nMarkdown yazildi: {args.out}")
    else:
        print("\n" + text)

    return 1 if collisions else 0


if __name__ == "__main__":
    sys.exit(main())
