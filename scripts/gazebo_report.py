#!/usr/bin/env python3
"""Toplu Gazebo kosusunun loglarindan Markdown sonuc tablosu uretir.

`scripts/batch_test.sh` her senaryo icin bir `.log` ve bir `.csv` birakir.
Bu script onlari okuyup tek bir tabloya cevirir -- Robotaksi simulasyon
raporunun "8.2 Test Sonuclari" bolumune dogrudan girecek bicimde.

Kullanim:
    python3 scripts/gazebo_report.py                      # en son kosu
    python3 scripts/gazebo_report.py results/20261116_1012
    python3 scripts/gazebo_report.py --out docs/test-sonuclari-gazebo.md
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "ros2_ws" / "src" / "mergen_avoidance"
sys.path.insert(0, str(PKG))

try:
    from mergen_avoidance.sim2d import SCENARIOS  # noqa: E402
except Exception:                                  # noqa: BLE001
    SCENARIOS = []

DESCRIPTIONS = {
    "w1_empty": "Boş alan — taban çizgisi",
    "w2_cone_slalom": "Koni slalomu — art arda yön değiştirme",
    "w3_corridor": "Koridor + variller — şerit disiplini",
    "w4_lane_block": "Şeridi kısmen kapatan statik engel",
    "w5_dead_end": "Kısa çıkmaz koridor",
    "w6_dense_clutter": "Yoğun koniler — sürekli yeniden planlama",
    "w7_trap_pocket": "Kapalı cep — bilinen sınır (yerel minimum)",
    "w8_pedestrian": "Aniden karşıya geçen yaya — dinamik engel",
    "w9_parkur": "20 m koridor, 10 engel, dar geçit — tam parkur",
}

# ANSI renk kodlarini temizlemek icin (ROS loglari renkli basiyor)
ANSI = re.compile(r"\x1b\[[0-9;]*m")

FIELDS = {
    "duration": re.compile(r"sure\s*:\s*([\d.]+)\s*s"),
    "distance": re.compile(r"kat edilen yol\s*:\s*([\d.]+)\s*m"),
    "avg_speed": re.compile(r"ortalama hiz\s*:\s*([\d.]+)"),
    "min_clear": re.compile(r"en yakin yaklasma\s*:\s*([\d.naN]+)\s*m"),
    "contacts": re.compile(r"temas sayisi\s*:\s*(\d+)"),
}
STATE_LINE = re.compile(r"^\s*([A-Z_?]+)\s+([\d.]+)\s*s\s*\(\s*([\d.]+)%\)")


def _rel(path: Path) -> str:
    """Repo koku altindaysa goreli, degilse mutlak yol dondurur."""
    try:
        return str(path.resolve().relative_to(REPO))
    except ValueError:
        return str(path)


def latest_run_dir() -> Path | None:
    results = REPO / "results"
    if not results.is_dir():
        return None
    runs = [d for d in results.iterdir() if d.is_dir() and d.name[0].isdigit()]
    return max(runs, key=lambda d: d.name) if runs else None


def parse_log(path: Path) -> dict:
    text = ANSI.sub("", path.read_text(encoding="utf8", errors="replace"))
    out: dict = {"name": path.stem, "states": {}, "ok": False}

    if "KOSU OZETI" not in text:
        out["error"] = "özet bulunamadı (koşu erken bitmiş olabilir)"
        return out

    block = text.split("KOSU OZETI", 1)[1]
    for key, pattern in FIELDS.items():
        m = pattern.search(block)
        if m:
            raw = m.group(1)
            out[key] = float("nan") if raw.lower().startswith("na") else float(raw)

    for line in block.splitlines():
        m = STATE_LINE.match(line.replace("[metrics_logger-5]", ""))
        if m:
            out["states"][m.group(1)] = float(m.group(3))

    # Saglik kontrolleri: kosu gercekten temiz mi bitti?
    out["tracebacks"] = text.count("Traceback (most recent call last)")
    out["pedestrian_fired"] = "YAYA YOLA CIKTI" in text
    out["ok"] = "contacts" in out
    return out


def csv_endpoint(run_dir: Path, scenario: str) -> tuple[float, float] | None:
    """Kosu sonundaki (x, y) -- ne kadar ilerledigini gosterir."""
    matches = sorted(run_dir.glob(f"{scenario}_*.csv"))
    if not matches:
        return None
    try:
        with matches[-1].open(encoding="utf8") as fh:
            rows = list(csv.DictReader(fh))
        if not rows:
            return None
        return float(rows[-1]["x"]), float(rows[-1]["y"])
    except Exception:                               # noqa: BLE001
        return None


# Bir senaryonun "gecerli" sayilmasi icin engelin araci GERCEKTEN etkilemis
# olmasi gerekir. Aksi halde "carpisma yok" bos bir iddiadir: engeli hic
# gormeyen bir arac da carpmaz.
#
# Bu kontrol gercek bir yanlis-pozitifi yakalamak icin eklendi: w8_pedestrian
# kosusunda yaya tetiklendi, "temas 0" yazdi ve tablo "BASARILI" dedi -- ama
# arac tum kosu boyunca CRUISE'da kaldi, cunku Gazebo'daki yaya modeli hic
# kipirdamamisti (ODE uyku modu). Olcum, olcmek istedigi seyi olcmuyordu.
ENGAGEMENT = {
    # senaryo: (etkilesim sayilan durumlar, beklenen en fazla 'en yakin' [m])
    "w2_cone_slalom": ({"AVOID", "SLOW"}, 1.0),
    "w3_corridor": ({"AVOID", "SLOW"}, 1.0),
    "w4_lane_block": ({"AVOID", "SLOW"}, 1.2),
    "w5_dead_end": ({"AVOID", "SLOW", "BLOCKED"}, 1.0),
    "w6_dense_clutter": ({"AVOID", "SLOW"}, 1.0),
    "w7_trap_pocket": ({"BLOCKED", "RECOVER"}, 1.5),
    "w8_pedestrian": ({"AVOID", "SLOW", "BLOCKED", "EMERGENCY"}, 1.2),
    "w9_parkur": ({"AVOID", "SLOW"}, 1.0),
}


def engagement_problem(r: dict) -> str | None:
    """Engel araci gercekten etkiledi mi? Etkilemediyse sebebini dondurur."""
    spec = ENGAGEMENT.get(r["name"])
    if spec is None:
        return None                       # w1_empty: engel yok, beklenen
    expected_states, max_clearance = spec

    reacted = any(r["states"].get(st, 0.0) > 0.5 for st in expected_states)
    mc = r.get("min_clear", float("nan"))
    came_close = isinstance(mc, float) and not math.isnan(mc) and mc <= max_clearance

    if not reacted and not came_close:
        return (f"arac hic tepki vermedi (beklenen durumlar: "
                f"{'/'.join(sorted(expected_states))}) ve engel "
                f"{max_clearance:.1f} m'den yakina hic gelmedi")
    return None


def verdict(r: dict) -> str:
    if not r.get("ok"):
        return "ÖLÇÜLEMEDİ"
    if r.get("contacts", 0) > 0:
        return "BAŞARISIZ (temas)"
    if engagement_problem(r):
        return "GEÇERSİZ (engel etkilemedi)"
    if r["name"] == "w7_trap_pocket":
        saw_trap = bool({"BLOCKED", "RECOVER"} & set(r["states"]))
        return "BEKLENEN (kapan farkedildi)" if saw_trap else "BAŞARISIZ (kapanı görmedi)"
    return "BAŞARILI"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", nargs="?", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    run_dir = args.run_dir or latest_run_dir()
    if run_dir is not None:
        # Goreli yol verilmis olabilir; rapor basliginda repo'ya gore
        # gosterebilmek icin mutlaklastiriyoruz.
        run_dir = run_dir.resolve()
    if run_dir is None or not run_dir.is_dir():
        print("HATA: koşu dizini bulunamadı. Önce: bash scripts/batch_test.sh",
              file=sys.stderr)
        return 1

    logs = sorted(run_dir.glob("*.log"))
    if not logs:
        print(f"HATA: {run_dir} içinde .log yok", file=sys.stderr)
        return 1

    rows = [parse_log(p) for p in logs]

    md = [
        "# Gazebo Test Sonuçları",
        "",
        f"Koşu: `{_rel(run_dir)}` · senaryo başına 60 s · "
        "TurtleBot3 Waffle · Gazebo Classic 11 · ROS 2 Humble",
        "",
        "Bu tablo `bash scripts/batch_test.sh` çıktısından",
        "`python3 scripts/gazebo_report.py` ile üretildi. Gerçek fizik motoru,",
        "gerçek lidar gürültüsü, gerçek diferansiyel sürüş eklentisi.",
        "",
        "| Senaryo | Ne test ediyor | Sonuç | Temas | En yakın | Yol | Ort. hız | Süre | Baskın durumlar |",
        "|---|---|---|---|---|---|---|---|---|",
    ]

    clean = problems = 0
    for r in rows:
        name = r["name"]
        if not r.get("ok"):
            md.append(f"| `{name}` | {DESCRIPTIONS.get(name, '')} | "
                      f"**ÖLÇÜLEMEDİ** | — | — | — | — | — | {r.get('error', '')} |")
            problems += 1
            continue

        states = ", ".join(f"{k} %{v:.0f}" for k, v in
                           sorted(r["states"].items(), key=lambda kv: -kv[1])[:3])
        mc = r.get("min_clear", float("nan"))
        mc_txt = "—" if (isinstance(mc, float) and math.isnan(mc)) else f"{mc:.2f} m"
        v = verdict(r)
        if v.startswith(("BAŞARILI", "BEKLENEN")):
            clean += 1
        else:
            problems += 1

        md.append(
            f"| `{name}` | {DESCRIPTIONS.get(name, '')} | **{v}** | "
            f"{int(r.get('contacts', 0))} | {mc_txt} | {r.get('distance', 0):.2f} m | "
            f"{r.get('avg_speed', 0):.2f} m/s | {r.get('duration', 0):.1f} s | {states} |"
        )

    md += [
        "",
        f"**Özet:** {len(rows)} senaryo, geçerli ve temiz: {clean}, sorunlu: {problems}.",
        "",
    ]

    invalid = [(r["name"], engagement_problem(r)) for r in rows
               if r.get("ok") and engagement_problem(r)]
    if invalid:
        md += ["> **Geçersiz ölçümler** — 'çarpışma yok' burada bir şey kanıtlamıyor,",
               "> çünkü engel aracı hiç etkilememiş:", ""]
        md += [f"> - `{name}`: {why}" for name, why in invalid]
        md.append("")

    ped = next((r for r in rows if r["name"] == "w8_pedestrian"), None)
    if ped:
        md.append("**Dinamik engel:** yaya tetiklendi mi — "
                  f"{'evet' if ped.get('pedestrian_fired') else 'HAYIR (incelenmeli)'}.")
        md.append("")

    tb = sum(r.get("tracebacks", 0) for r in rows)
    md.append(f"**Kapanış sağlığı:** toplam traceback sayısı {tb} "
              f"({'temiz' if tb == 0 else 'incelenmeli'}).")
    md += [
        "",
        "Notlar:",
        "",
        "- 'En yakın' robot gövdesinden engel yüzeyine ölçülen en küçük mesafedir",
        "  (ham lidar taramasından, `robust_k` filtresi uygulanmadan — ölçüm ile",
        "  karar aynı filtreyi paylaşmamalı).",
        "- `w7_trap_pocket` kasıtlı olarak çözülemez: salt tepkisel bir yerel",
        "  planlayıcı yerel minimumdan kendi başına çıkamaz. Beklenti çarpmamak",
        "  ve kapanı fark etmek.",
        "- Süreler 60 s'lik üst sınıra takılmış olabilir; 'BAŞARILI' burada",
        "  'temassız ve düzgün ilerledi' demektir, '9 m'yi tamamladı' demek değil.",
        "  2B tablosundaki (`docs/test-sonuclari.md`) hedefe varma ölçütüyle",
        "  karıştırma.",
    ]

    text = "\n".join(md) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf8")
        print(f"Markdown yazıldı: {args.out}")
    else:
        print(text)

    for r in rows:
        extra = ""
        if r.get("tracebacks"):
            extra = f"  [{r['tracebacks']} traceback]"
        contacts = r.get("contacts")
        print(f"  {r['name']:18s} {verdict(r):28s}"
              f" temas={int(contacts) if contacts is not None else '?'}{extra}",
              file=sys.stderr)

    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
