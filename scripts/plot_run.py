#!/usr/bin/env python3
"""Gazebo kosusundan (CSV) rapor grafikleri uretir.

Robotaksi simulasyon raporu sablonu, "8.2 Test Sonuclari" altinda aynen
su grafikleri istiyor: "Yorunge, hiz profili, yonelim hatasi, engel algilama".
Bu script onlari dogrudan uretir.

Kullanim:
    python3 scripts/plot_run.py ~/mergen_results/w2_cone_slalom_*.csv
    python3 scripts/plot_run.py results/20261115_*/ *.csv --out-dir docs/figures
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")           # pencere yok -> sunucuda/headless calisir
    import matplotlib.pyplot as plt
except ImportError:
    print("HATA: matplotlib yok.  sudo apt install python3-matplotlib", file=sys.stderr)
    sys.exit(1)

# Durum -> renk. avoider_node.py'deki RViz renkleriyle AYNI tutuluyor;
# boylece RViz'de gordugun renk grafikte de ayni anlama geliyor.
STATE_COLOURS = {
    "CRUISE": "#1ae533",
    "SLOW": "#e5e51a",
    "AVOID": "#ff8000",
    "BLOCKED": "#ff1a1a",
    "EMERGENCY": "#ff0099",
    "RECOVER": "#9933ff",
    "NO_SCAN": "#777777",
    "EMPTY_SCAN": "#777777",
}


def read_csv(path: Path) -> dict[str, list]:
    with path.open(encoding="utf8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"{path}: bos dosya")

    out: dict[str, list] = {k: [] for k in rows[0]}
    for row in rows:
        for key, value in row.items():
            if key == "state":
                out[key].append(value)
            else:
                try:
                    out[key].append(float(value))
                except (TypeError, ValueError):
                    out[key].append(math.nan)
    return out


def shade_states(ax, t: list[float], states: list[str]) -> None:
    """Durum degisimlerini arka plan rengi olarak cizer -- grafigi okunur yapar."""
    if not states:
        return
    start = 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            colour = STATE_COLOURS.get(states[start], "#cccccc")
            ax.axvspan(t[start], t[min(i, len(t) - 1)], color=colour, alpha=0.18, linewidth=0)
            start = i


def plot_run(path: Path, out_dir: Path) -> Path:
    d = read_csv(path)
    t = d["t"]
    states = d["state"]
    name = path.stem

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(f"mergen-01 kosu raporu -- {name}", fontsize=13, fontweight="bold")

    # --- 1) Yorunge ---
    ax = axes[0][0]
    ax.plot(d["x"], d["y"], color="#222222", linewidth=1.2, zorder=3)
    # Durum renkleriyle noktalar
    for state, colour in STATE_COLOURS.items():
        xs = [x for x, s in zip(d["x"], states) if s == state]
        ys = [y for y, s in zip(d["y"], states) if s == state]
        if xs:
            ax.scatter(xs, ys, s=7, color=colour, label=state, zorder=4)
    ax.scatter([d["x"][0]], [d["y"][0]], marker="o", s=90, facecolor="white",
               edgecolor="black", zorder=5, label="baslangic")
    ax.scatter([d["x"][-1]], [d["y"][-1]], marker="X", s=90, color="black",
               zorder=5, label="bitis")
    ax.set_title("Yorunge (ustten gorunum)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.axis("equal"); ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="best", ncol=2)

    # --- 2) Hiz profili ---
    ax = axes[0][1]
    shade_states(ax, t, states)
    ax.plot(t, d["v_cmd"], label="v komut [m/s]", color="#0066cc", linewidth=1.3)
    ax.plot(t, d["v_odom"], label="v olculen [m/s]", color="#cc0000",
            linewidth=1.0, alpha=0.75)
    ax.set_title("Hiz profili (komut vs olculen)")
    ax.set_xlabel("t [s]"); ax.set_ylabel("m/s")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # --- 3) Engel mesafeleri ---
    ax = axes[1][0]
    shade_states(ax, t, states)
    ax.plot(t, d["front"], label="on koni", color="#0066cc", linewidth=1.3)
    ax.plot(t, d["nearest"], label="en yakin nokta (her yon)", color="#cc6600",
            linewidth=1.1)
    ax.axhline(0.45, color="#ff8800", linestyle="--", linewidth=1,
               label="stop_distance 0.45 m")
    ax.axhline(0.25, color="#cc0000", linestyle="--", linewidth=1,
               label="emergency_distance 0.25 m")
    ax.set_title("Engel mesafeleri")
    ax.set_xlabel("t [s]"); ax.set_ylabel("m")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # --- 4) Yonelim: secilen yon vs uygulanan donus ---
    ax = axes[1][1]
    shade_states(ax, t, states)
    if any(not math.isnan(v) for v in d.get("chosen_heading", [])):
        ax.plot(t, [math.degrees(v) for v in d["chosen_heading"]],
                label="secilen yon [derece]", color="#009933", linewidth=1.3)
    ax.plot(t, [math.degrees(v) for v in d["w_cmd"]],
            label="w komut [derece/s]", color="#9933ff", linewidth=1.0, alpha=0.8)
    ax.set_title("Yonelim karari")
    ax.set_xlabel("t [s]"); ax.set_ylabel("derece | derece/s")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{name}.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", type=Path, help="metrics_logger CSV dosyalari")
    ap.add_argument("--out-dir", type=Path, default=Path("results/figures"))
    args = ap.parse_args()

    made = 0
    for path in args.csv:
        if not path.is_file():
            print(f"  atlandi (dosya yok): {path}")
            continue
        try:
            out = plot_run(path, args.out_dir)
        except Exception as exc:                      # noqa: BLE001
            print(f"  HATA {path.name}: {exc}")
            continue
        print(f"  {out}")
        made += 1

    print(f"\n{made} grafik uretildi -> {args.out_dir}")
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
