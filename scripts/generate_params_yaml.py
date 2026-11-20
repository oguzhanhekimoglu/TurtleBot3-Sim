#!/usr/bin/env python3
"""config/avoider.yaml dosyasini AvoidParams'tan uretir.

Neden uretim: parametre listesi iki yerde (dataclass + YAML) elle tutulursa
kacinilmaz olarak birbirinden uzaklasir -- YAML'da olmayan bir parametre
sessizce varsayilanda kalir ve "ayari degistirdim ama hicbir sey olmadi"
saatleri uretir. Burada YAML bir TUREVDIR, kaynak degil.

Her parametrenin aciklamasi da params.py'deki docstring'den gelir; yani
dokumantasyon tek yerde yazilir.

Kullanim:  python3 scripts/generate_params_yaml.py
"""
from __future__ import annotations

import re
import sys
from dataclasses import fields
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "ros2_ws" / "src" / "mergen_avoidance"
sys.path.insert(0, str(PKG))

from mergen_avoidance.params import AvoidParams  # noqa: E402

NODE_NAME = "mergen_avoider"
WRAP = 86

# Dugume ozel, algoritmaya ait olmayan parametreler (avoider_node.py'de tanimli)
NODE_PARAMS = [
    ("world_goal_heading", "0.0",
     "Engel yoksa gidilecek DUNYA cercevesi yonu [rad]. 0 = +x (dogu). "
     "Gercek sistemde bunu GEOJSON rotasindan uretilen global planlayici doldurur."),
    ("use_odom_for_goal", "true",
     "Hedef yonu odometriden govde cercevesine cevir. "
     "false ise planlayici 'burnunun dogrultusuna git' moduna duser."),
    ("scan_timeout", "0.5",
     "Lidar bu kadar sure susarsa araci durdur [s]. Eski taramayla surmeye "
     "devam etmek otonom bir aracta en tehlikeli hatadir."),
    ("publish_markers", "true", "RViz karar gorsellerini yayinla."),
    ("log_period", "1.0", "Konsola durum basma periyodu [s]."),
]


def field_docs() -> dict[str, str]:
    """params.py icindeki alan docstring'lerini cikarir."""
    src = (PKG / "mergen_avoidance" / "params.py").read_text(encoding="utf8")
    pattern = re.compile(r'^    (\w+): [^=]+= [^\n]+\n    """(.*?)"""', re.S | re.M)
    return {m.group(1): " ".join(m.group(2).split()) for m in pattern.finditer(src)}


def wrap_comment(text: str) -> list[str]:
    """Uzun aciklamayi kelime siniriyla sarar (ortasindan kesmez)."""
    out, line = [], ""
    for word in text.split():
        if line and len(line) + 1 + len(word) > WRAP:
            out.append(f"    # {line}")
            line = word
        else:
            line = f"{line} {word}".strip()
    if line:
        out.append(f"    # {line}")
    return out


def main() -> int:
    docs = field_docs()
    defaults = AvoidParams()

    lines = [
        "# =============================================================================",
        "# mergen_avoidance -- sakinma dugumu ayarlari",
        "# =============================================================================",
        "# OTOMATIK URETILDI -- elle duzenleme kalici olmaz.",
        "#   kaynak : ros2_ws/src/mergen_avoidance/mergen_avoidance/params.py",
        "#   uretim : python3 scripts/generate_params_yaml.py",
        "#",
        "# Calisirken denemek icin (yeniden baslatmadan):",
        "#   ros2 param set /mergen_avoider max_linear_speed 0.15",
        "#   ros2 param get /mergen_avoider w_clearance",
        "# Kalici degisiklik icin params.py'deki varsayilani degistir ve bu scripti kostur.",
        "# =============================================================================",
        f"{NODE_NAME}:",
        "  ros__parameters:",
        "",
    ]

    for f in fields(AvoidParams):
        if f.name in docs:
            lines += wrap_comment(docs[f.name])
        value = getattr(defaults, f.name)
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        else:
            rendered = str(value)
        lines.append(f"    {f.name}: {rendered}")
        lines.append("")

    lines.append("    # ---- dugume ozel (algoritma disi) ----")
    lines.append("")
    for name, value, doc in NODE_PARAMS:
        lines += wrap_comment(doc)
        lines.append(f"    {name}: {value}")
        lines.append("")

    out_path = PKG / "config" / "avoider.yaml"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf8")

    n = len(fields(AvoidParams)) + len(NODE_PARAMS)
    print(f"{out_path.relative_to(REPO)} yazildi ({n} parametre)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
