#!/usr/bin/env python3
"""pytest olmadan test kosucusu.

Neden var: ROS kurulmadan once (ya da pytest yuklu olmayan bir makinede)
cekirdek mantigi dogrulayabilmek icin. pytest varsa onu kullan; bu sadece
bagimlilik gerektirmeyen yedek yol.

    python3 test/run_tests.py
    python3 test/run_tests.py -v
"""
from __future__ import annotations

import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def main() -> int:
    verbose = "-v" in sys.argv
    import test_algorithm as mod

    tests = sorted(n for n in dir(mod) if n.startswith("test_"))
    passed, failed = 0, []
    for name in tests:
        try:
            getattr(mod, name)()
        except Exception as exc:                       # noqa: BLE001 -- kosucu
            failed.append((name, exc, traceback.format_exc()))
            print(f"  BASARISIZ  {name}")
            if verbose:
                print(failed[-1][2])
        else:
            passed += 1
            if verbose:
                print(f"  tamam      {name}")

    print(f"\n{passed}/{len(tests)} test gecti")
    for name, exc, tb in failed:
        print(f"\n--- {name} ---")
        print(tb if verbose else f"{type(exc).__name__}: {exc}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
