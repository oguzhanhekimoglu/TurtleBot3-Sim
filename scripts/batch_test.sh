#!/usr/bin/env bash
# TUM dunyalari sirayla, penceresiz (headless) kosturur ve ozet tablo uretir.
#
# Nicin onemli: Robotaksi simulasyon raporu "farkli engel yerlesimleriyle test"
# ve "test sonuclari tablosu" istiyor. Bunu elle 8 kez yapmak yerine tek komut.
#
#   bash scripts/batch_test.sh            # her senaryo 60 s
#   bash scripts/batch_test.sh 90         # her senaryo 90 s
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DURATION="${1:-60}"
OUT_DIR="$REPO/results/$(date +%Y%m%d_%H%M%S)"

if [[ ! -f "$REPO/ros2_ws/install/setup.bash" ]]; then
  echo "HATA: calisma alani derlenmemis. Once: bash install/03_env_and_build.sh" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models"

WORLDS=()
while IFS= read -r w; do WORLDS+=("$w"); done < <(
  ls -1 "$REPO/ros2_ws/src/mergen_avoidance/worlds/"*.world | xargs -n1 basename | sed 's/\.world$//'
)

# BASLAMADAN ONCE temizle. Gazebo, `ros2 launch` oldurulse bile arkada
# gzserver birakabiliyor; o surec 11345 portunu tuttugu icin sonraki TUM
# kosular "Unable to start server [bind: Address already in use]" ile
# aninda olur ve bos CSV uretir. Bir kez 8 senaryodan birini sessizce
# gecersiz kildi.
if pgrep -x gzserver > /dev/null 2>&1; then
  echo "Onceki kosudan kalan gzserver temizleniyor..."
  pkill -x gzserver || true
  pkill -x gzclient || true
  sleep 2
  pkill -9 -x gzserver 2>/dev/null || true
  sleep 1
fi

echo "=============================================================="
echo " TOPLU TEST | ${#WORLDS[@]} senaryo x ${DURATION} s | cikti: $OUT_DIR"
echo "=============================================================="

for w in "${WORLDS[@]}"; do
  echo
  echo "--- $w ---"
  # Gazebo'yu kapanirken bazen asili kalir; timeout + pkill ile garanti altina aliyoruz.
  timeout --signal=INT --kill-after=15 "$DURATION" \
    ros2 launch mergen_avoidance avoid.launch.py \
      world:="$w" gui:=false rviz:=false metrics:=true \
      output_dir:="$OUT_DIR" > "$OUT_DIR/${w}.log" 2>&1 || true

  # Artik surecleri temizle, yoksa sonraki senaryo kirli ortamda baslar.
  #
  # DIKKAT: `-f` DEGIL `-x` kullaniyoruz. `pkill -f gzserver` tum komut
  # satirlarinda arama yapar ve kendi kabugunun komut satirinda da "gzserver"
  # gecerse KENDINI OLDURUR. Bu gercekten yasandi: terminalden
  # `... pkill -f gzserver ...` iceren bir komut calistirinca komut aninda
  # olduruldu ve hicbir cikti uretmedi. `-x` tam surec adiyla eslesir.
  pkill -x gzserver 2>/dev/null || true
  pkill -x gzclient 2>/dev/null || true
  pkill -f "[m]ergen_avoidance" 2>/dev/null || true
  sleep 3
  # Inatci olanlari zorla kapat; yoksa SONRAKI senaryo bos CSV uretir.
  pkill -9 -x gzserver 2>/dev/null || true
  sleep 1

  # Ozeti logdan cek
  if grep -q "KOSU OZETI" "$OUT_DIR/${w}.log" 2>/dev/null; then
    sed -n '/KOSU OZETI/,/^=\{40,\}$/p' "$OUT_DIR/${w}.log" | sed 's/^/    /' | head -20
  else
    echo "    (ozet bulunamadi -- $OUT_DIR/${w}.log dosyasina bak)"
  fi
done

echo
echo "=============================================================="
echo " Bitti. CSV ve loglar: $OUT_DIR"
echo " Grafikler icin:  python3 scripts/plot_run.py $OUT_DIR/*.csv"
echo "=============================================================="
