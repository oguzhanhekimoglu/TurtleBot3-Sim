#!/usr/bin/env bash
# Tek senaryoyu Gazebo'da calistirir.
#
#   bash scripts/run_scenario.sh w8_pedestrian viz    # ONERILEN: sadece RViz
#   bash scripts/run_scenario.sh                      # varsayilan: slalom, Gazebo penceresi
#   bash scripts/run_scenario.sh w4_lane_block        # baska dunya
#   bash scripts/run_scenario.sh w8_pedestrian rviz   # Gazebo + RViz (en agir)
#   bash scripts/run_scenario.sh w6_dense_clutter headless
#
# Modlar:
#   gui       Gazebo penceresi (3B fizik gorunumu).  AGIR.
#   rviz      Gazebo + RViz birlikte.                EN AGIR.
#   viz       Sadece RViz.                           HAFIF -- bunu kullan.
#   headless  Hic pencere yok.                       Toplu test icin.
#
# Nicin "viz" onerilir: Gazebo'nun 3B penceresi SADECE goruntuleme icindir,
# simulasyon onsuz da calisir. Robotu, lidar isinlarini ve karar gorsellerini
# zaten RViz gosteriyor ve RViz cok daha hafif. Zayif/entegre ekran kartinda
# fark buyuktur.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-w2_cone_slalom}"
MODE="${2:-gui}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "HATA: ROS 2 Humble yok. Once: sudo bash install/01_ros2_humble.sh" >&2
  exit 1
fi
if [[ ! -f "$REPO/ros2_ws/install/setup.bash" ]]; then
  echo "HATA: calisma alani derlenmemis. Once: bash install/03_env_and_build.sh" >&2
  exit 1
fi

WORLD_FILE="$REPO/ros2_ws/src/mergen_avoidance/worlds/${WORLD}.world"
if [[ ! -f "$WORLD_FILE" ]]; then
  echo "HATA: dunya bulunamadi: $WORLD_FILE" >&2
  echo "Mevcut dunyalar:" >&2
  ls -1 "$REPO/ros2_ws/src/mergen_avoidance/worlds/" | sed 's/\.world$//; s/^/  /' >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/opt/ros/humble/share/turtlebot3_gazebo/models"

GUI=true; RVIZ=false
case "$MODE" in
  gui)      GUI=true;  RVIZ=false ;;   # sadece Gazebo penceresi
  rviz)     GUI=true;  RVIZ=true  ;;   # Gazebo + RViz (en agir)
  viz)      GUI=false; RVIZ=true  ;;   # SADECE RViz -- onerilen
  headless) GUI=false; RVIZ=false ;;   # pencere yok (toplu test)
  *) echo "HATA: mod 'gui' | 'rviz' | 'viz' | 'headless' olmali (verilen: $MODE)" >&2
     exit 1 ;;
esac

# --- ASILI KALAN GAZEBO KONTROLU ---------------------------------------
# Gazebo, Ctrl-C'den sonra sik sik arkada gzserver birakir. O surec 11345
# portunu tuttugu icin bir sonraki kosu "Unable to start server
# [bind: Address already in use]" ile ANINDA olur -- ama hata mesaji
# launch ciktisinin ortasinda kaybolur ve "simulasyon calismiyor" gibi
# gorunur. Bir kez 9 dakika boyunca her kosuyu bozdu.
#
# Not: `pkill -f gzserver` KULLANMA. `-f` tum komut satirinda arar ve bu
# scriptin kendi komut satirinda da "gzserver" gectigi icin KENDINI oldurur.
if pgrep -x gzserver > /dev/null 2>&1; then
  echo "UYARI: arkada calisan bir gzserver var (onceki kosudan kalmis olabilir)."
  ps -eo pid,etime,comm | grep -E '^ *[0-9]+ .* gz(server|client)$' | sed 's/^/   /'

  # Terminal YOKSA soru sorma. Dosya yoneticisinden "Run as a program" ile
  # calistirildiginda stdin bir terminal degildir; `read` orada sessizce
  # bekler ve script hic acilmamis gibi gorunur. O durumda dogrudan temizle.
  if [ -t 0 ]; then
    read -r -p "Kapatilsin mi? [E/h] " answer
  else
    echo "(terminal yok -- otomatik temizleniyor)"
    answer=E
  fi

  case "${answer:-E}" in
    [Hh]*) echo "Devam ediliyor -- kosu muhtemelen basarisiz olacak." ;;
    *)     pkill -x gzserver; pkill -x gzclient; sleep 2
           pkill -9 -x gzserver 2>/dev/null || true
           echo "Temizlendi." ;;
  esac
  echo
fi

# --- NVIDIA PRIME OFFLOAD ------------------------------------------------
# Bu makine hibrit: Intel iGPU + NVIDIA dGPU, prime-select "on-demand".
# Varsayilan olarak HER SEY Intel'de render edilir ve Gazebo takilir.
# NVIDIA surucusu calisiyorsa bu iki degisken render'i NVIDIA'ya yonlendirir.
#
# Surucu calismiyorsa (ornegin Secure Boot MOK anahtari kayitli degilse)
# degiskenleri ayarlamak ZARARLIDIR: uygulama siyah pencere acar veya coker.
# O yuzden once nvidia-smi ile gercekten calisip calismadigini SINIYORUZ.
if nvidia-smi -L > /dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __VK_LAYER_NV_optimus=NVIDIA_only
  GPU_BILGI="NVIDIA ($(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1))"
else
  GPU_BILGI="Intel iGPU  <-- NVIDIA surucusu calismiyor, performans dusuk"
fi

echo "=============================================="
echo " dunya  : $WORLD"
echo " GPU    : $GPU_BILGI"
echo " model  : $TURTLEBOT3_MODEL"
echo " mod    : $MODE (gui=$GUI rviz=$RVIZ)"
echo " cikti  : ~/mergen_results/"
echo "=============================================="
echo "Durdurmak icin Ctrl-C."
echo

exec ros2 launch mergen_avoidance avoid.launch.py \
  world:="$WORLD" gui:="$GUI" rviz:="$RVIZ" metrics:=true
