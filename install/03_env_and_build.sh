#!/usr/bin/env bash
# =============================================================================
# 03 - Ortam degiskenleri (.bashrc) + colcon derlemesi
# =============================================================================
# Kullanim:  bash install/03_env_and_build.sh      (sudo YOK)
# =============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO/ros2_ws"
MARK_BEGIN="# >>> mergen-01 robotaksi ortami >>>"
MARK_END="# <<< mergen-01 robotaksi ortami <<<"

if [[ ! -d /opt/ros/humble ]]; then
  echo "HATA: ROS 2 Humble yok. Once 01 ve 02 scriptlerini calistir." >&2
  exit 1
fi

echo "==> .bashrc ortami yaziliyor"
# Idempotent: eski blogu sil, yenisini ekle.
if grep -qF "$MARK_BEGIN" "$HOME/.bashrc" 2>/dev/null; then
  sed -i "/$(printf '%s' "$MARK_BEGIN" | sed 's/[][\\.*^$/]/\\&/g')/,/$(printf '%s' "$MARK_END" | sed 's/[][\\.*^$/]/\\&/g')/d" "$HOME/.bashrc"
fi
cat >> "$HOME/.bashrc" <<BASHRC
$MARK_BEGIN
source /opt/ros/humble/setup.bash
# Yarisma araci burger'dan buyuk; 'waffle' daha gercekci (daha genis, daha agir).
export TURTLEBOT3_MODEL=\${TURTLEBOT3_MODEL:-waffle}
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:$WS/src/mergen_avoidance/models
# Ayni agdaki diger ROS makinelerinin trafigini karistirmamak icin:
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=1
# Calisma alani (derlendikten sonra olusur)
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
$MARK_END
BASHRC

echo "==> rosdep update"
rosdep update || echo "UYARI: rosdep update basarisiz (offline olabilir)"

echo "==> colcon build"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install --from-paths src --ignore-src -r -y || echo "UYARI: rosdep install eksik kaldi"
colcon build --symlink-install

echo
echo "BITTI."
echo "Yeni bir terminal ac (veya: source ~/.bashrc) ve dene:"
echo "  ros2 launch mergen_avoidance avoid.launch.py world:=w2_cone_slalom"
