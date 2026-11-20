#!/usr/bin/env bash
# =============================================================================
# 01 - ROS 2 Humble + Gazebo Classic 11 kurulumu (Ubuntu 22.04 "jammy")
# =============================================================================
# Kullanim:  sudo bash install/01_ros2_humble.sh
#
# Neden Humble?  Yarisma araci (Hazir Arac Mimari Tanimlama Dok. 2026) uzerinde
# Ubuntu 20.04 + ROS2 Foxy var. Foxy'nin Ubuntu 22.04 icin binary paketi YOK.
# Humble, Foxy ile API olarak neredeyse birebir ayni (rclpy tarafinda farkli olan
# tek sey QoS/lifecycle detaylari). Bu depoda yazdigimiz kod ikisinde de derlenir.
# =============================================================================
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "HATA: bu script root ister ->  sudo bash $0" >&2
  exit 1
fi

CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
if [[ "$CODENAME" != "jammy" ]]; then
  echo "UYARI: bu script jammy (22.04) icin yazildi, sistem: $CODENAME" >&2
fi

echo "==> [1/6] locale (ROS 2 UTF-8 ister)"
apt-get update -qq
apt-get install -y -qq locales curl gnupg lsb-release software-properties-common
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "==> [2/6] universe deposu"
add-apt-repository -y universe >/dev/null

echo "==> [3/6] ROS 2 apt anahtari + deposu"
install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${CODENAME} main" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq

echo "==> [4/6] ros-humble-desktop (~2.5 GB, uzun surer)"
apt-get install -y ros-humble-desktop

echo "==> [5/6] gelistirme araclari + Gazebo Classic 11 koprusu"
apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool python3-argcomplete \
  ros-dev-tools \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-rqt-common-plugins \
  ros-humble-tf2-tools \
  ros-humble-rmw-cyclonedds-cpp

echo "==> [6/6] rosdep init"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

echo
echo "BITTI. Dogrulama (normal kullanici olarak):"
echo "  source /opt/ros/humble/setup.bash && ros2 doctor --report | head -20"
echo "Sonraki adim:  sudo bash install/02_turtlebot3_sim.sh"
