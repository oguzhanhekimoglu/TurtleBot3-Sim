#!/usr/bin/env bash
# =============================================================================
# 02 - TurtleBot3 + Gazebo simulasyon paketleri
# =============================================================================
# Kullanim:  sudo bash install/02_turtlebot3_sim.sh
# Not: ros-humble-turtlebot3-gazebo binary olarak mevcut, kaynaktan derlemeye
#      gerek yok (packages.ros.org/ros2/ubuntu jammy icin dogrulandi).
# =============================================================================
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "HATA: bu script root ister ->  sudo bash $0" >&2
  exit 1
fi

if [[ ! -d /opt/ros/humble ]]; then
  echo "HATA: ROS 2 Humble bulunamadi. Once: sudo bash install/01_ros2_humble.sh" >&2
  exit 1
fi

echo "==> TurtleBot3 paketleri"
apt-get update -qq
apt-get install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-description \
  ros-humble-turtlebot3-bringup \
  ros-humble-turtlebot3-teleop \
  ros-humble-turtlebot3-simulations \
  ros-humble-turtlebot3-gazebo

echo "==> Opsiyonel: Nav2 (ileride global planlama / kostmap icin)"
apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup || echo "UYARI: nav2 kurulamadi, simdilik sorun degil"

echo "==> Python yardimci paketler (rapor grafikleri icin)"
apt-get install -y python3-matplotlib python3-numpy python3-pandas

echo
echo "BITTI. Sonraki adim (root DEGIL, normal kullanici):"
echo "  bash install/03_env_and_build.sh"
