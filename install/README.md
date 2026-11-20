# Kurulum

Sirayla, **tek tek** calistir. 1. adim ~2.5 GB indirir, internet hizina gore 10-40 dk surer.

```bash
sudo bash install/01_ros2_humble.sh      # ROS 2 Humble + Gazebo Classic 11
sudo bash install/02_turtlebot3_sim.sh   # TurtleBot3 + (ops.) Nav2
bash      install/03_env_and_build.sh    # .bashrc + colcon build   <-- sudo YOK
```

Sonra **yeni bir terminal** ac.

## Dogrulama

```bash
ros2 doctor --report | head -20
echo $TURTLEBOT3_MODEL              # waffle
ros2 pkg list | grep turtlebot3     # paketler gorunmeli
ros2 pkg list | grep mergen         # mergen_avoidance gorunmeli
```

## Bu makineye ozel not: hibrit ekran karti (Intel + NVIDIA)

`lspci` cikisi Intel iGPU + NVIDIA dGPU gosteriyor. Gazebo varsayilan olarak Intel
tarafinda calisir; akici olmazsa NVIDIA'ya yonlendir:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch mergen_avoidance avoid.launch.py world:=w2_cone_slalom
```

Veya hic GUI istemiyorsan (toplu test / CI icin):

```bash
ros2 launch mergen_avoidance avoid.launch.py world:=w2_cone_slalom gui:=false
```

## Neden Humble, araçta Foxy varken?

2026 "Hazir Arac Mimari Tanimlama Dokumani"na gore yarisma araci:
**Ubuntu 20.04 + ROS1 Noetic & ROS2 Foxy, Docker icinde.**
Foxy'nin Ubuntu 22.04 icin binary dagitimi yok. Humble, Foxy ile `rclpy` API'si
seviyesinde birebir uyumlu; bu depodaki kod iki dagitim altinda da calisir.
Araca gecerken tek degisiklik topic isimleri olacak (bkz. `docs/02-otonom-arac-nasil-yurur.md`).
