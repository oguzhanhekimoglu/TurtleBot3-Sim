#!/usr/bin/env python3
"""Gazebo `.world` dosyalarini sim2d senaryolarindan URETIR.

Neden uretim, elle yazim degil:
  * 2B simulatorde test ettigin yerlesim ile Gazebo'da gordugun yerlesimin
    AYNI olmasini garanti eder. Iki yerde elle tanimlanan parkurlar zamanla
    kacinilmaz olarak birbirinden uzaklasir ve "2B'de gecti, Gazebo'da carpti"
    durumuna yol acar.
  * Yeni senaryo eklemek tek satir: sim2d.scenario()'ya ekle, bu scripti kostur.

Kullanim:
    python3 scripts/generate_worlds.py            # hepsini uret
    python3 scripts/generate_worlds.py w2_cone_slalom
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "ros2_ws" / "src" / "mergen_avoidance"
sys.path.insert(0, str(PKG))

from mergen_avoidance.sim2d import SCENARIOS, Box, Circle, scenario  # noqa: E402

OBSTACLE_HEIGHT = 0.7
"""Engel yuksekligi [m]. TurtleBot3'un lidari ~0.17 m yukseklikte tarar;
0.7 m hem lidar tarafindan gorulmeyi hem gorsel netligi garanti eder."""

HEADER = """<?xml version="1.0" ?>
<!-- ====================================================================
     OTOMATIK URETILDI. Elle duzenleme kalici olmaz.
     Kaynak: ros2_ws/src/mergen_avoidance/mergen_avoidance/sim2d.py
             scenario("{name}")
     Yeniden uretmek icin: python3 scripts/generate_worlds.py {name}
     ==================================================================== -->
<sdf version="1.6">
  <world name="{name}">

    <!-- Isik ve zemin DOGRUDAN burada tanimli; model://ground_plane gibi
         harici modeller Gazebo Fuel'den indirilmeye calisilir ve internet
         yoksa dunya acilmaz. Self-contained olmasi kasitli. -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.7 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <physics type="ode">
      <!-- 1 ms adim / 1000 Hz: diff_drive ve lidar icin fazlasiyla yeterli.
           Toplu testte hizlandirmak icin real_time_update_rate'i 0 yap
           (olabildigince hizli kos); bkz. scripts/batch_test.sh -->
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
"""

FOOTER = """
  </world>
</sdf>
"""

CYLINDER = """
    <model name="{mname}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {hz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material>
            <ambient>0.9 0.35 0.05 1</ambient>
            <diffuse>1.0 0.45 0.05 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

BOX = """
    <model name="{mname}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {hz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
          <material>
            <ambient>0.75 0.75 0.78 1</ambient>
            <diffuse>0.85 0.85 0.88 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

# Dinamik engel: gazebo_ros_planar_move eklentisiyle surulen kutu.
# `pedestrian_node.py` buna /pedestrian/cmd_vel yazar.
PEDESTRIAN = """
    <!-- ================= DINAMIK ENGEL (yaya) =================
         gazebo_ros_planar_move eklentisi modeli duzlemde kinematik olarak
         surer. Komut: /pedestrian/cmd_vel  (mergen_avoidance/pedestrian)
         <static> OLMAMALI, yoksa eklenti modeli hareket ettiremez.
         ======================================================== -->
    <model name="pedestrian">
      <pose>{x:.3f} {y:.3f} {hz:.3f} 0 0 0</pose>

      <!-- ODE hareketsiz govdeleri bir sure sonra UYKU MODUNA alir
           (auto-disable). Yaya tetiklenene kadar ~16 saniye kipirdamadan
           bekledigi icin uykuya daliyor ve `SetLinearVel` onu uyandirmiyor:
           komut gidiyor, model yerinde kaliyor. Olculdu: 3 saniyelik
           0.45 m/s komuttan sonra yer degistirme 0.0001 m. -->
      <allow_auto_disable>false</allow_auto_disable>

      <link name="ped_link">
        <!-- Kinematik govde: hiz komutuyla hareket eder, kuvvetlerden
             etkilenmez. Yaya icin dogru model: arac ona carparsa yaya
             savrulmaz, ki gercekte de carpismayi oyle degerlendiriyoruz. -->
        <kinematic>true</kinematic>
        <inertial>
          <mass>20.0</mass>
          <inertia>
            <ixx>1.5</ixx><iyy>1.5</iyy><izz>0.6</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.45 0.45 {h:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.45 0.45 {h:.3f}</size></box></geometry>
          <material>
            <ambient>0.85 0.1 0.1 1</ambient>
            <diffuse>1.0 0.15 0.15 1</diffuse>
          </material>
        </visual>
      </link>
      <plugin name="pedestrian_planar_move" filename="libgazebo_ros_planar_move.so">
        <ros>
          <namespace>/pedestrian</namespace>
        </ros>
        <!-- Resmi ornekte (gazebo_plugins/worlds/gazebo_ros_planar_move_demo.world)
             bu iki alan var; eksikken davranis surumden surume degisiyor. -->
        <update_rate>100</update_rate>
        <publish_rate>10</publish_rate>
        <odometry_frame>pedestrian_odom</odometry_frame>
        <robot_base_frame>ped_link</robot_base_frame>
        <publish_odom>false</publish_odom>
        <publish_odom_tf>false</publish_odom_tf>
      </plugin>
    </model>
"""


def build_world(name: str) -> str:
    obstacles = scenario(name)
    parts = [HEADER.format(name=name)]
    half_h = OBSTACLE_HEIGHT / 2.0

    cones = boxes = 0
    for obstacle in obstacles:
        if isinstance(obstacle, Circle):
            parts.append(CYLINDER.format(
                mname=f"cone_{cones}", x=obstacle.x, y=obstacle.y,
                hz=half_h, r=obstacle.radius, h=OBSTACLE_HEIGHT))
            cones += 1
        elif isinstance(obstacle, Box):
            parts.append(BOX.format(
                mname=f"wall_{boxes}", x=obstacle.x, y=obstacle.y, hz=half_h,
                sx=obstacle.size_x, sy=obstacle.size_y, h=OBSTACLE_HEIGHT))
            boxes += 1
        else:
            raise TypeError(f"bilinmeyen engel tipi: {type(obstacle)!r}")

    if name == "w8_pedestrian":
        # 2B taraftaki `pedestrian_crossing()` ile ayni baslangic noktasi.
        parts.append(PEDESTRIAN.format(x=4.0, y=-2.0, hz=half_h, h=OBSTACLE_HEIGHT))

    parts.append(FOOTER)
    text = "".join(parts)
    _assert_comments_are_valid(text)
    return text


def _assert_comments_are_valid(text: str) -> None:
    """XML yorumlari '--' iceremez (XML 1.0 spesifikasyonu).

    Bu hata SESSIZDIR ve bir kez yasandi: uretilen 8 dunyanin tamami gecersiz
    XML'di cunku bir yorumda "URETILDI -- elle duzenleme" yaziyordu. Gazebo'nun
    TinyXML ayristiricisi boyle durumlari bazen tolere eder, bazen dunyayi hic
    acmaz -- yani hatayi ancak Gazebo'yu actiginda ve anlamsiz bir mesajla
    gorursun. Ureticinin bunu uretmesini imkansiz kiliyoruz.
    """
    cursor = 0
    while True:
        start = text.find("<!--", cursor)
        if start < 0:
            return
        end = text.find("-->", start + 4)
        if end < 0:
            raise ValueError("kapatilmamis XML yorumu")
        body = text[start + 4:end]
        if "--" in body:
            where = body.find("--")
            snippet = body[max(0, where - 40):where + 40]
            raise ValueError(
                "XML yorumunun icinde '--' var, bu gecersiz XML uretir:\n"
                f"  ...{snippet.strip()}..."
            )
        cursor = end + 3


def main() -> int:
    targets = sys.argv[1:] or SCENARIOS
    out_dir = PKG / "worlds"
    out_dir.mkdir(parents=True, exist_ok=True)

    for name in targets:
        if name not in SCENARIOS:
            print(f"HATA: bilinmeyen senaryo '{name}'. Mevcut: {', '.join(SCENARIOS)}")
            return 1
        text = build_world(name)
        path = out_dir / f"{name}.world"
        path.write_text(text, encoding="utf8")
        n_models = text.count("<model name=")
        print(f"  {path.relative_to(REPO)}  ({n_models} model, {len(text)} bayt)")
    print(f"\n{len(targets)} dunya uretildi -> {out_dir.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
