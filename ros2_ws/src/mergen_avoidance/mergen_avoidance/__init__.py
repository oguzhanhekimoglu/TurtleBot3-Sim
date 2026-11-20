"""mergen_avoidance -- TEKNOFEST Robotaksi hazirligi icin lidar tabanli engelden sakinma.

Katmanlar (Robotaksi simulasyon raporu sablonundaki isimlerle birebir):
  geometry.py   : aci/aralik matematigi
  perception.py : ALGILAMA   -- LaserScan -> PolarScan -> engel araliklari
  planner.py    : PLANLAMA + KARAR VERME -- aday rota secimi + durum makinesi
  control.py    : KONTROL    -- hiz profili, ivme limiti, kinematik kisitlar
  sim2d.py      : ROS'suz 2B kinematik simulator (hizli dogrulama + testler)

ROS 2 dugumleri (ince kabuk; tum mantik yukaridaki saf modullerde):
  avoider_node.py    : ana sakinma dugumu   -> /cmd_vel
  metrics_logger.py  : CSV kaydi + kosu ozeti
  pedestrian_node.py : dinamik engel (yaya) surucusu
"""
__version__ = "0.1.0"
