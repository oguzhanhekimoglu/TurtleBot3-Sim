# ROS 1 (Noetic) Sürümü

Takım **Ubuntu 20.04 + ROS Noetic** kullanıyor; bu depo **ROS 2 Humble** ile
yazıldı. Bu klasör o farkın ne kadar küçük olduğunu gösteriyor.

---

## Tek sayı

```
Taşınması gerekmeyen kod (saf Python) : 1555 satır
Yeniden yazılan kod (ROS kabuğu)      :  332 satır  ← sadece bu dosya
```

| Dosya | ROS'a bağımlı mı? | Noetic'te |
|---|---|---|
| `geometry.py` | hayır | aynen çalışır |
| `params.py` | hayır | aynen çalışır |
| `perception.py` | hayır | aynen çalışır |
| `planner.py` | hayır | aynen çalışır |
| `control.py` | hayır | aynen çalışır |
| `sim2d.py` | hayır | aynen çalışır |
| `avoider_node.py` | **evet** | → `avoider_node_rospy.py` |

Mesaj tipleri bile değişmiyor: `sensor_msgs/LaserScan`,
`geometry_msgs/Twist`, `nav_msgs/Odometry` ROS 1 ve ROS 2'de aynı isimlerle var.

Bu tesadüf değil — mimari baştan böyle kuruldu. "Karar mantığını ROS'tan
ayırdım" cümlesinin somut karşılığı bu klasör.

---

## ROS 2 → ROS 1 karşılık tablosu

| ROS 2 (rclpy) | ROS 1 (rospy) |
|---|---|
| `rclpy.init()` | `rospy.init_node('ad')` |
| `class X(Node)` | sade sınıf, taban sınıf yok |
| `create_subscription(T, 'top', cb, qos)` | `rospy.Subscriber('top', T, cb, queue_size=1)` |
| `create_publisher(T, 'top', 10)` | `rospy.Publisher('top', T, queue_size=10)` |
| `create_timer(0.05, cb)` | `rospy.Timer(rospy.Duration(0.05), cb)` |
| `get_logger().info(...)` | `rospy.loginfo(...)` |
| `get_clock().now()` | `rospy.Time.now()` |
| `declare_parameter` / `get_parameter` | `rospy.get_param('~ad', varsayılan)` |
| **QoS profilleri** | **yok** — yerine `queue_size` |
| `rclpy.spin(node)` | `rospy.spin()` |
| `try/finally` ile temizlik | `rospy.on_shutdown(cb)` |

---

## Üç kavramsal fark

**1. QoS yok.** ROS 2'de sensör verisi için `best effort` seçiyorduk:
"geciken kareyi bekleme, yenisini al". ROS 1'de bunun karşılığı
`queue_size=1` — kuyrukta eski tarama biriktirme.

**2. Parametreler merkezi.** ROS 2'de her düğüm kendi parametresini
*tanımlar*. ROS 1'de hepsi tek bir parameter server'da durur ve düğüm
sadece *okur* (`~` öneki "bu düğüme özel" demek). Bizim `AvoidParams`
sınıfı iki tarafta da aynı kalıyor, sadece okuma şekli değişiyor.

**3. Kapanış daha kolay.** ROS 2'de Ctrl-C'de "dur" komutunun hiç
gitmediğini bulmuştuk — `rclpy` context'i `finally` bloğundan önce
kapatıyordu, çözüm için sinyal işleyicisini devralmak gerekti
(`runner.py`). ROS 1'de `on_shutdown` geri çağrısı context hâlâ
geçerliyken çalışır, yani o sorun kendiliğinden yok.

> Bu üçüncü madde iyi bir sohbet konusu: ROS 2 çoğu konuda daha
> gelişmiş, ama her değişiklik bedava değil.

---

## Çalıştırma (Noetic makinede)

```bash
# turtlebot3 simülasyonu
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# başka terminalde
python ros1_noetic/avoider_node_rospy.py
```

Parametre vermek istersen:

```bash
rosparam set /mergen_avoider/max_linear_speed 0.15
rosparam set /mergen_avoider/bicycle_mode true
```

---

## Dürüst not

**Bu dosya çalıştırılmadı.** Noetic, Ubuntu 20.04 ister; bu makine 22.04.
Söz dizimi kontrol edildi ve ROS 1 API'si satır satır karşılaştırıldı, ama
gerçek bir Noetic kurulumunda denenmedi.

Denemeden "çalışıyor" demiyoruz — bu projede bulduğumuz on hatanın çoğu tam
olarak bu yüzden bulundu.

**Takımın Noetic makinesinde denerken beklenecek tipik pürüzler:**

- `sys.path` eklemesi yerine düzgün bir catkin paketi yapmak gerekebilir
- Python sürümü: saf modüller `dataclasses` ve `from __future__ import
  annotations` kullanıyor (Python 3.7+). Noetic'te Python 3.8 var — uyumlu,
  kontrol edildi.
- `Point(x, y, z)` konumsal argümanları ROS 1'de çalışır; ROS 2'de anahtar
  kelime gerekiyordu — bu dosyada ROS 1 biçimi kullanıldı
