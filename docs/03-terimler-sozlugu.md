# Terimler Sözlüğü

Bir hafta içinde öğrenilmesi gereken dil. Sıralama alfabetik değil,
**öğrenme sırasına göre**: önce ROS, sonra otonomi, sonra yarışma.

---

## Bölüm 1 — ROS 2 temelleri

| Terim | Ne demek | Neden önemli |
|---|---|---|
| **ROS** | Robot Operating System. İşletim sistemi değil; robot yazılımı için **çatı/ara katman** (framework + middleware). | Bütün yarışma yazılımı bunun üstünde |
| **ROS 1 / ROS 2** | ROS 2 yeniden yazım. DDS tabanlı, gerçek zamanlıya daha yakın, dağıtık ve güvenli. | Araçta **ikisi de** kurulu (Noetic + Foxy) |
| **Dağıtım (distro)** | ROS sürüm adı: Noetic (ROS1), Foxy/Humble (ROS2) | Araç = Foxy, bizim makine = Humble |
| **Node (düğüm)** | Bir işi yapan tek program. `mergen_avoider` bir düğüm. | Mimarinin yapı taşı |
| **Topic (konu)** | İsimli veri kanalı. `/scan`, `/cmd_vel` | Düğümler sadece bununla konuşur |
| **Publisher / Subscriber** | Yayıncı yazar, abone okur. **Birbirlerini tanımazlar.** | Gevşek bağlılık (loose coupling) buradan gelir |
| **Message (mesaj)** | Topic'ten akan veri tipi. `sensor_msgs/LaserScan` | Arayüz sözleşmesi |
| **Service** | İstek–yanıt (senkron). "Haritayı kaydet" gibi. | Sürekli akış için değil |
| **Action** | Uzun süren, iptal edilebilir, geri bildirimli iş. "Şu noktaya git." | Nav2 bunu kullanır |
| **Parameter** | Düğümün çalışma zamanı ayarı. `ros2 param set ...` | Kodu yeniden derlemeden ayar |
| **Launch file** | Birçok düğümü tek komutla, ayarlarıyla başlatan Python/XML dosyası | `avoid.launch.py` |
| **Workspace / colcon** | `src/` + `build/` + `install/` düzeni; `colcon build` ile derleme | `ros2_ws/` |
| **Package (paket)** | Dağıtılabilir birim. `package.xml` + `setup.py` | `mergen_avoidance` |
| **QoS** | Quality of Service — teslim politikası: `reliable` (garanti) / `best effort` (hızlı) | Sensör verisi best effort olmalı |
| **DDS** | ROS 2'nin altındaki veri dağıtım katmanı | `ROS_DOMAIN_ID` çakışmalarının sebebi |
| **TF2** | Çerçeve dönüşüm ağacı: `map → odom → base_link → base_scan` | "Lidar gövdenin neresinde?" |
| **Frame (çerçeve)** | Koordinat sistemi. `base_link` = gövde, `odom` = başlangıç, `map` = harita | Açı/işaret hatalarının ana kaynağı |
| **rosbag** | Topic kayıt/oynatma. `ros2 bag record -a` | İtirazlarda log kanıtı; offline test |
| **RViz** | 3B görselleştirme. Sensör + karar görünümü. | "Neden sola döndü?" sorusunu *görmek* |
| **Gazebo** | Fizik simülatörü. Classic 11 (bu depo) / Ignition-Harmonic (yeni) | Araç olmadan test |
| **URDF / SDF** | Robot/dünya tanım XML'leri. URDF robot, SDF Gazebo dünyası. | `worlds/*.world` SDF'tir |

---

## Bölüm 2 — Otonomi yığını

| Terim | Ne demek |
|---|---|
| **Autonomy stack** | Algılama → Konumlama → Planlama → Karar → Kontrol döngüsünün tamamı |
| **Perception (algılama)** | Sensörden "etrafımda ne var" bilgisi çıkarmak |
| **Sensor fusion** | Birden çok sensörü birleştirmek; her sensörün kör noktası farklı |
| **Localization (konumlama)** | "Neredeyim" — GPS + IMU + odometri + harita eşleme |
| **Odometry (odometri)** | Tekerlek/IMU'dan hareket kestirimi. Hassas ama **sürüklenir** |
| **Drift (sürüklenme)** | Odometri hatasının zamanla birikmesi |
| **SLAM** | Simultaneous Localization and Mapping — aynı anda haritala ve konumlan |
| **Kalman filtresi / EKF** | Gürültülü ölçümleri belirsizliklerine göre ağırlıklı birleştirme |
| **RTK** | Real-Time Kinematic GPS. ±1.5 m → **±2 cm**. Robotaksi'de ±1 m tolerans için kritik |
| **ENU** | East-North-Up yerel metrik çerçeve. GPS enlem/boylamı buna çevrilir |
| **Global planner** | Haritanın tamamına bakıp rota çıkarır (A*, Dijkstra, RRT) |
| **Local planner** | Anlık sensöre göre rotayı takip edip engelden kaçar (DWA, VFH, MPC) |
| **A\*** | Dijkstra + hedefe sezgisel mesafe. En yaygın global planlayıcı |
| **RRT / RRT\*** | Rastgele ağaç büyüterek yol bulma. Park manevrası gibi kısıtlı problemler |
| **DWA** | Dynamic Window Approach — uygulanabilir hız komutlarını örnekle ve puanla |
| **VFH / VFH+** | Vector Field Histogram — engelleri açı histogramına çevir, boşluk seç. **Bu depodaki yöntem** |
| **Potential field** | Hedef çeker, engel iter. Basit ama yerel minimumda kilitlenir |
| **Costmap** | Izgara haritada her hücrenin "maliyeti". Nav2'nin temeli |
| **Inflation (şişirme)** | Engelleri robot yarıçapı kadar büyütmek → robotu nokta gibi planlayabilmek. **C-space** |
| **C-space** | Konfigürasyon uzayı. Robotun gövdesi engellere eklenince robot bir noktaya indirgenir |
| **Local minimum** | Yerel planlayıcının kendi başına çıkamadığı tuzak (U şeklinde çıkmaz sokak) |
| **State machine (durum makinesi)** | Davranışı ayrık modlara bölmek: CRUISE / AVOID / BLOCKED ... |
| **FSM** | Finite State Machine — sonlu durum makinesi |
| **Hysteresis (histerezis)** | Karar değiştirmeyi zorlaştıran ek maliyet → titremeyi önler |
| **Chattering (titreme)** | Kararın hızla taraf değiştirmesi. Gerçek bir arıza biçimi |
| **Limit cycle** | Sistemin kendini tekrarlayan ve ilerlemeyen döngüye kilitlemesi |
| **TTC** | Time To Collision — çarpışmaya kalan süre = mesafe / kapanma hızı |
| **Closing rate (kapanma hızı)** | Mesafenin azalma hızı [m/s]. Pozitif = yaklaşıyor |
| **Control (kontrol)** | Kararı aktüatör komutuna çevirmek |
| **PID** | Oransal + İntegral + Türevsel kontrolcü. Araçta direksiyon/hız için hazır var |
| **MPC** | Model Predictive Control — araç modeliyle gelecek N adımı optimize et |
| **Pure Pursuit** | Rotada ileride bir nokta seç, oraya yay çiz. Rota takibi |
| **Slew rate limit** | Komutun birim zamanda değişebileceği miktarın sınırı = ivme limiti |
| **Ackermann** | Otomobil direksiyon geometrisi. **Yerinde dönemez**, min dönüş yarıçapı var |
| **Differential drive** | İki tekerin hız farkıyla dönüş. **Yerinde dönebilir** (TurtleBot3) |
| **Unicycle model** | (v, ω) ile hareket modeli. Diferansiyel sürüşün matematiği |
| **Holonomic / non-holonomic** | Her yöne gidebilen / gidemeyen. Araba non-holonomic |
| **Dead reckoning** | Sadece odometriyle konum takibi (dış referans yok) |
| **Point cloud** | 3B nokta kümesi. Lidar çıktısı (`sensor_msgs/PointCloud2`) |
| **RANSAC** | Aykırı değerlere dayanıklı model uydurma. Lidar'da zemin düzlemi bulmak için |
| **Ground segmentation** | Nokta bulutundan zemini ayırıp atmak (yoksa zemin "engel" olur) |
| **Voxel grid** | Nokta bulutunu 3B hücrelere indirgeyerek seyrekleştirme |
| **IPM** | Inverse Perspective Mapping — kamerayı kuşbakışına çevirmek (şerit tespiti) |
| **Object tracking** | Cisimleri kareler arası eşleyip kimliklendirmek ve hızını kestirmek |
| **ODD** | Operational Design Domain — sistemin çalışmayı taahhüt ettiği koşullar |
| **SAE Seviye 0–5** | Otonomi seviyeleri. Robotaksi hedefi pratikte **Seviye 4** (sınırlı alanda tam otonom) |

---

## Bölüm 3 — Yarışma terimleri

| Terim | Ne demek |
|---|---|
| **Şartname** | Yarışmanın kural kitabı. Her yıl sürüm numarasıyla güncellenir |
| **Teknik Şartname** | Ana şartnameden **ayrı** yayımlanan, ölçülebilir kriterler dokümanı (durma mesafesi, fren tepki süresi, JSON şeması) |
| **TYF** | Teknik Yeterlilik Formu — ilk eleme aşaması (Mart) |
| **KTR** | Kritik Tasarım Raporu — ikinci aşama (Mayıs) |
| **Simülasyon Raporu** | Hazır Araç kategorisinde istenen, maks. 20 sayfalık rapor. **Başlıkları doğrudan puanlanır** |
| **Tur** | Final günlerinde her takımın hakkı. Puanlar ve cezalar turlar üzerinden toplanır |
| **Datum / Referans Noktası** | Parkurda önceden belirlenmiş GPS koordinatı; tüm ölçümlerin referansı |
| **GEOJSON** | Görev noktalarının paylaşıldığı dosya formatı. Koordinat sırası **[boylam, enlem]** |
| **Kafa açısı (heading)** | Aracın kuzey referansına göre baktığı yön. Görev noktalarında *doğru kafa açısı* şart |
| **Görev noktası** | GEOJSON'daki, ±1 m toleransla geçilmesi gereken nokta (+30 puan) |
| **Yolcu alma/indirme** | Belirtilen noktada 15–20 sn duraklama (+70 puan/nokta) |
| **Park bölgesi noktası** | Otoparkın **girişi** — park edilecek yer değil |
| **UMS** | Uzaktan Acil Müdahale Sistemi. Teknik şartnamede detaylanıyor |
| **Teknik kontrol** | Yarış öncesi zorunlu araç denetimi. Geçemeyen değerlendirmeye giremez |
| **Start verme** | Araç başlangıçta çalıştırılır; hareket ederse start verilmiş sayılır. **Sonrasında müdahale yok** |
| **Statik engel** | Şeridi kısmen/tamamen kapatan sabit engel (bariyer, koni) |
| **Dinamik engel** | Aniden önüne çıkan hareketli engel. Çarpmak = turdan diskalifiye |
| **Şerit ihlali** | 2 tekerleğin tamamen şerit dışına çıkması. −50, her 10 sn yeni ihlal |
| **Sürtme** | Engele temas ederek sakınma. −10 (çarpma değil ama cezalı) |
| **Normalizasyon** | Diskalifiye puanlarının sıfıra çekilip farkın diğer takımlara eklenmesi |
| **BEEMOBS** | Hazır Araç platformunun ROS topic ad alanı (`/beemobs/...`) |
| **Özgün / Hazır Araç** | Kendi aracını üretmek / organizasyonun aracını kullanmak |

---

## Bölüm 3.5 — Entegrasyon tuzakları

Bunlar "terim" değil, **saat yiyen tuzaklar**. Hepsi bu depoda gerçekten
yaşandı; isimlerini bilmek ikinci kez yakalanmanı önler.

| Tuzak | Ne oluyor | Belirtisi |
|---|---|---|
| **YAML 1.1 boolean coercion** ("Norway problem") | YAML çıplak `y`, `n`, `yes`, `no`, `on`, `off`, `NO` değerlerini **boolean** okur | String beklenen parametreye `True` gelir → `InvalidParameterTypeException`. Çözüm: tırnakla veya o değerleri hiç kullanma |
| **`os.path.join` absolute yutması** | İkinci parça `/` ile başlıyorsa Python ilk parçayı **sessizce atar** | Dosya beklenmedik yere yazılır, hata yok |
| **Launch kapsam sızıntısı** | `IncludeLaunchDescription`'ın `launch_arguments`'ı üst kapsamı ezebilir | Başka bir düğüme yanlış parametre gider. Çözüm: `GroupAction(scoped=True)` |
| **rclpy sinyal işleyicisi** | `rclpy.init` kendi SIGINT işleyicisini kurar; context `finally`'den **önce** kapanır | `publisher's context is invalid` — ve kapanışta yayınlamak istediğin komut hiç gitmez |
| **Çift SIGINT** | Ctrl-C süreç grubuna birden çok kez ulaşır | Temizlik ortasında ikinci `KeyboardInterrupt` |
| **QoS uyuşmazlığı** | Yayıncı `reliable`, abone `best effort` (veya tersi) | Topic var, `ros2 topic hz` çalışıyor ama **düğüm hiç mesaj almıyor** |
| **`inf` ≠ engel** | LaserScan'de `inf` "dönüş yok" = **boş alan** demek | Atarsan boş koridoru duvar sanarsın |
| **Göreli topic adı** | `create_subscription(..., "scan", ...)` düğümün ad alanına göre çözülür | Namespace'li düğümde `/pedestrian/scan`'e abone olursun |
| **`use_sim_time`** | Gazebo'da saat `/clock`'tan gelir | Unutursan zaman damgaları ve TF tutarsız olur |
| **apport "çökme" bildirimi** | Launch dosyasındaki her istisna Ubuntu'ya "çökme" gibi görünür | Gerçek bir sorun değil; terminaldeki mesajı oku |
| **Asılı kalan `gzserver`** | Ctrl-C'den sonra arkada kalır, 11345 portunu tutar | Sonraki koşu `Unable to start server [bind: Address already in use]` ile **anında** ölür; mesaj launch çıktısında kaybolur. `pgrep -x gzserver` ile bak |
| **`pkill -f` kendini öldürmesi** | `-f` tüm komut satırını tarar; kendi komutun da o kelimeyi içeriyorsa kendini öldürür | Komut hiç çıktı üretmeden ölür. `pkill -x <ad>` kullan |
| **ODE auto-disable** | Hareketsiz gövde uyku moduna girer, `SetLinearVel` uyandırmaz | Komut gidiyor, model yerinde duruyor. `<allow_auto_disable>false</allow_auto_disable>` |
| **Yeşil yanan boş test** | Engeli hiç görmeyen araç da çarpmaz | "Çarpışma yok" tek başına kanıt değil; testin etkileşim gerçekleştiğini de doğrulaması gerekir |

---

## Bölüm 4 — Bu depodaki isimler

Kaptanla kod konuşurken bu isimleri kullanacaksın:

| İsim | Ne |
|---|---|
| `PolarScan` | Temizlenmiş tarama: (açı, mesafe) çiftleri, gövde çerçevesinde |
| `ScanGeometry` | LaserScan başlığındaki sabitler (angle_min, increment, range_min/max) |
| `clean_scan()` | Ham `ranges` → `PolarScan`. `inf` = boş alan, `nan` = atılır |
| `cone_distance()` | Bir koni içindeki engel mesafesi, `robust_k` ile gürültüye dayanıklı |
| `blocked_intervals()` | Geçilemeyen açı aralıkları (C-space şişirme) |
| `closing_rate()` | İki ardışık taramadan kapanma hızı |
| `angular_profile()` | Açısal profil — sahne değişimini ölçmek için kovalar |
| `profile_activity()` | Sahne ne kadar hızlı değişiyor [m/s] → "duvar mı, geçen biri mi?" |
| `AvoidancePlanner` | Durum makinesi + aday rota seçimi. ROS'tan habersiz |
| `DriveState` | CRUISE / SLOW / AVOID / BLOCKED / EMERGENCY / RECOVER |
| `Decision` | Bir adımın tüm çıktısı **ve gerekçesi** (loglanabilir) |
| `speed_profile()` | Açıklığa ve dönüş keskinliğine göre hedef hız |
| `rate_limit()` | İvme limiti (slew rate limiter) |
| `apply_kinematics()` | Diferansiyel mi Ackermann mı — kısıtları uygular |
| `sim2d.py` | ROS'suz 2B kinematik simülatör + ışın izleme |
| `AvoidParams` | 41 parametrenin tamamı, adıyla ve birimiyle tek yerde |
