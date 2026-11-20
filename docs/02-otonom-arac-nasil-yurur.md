# Otonom Araç Nasıl Yürür?

> Takımda en çok sorulan soru bu. Aşağıdaki cevap, TEKNOFEST Robotaksi 2026
> **Simülasyon Raporu Şablonu**'nun 7. bölümüyle birebir aynı sırayı izliyor —
> yani hakemler de tam olarak bu yapıyı arıyor. Her katmanın yanında bu depodaki
> karşılığı da yazılı.

## 30 saniyelik cevap

Otonom araç bir döngüdür. Saniyede 10–50 kez şu beş adım tekrarlanır:

```
    ┌──────────────┐
    │ 1. ALGILAMA  │  "Etrafımda ne var?"        lidar, kamera, radar
    └──────┬───────┘
           ▼
    ┌──────────────┐
    │ 2. KONUMLAMA │  "Ben neredeyim?"           GPS/IMU, odometri, SLAM
    └──────┬───────┘
           ▼
    ┌──────────────┐
    │ 3. PLANLAMA  │  "Nereye gitmeliyim?"       global rota + yerel yörünge
    └──────┬───────┘
           ▼
    ┌──────────────┐
    │ 4. KARAR     │  "Şimdi ne yapmalıyım?"     durum makinesi, trafik kuralları
    └──────┬───────┘
           ▼
    ┌──────────────┐
    │ 5. KONTROL   │  "Direksiyonu kaç derece?"  PID / MPC → gaz, fren, direksiyon
    └──────┬───────┘
           │
           └──► araç hareket eder ──► dünya değişir ──► başa dön
```

Buna literatürde **"sense–plan–act" döngüsü** veya **otonomi yığını
(autonomy stack)** denir. Kritik nokta: bu bir boru hattı (pipeline) değil,
**kapalı çevrimdir**. Araç her hareket ettiğinde dünya değişir ve döngü
baştan başlar. Bir katman yavaşlarsa (örn. algılama 2 Hz'e düşerse) araç
eski bir dünya resmine göre gaz verir — bu, otonom araçlardaki en tehlikeli
arıza biçimidir. (Bu depoda `avoider_node.py` içindeki *güvenlik bekçisi*
tam olarak bunu engeller: lidar 0.5 s susarsa araç durdurulur.)

---

## 1. ALGILAMA (Perception)

**Soru:** Etrafımda ne var, nerede, ne kadar uzakta?

| Sensör | Ne verir | Güçlü yanı | Zayıf yanı |
|---|---|---|---|
| **Lidar** (Velodyne VLP-16) | 3B nokta bulutu, 360°, 100 m | Mesafe çok kesin (±3 cm), ışıktan bağımsız | Renk/yazı okumaz → tabela tanımaz, pahalı, yağmur/toz |
| **Kamera** (ZED2 stereo) | RGB görüntü + derinlik | Renk, şerit çizgisi, tabela, trafik ışığı | Işığa bağımlı (gölge, karşı güneş), mesafe daha az kesin |
| **Radar** | Mesafe + **hız** (Doppler) | Hızı doğrudan ölçer, hava koşullarına dayanıklı | Düşük çözünürlük, küçük cisimleri karıştırır |
| **GPS/IMU** (Xsens MTi-680) | Konum + yönelim + ivme | Mutlak konum | GPS tek başına ±1.5 m, tünelde/bina arasında kesilir |

**Neden hepsi birden?** Buna **sensör füzyonu** denir ve tek sebebi vardır:
her sensörün kör noktası farklıdır. Lidar "2 m ileride 40 cm genişliğinde bir
şey var" der ama onun bir yaya mı, varil mi olduğunu bilmez. Kamera "orada
bir yaya var" der ama 2 m mi 3 m mi olduğundan emin değildir. İkisini
birleştirince "2.0 m ileride bir yaya var" olur.

**Bu depoda:** `perception.py`. Girdi `sensor_msgs/LaserScan`, çıktı
engel açı aralıkları. Üç iş yapar:

1. **Temizleme** (`clean_scan`) — `nan` atılır, `inf` **boş alan** olarak
   işaretlenir (en sık yapılan hata: `inf`'i atıp boş koridoru duvar sanmak),
   gövdeye çarpan ışınlar atılır.
2. **Gürbüzleştirme** (`cone_distance`, `robust_k=2`) — tek hatalı ışın
   robotu durdurmasın diye saf minimum yerine *k.* en küçük alınır.
3. **Konfigürasyon uzayına şişirme** (`blocked_intervals`) — robot bir nokta
   değil. `d` mesafesindeki engel, o yönde `asin(R/d)` kadar açıyı da kapatır.
   Bunu yapmazsan araç "boşluk var" der, girer ve omzuyla sıyırır.
   *Şartnamedeki "sürterek sakındı → −10 puan" tam olarak bu hatadır.*

> **Gerçek araca geçiş:** VLP-16 `PointCloud2` yayınlar. Yol:
> zemin ayrıştırma (RANSAC ile düzlem bul, at) → yükseklik dilimi al →
> 360 açı kovasına indir → aynı `PolarScan` yapısı. Bu dosyanın
> **altındaki her şey aynen kalır.** Kasıtlı bir tasarım kararı:
> sensör değişimi tek bir dosyayı etkiliyor.

---

## 2. KONUMLAMA (Localization)

**Soru:** Haritada tam olarak neredeyim ve hangi yöne bakıyorum?

Üç kaynak, üç farklı zayıflık:

- **GPS/GNSS** — mutlak konum verir ama Robotaksi aracında RTK olmadan
  **±1.5 m** hata var. Bir şerit 3 m genişliğindeyse, ±1.5 m hata "hangi
  şeritteyim" sorusunu cevaplayamaz demektir. RTK ile bu ±2 cm'e iner.
- **IMU + tekerlek odometrisi** — çok hassas ve hızlıdır (200 Hz) ama
  **sürüklenir** (drift). Hatalar entegre edildiği için 1 dakikada metrelerce
  kayar. Tek başına asla yeterli değildir.
- **SLAM / harita eşleme** — lidar taramasını önceden hazırlanmış haritayla
  eşleyip konumu düzeltir. Yavaş ama sürüklenmez.

**Çözüm:** bunları bir **Kalman filtresi** (pratikte EKF veya UKF) ile
birleştirmek. Sezgisi basit: IMU'ya kısa vadede güven (hızlı ama kayıyor),
GPS'e uzun vadede güven (gürültülü ama kaymıyor). Filtre her ikisinin
belirsizliğini takip edip ağırlıklı ortalama alır.

ROS 2'de hazır çözüm: `robot_localization` paketi (`ekf_node`).

**Bu depoda:** şu an **kasıtlı olarak yok**. Gazebo'daki `/odom` doğrudan
kullanılıyor çünkü simülasyonda sürüklenme yok. Bu bir eksiklik olarak değil,
bilinçli bir sınır olarak duruyor — gerçek araçta `robot_localization`
eklenmesi gerekir ve bu yol haritasının 2. maddesidir.

> **Robotaksi özel notu:** Yarışmada konumlar **GEOJSON** dosyasıyla GPS
> koordinatı olarak veriliyor. Yani enlem/boylamı yerel metrik çerçeveye
> (ENU: doğu-kuzey-yukarı) çevirmek zorunludur. Görev noktası toleransı
> **±1 metre** — RTK olmadan bu toleransı tutturmak çok zor. Bu, takımın
> erken karar vermesi gereken bir konu.

---

## 3. PLANLAMA (Planning)

Burada **iki ayrı problem** var ve karıştırılması en yaygın kavram hatasıdır.

### 3a. Global planlama — "hangi yoldan gideceğim?"

Haritanın tamamına bakar, başlangıçtan hedefe bir **rota** çıkarır.
Saniyede bir kez bile çalışması yeterli.

- **Dijkstra / A\*** — ızgara (grid) veya graf üzerinde en kısa yol.
  A\* = Dijkstra + "hedefe kuş uçuşu mesafe" sezgisi; aynı sonucu çok daha
  hızlı bulur.
- **RRT / RRT\*** — yüksek boyutlu veya kinematik kısıtlı uzaylarda rastgele
  ağaç büyütme. Park manevrası gibi problemler için.

Robotaksi'de global rota **GEOJSON noktalarından** üretilir:
başlangıç → görev noktaları → park girişi.

### 3b. Yerel planlama — "önümdeki 3 metrede ne yapacağım?"

Global rotayı takip etmeye çalışır ama **anlık sensör verisine** göre
engelden kaçar. 10–50 Hz çalışmak zorundadır.

- **DWA** (Dynamic Window Approach) — uygulanabilir (hız, ivme) komut
  kümesini örnekle, her birinin yörüngesini simüle et, en iyisini seç.
- **VFH / VFH+** (Vector Field Histogram) — engelleri açı histogramına çevir,
  boşlukları bul, maliyet fonksiyonuyla en iyi yönü seç.
- **MPC** (Model Predictive Control) — araç modelini kullanarak gelecekteki
  N adımı optimize et. En güçlüsü, en pahalısı.
- **Potansiyel alan** (potential field) — hedef çeker, engeller iter.
  Öğretmesi kolay ama **yerel minimumda kilitlenir** ve dar koridorda titrer.
  Bu yüzden bu depoda kullanılmadı.

**Bu depoda:** `planner.py`, **VFH ailesinden sadeleştirilmiş bir
aday-rota + maliyet fonksiyonu** şeması:

```
1. Lidar'dan geçilemeyen açı aralıklarını çıkar (C-space şişirme)
2. ±90° içinde ~2° adımla ADAY YÖNLER üret  (91 aday)
3. Her adayı üç terimle puanla:
       maliyet(θ) =  w_goal      · |θ − hedef|        → rotadan sapma
                   + w_clearance · (1 − açıklık/ufuk) → kapalılık
                   + w_smooth    · |θ − önceki|       → HİSTEREZİS
4. En ucuz adayı seç
5. Durum makinesine ver
```

Üçüncü terim (`w_smooth`) kritiktir. Onu 0 yaparsan iki eşit boşluk arasında
araç saniyede 10 kez karar değiştirir ve yerinde titrer. Buna **karar
titremesi** denir ve gerçek bir hatadır — bu depoda `test_heading_choice_is_
stable_between_consecutive_ticks` testi onu kilitliyor.

> **Neden potansiyel alan değil?** İki somut sebep: (1) yerel minimum —
> U şeklinde çıkmaz sokakta kilitlenir, (2) dar koridorda iki duvarın itme
> kuvveti dengelenmediği için titrer. Aday-rota şeması ikisini de doğrudan
> ele alıyor: kilitlenmeyi `RECOVER` durumu, titremeyi `w_smooth` çözüyor.

---

## 4. KARAR VERME (Decision Making)

**Soru:** Kurallar ve durumlar ne diyor? Kırmızı ışık var mı? Yaya geçiyor mu?
Sollamalı mıyım, beklemeli miyim?

Pratikte bu bir **durum makinesidir** (state machine) — çünkü davranış
sadece "şu an ne görüyorum"a değil, "hangi moddayım"a da bağlıdır.
Kırmızı ışıkta duran araç ile engel önünde duran araç aynı komutu üretir
(v=0) ama **farklı sebeplerle** durur ve **farklı koşullarda** kalkar.

**Bu depoda:** 6 durumlu makine (`planner.py`, `DriveState`):

| Durum | Ne zaman | Ne yapar |
|---|---|---|
| `CRUISE` | yol açık (> 2.0 m) | hedef yönde tam hız |
| `SLOW` | engel ufukta (< 2.0 m) | hız kademeli düşer |
| `AVOID` | engel yakın (< 1.4 m) | boşluğa yönelir, yavaş ilerler |
| `BLOCKED` | geçilebilir yön yok | **önce durup bekler**, sonra yön arar |
| `EMERGENCY` | TTC kritik / çok yakın | ani fren |
| `RECOVER` | kilitlendi | geri al + dön |

**Burada öğrendiğimiz iki şey, ikisi de ölçülmüş hatalardan:**

1. **BLOCKED'da dönüş yönü kilitlenmeli.** "Hangi taraf daha boş" sorusunun
   cevabı simetrik bir koridorda her karede taraf değiştirir. Yönü her karede
   yeniden seçince araç −19° ile +2° arasında salındı ve çıkmaz sokaktan
   **hiç** kurtulamadı (sonsuz limit çevrimi). Karar bir kez verilip
   korunmalı.

2. **"Yol kapalı" iki farklı şey olabilir.** Önümü bir *duvar* kapatıyorsa
   yön aramaya başlamalıyım. Önümden biri *geçiyorsa* **beklemeliyim.**
   Ayırt edici ölçüt: sahne değişiyor mu? (`profile_activity` — açısal
   profilin kova başına ortalama değişimi, m/s). Bu ayrımı yapmadan önce
   araç, yaya yolu kapatınca 180° dönüp koridorda **ters yöne** gidiyordu.
   *Şartname: ters yön ihlali + "engel tamamen geçene kadar bekledi → +50"
   puanının kaybı.*

---

## 5. KONTROL (Control)

**Soru:** "Sola 20° dön" kararını direksiyon açısına, gaza ve frene nasıl
çeviririm?

- **PID** — hata (istenen − ölçülen) üzerinden oransal + integral + türevsel
  düzeltme. Basit, sağlam, her yerde kullanılır. Robotaksi aracında
  direksiyon ve hız için **hazır PID zaten var** (`steer_pid.yaml`,
  `thrt_pid.yaml`); `/beemobs/steering_target_value` ve
  `/beemobs/speed_target_value` topic'lerine hedef yazarsın, gerisini o halleder.
- **Pure Pursuit / Stanley** — rota takibi için geometrik yöntemler.
  Pure pursuit: rotada ileride bir nokta seç, oraya doğru yay çiz.
- **MPC** — araç modelini kullanıp gelecek N adımı optimize eder. En iyisi,
  en hesap yoğunu.

**Bu depoda:** `control.py`. Üç iş:

1. **Hız profili** — `stop_distance`'ta 0, `slow_distance`'ta tam hız
   (lineer rampa), keskin dönüşte ek kesinti.
2. **İvme limiti** (`rate_limit`) — komut zıplamaz, *sürülür*. Şartnamedeki
   **"ergonomi"**/yolcu konforu maddesinin karşılığı. 0'dan 0.22 m/s'ye tek
   karede zıplayan bir komut gerçek araçta aktüatörü doyurur ve tekerlek
   kaydırır.
3. **Kinematik kısıt** (`apply_kinematics`) — ve burası **en önemli nokta**:

### Diferansiyel sürüş ≠ Ackermann

| | TurtleBot3 (diferansiyel) | Robotaksi (Ackermann) |
|---|---|---|
| Yerinde dönebilir mi? | **Evet** | **Hayır** |
| Dönüş yarıçapı | 0 olabilir | ≥ ~4 m |
| `v=0, w=1.0` komutu | geçerli | **fiziksel olarak imkansız** |

Bu, simülasyondan gerçek araca geçerken takımların en sık düştüğü tuzaktır.
TurtleBot3'te kusursuz çalışan bir algoritma, yerinde dönmeye dayandığı için
gerçek araçta ilk kavşakta kilitlenir.

Bu yüzden `bicycle_mode` parametresi var: `True` yapıldığında yerinde dönme
yasaklanır ve `|w| ≤ v / R_min` zorlanır. `BLOCKED` durumunda araç dönmek
yerine **geri almaya** geçer. Ackermann kinematiğiyle test sonuçları:
`docs/test-sonuclari-ackermann.md`.

---

## Bu döngü ROS 2'de nasıl kurulur?

ROS, bu beş katmanı birbirinden ayırmanın standart yoludur:

- **Node (düğüm)** — bir işi yapan program. `mergen_avoider` bir düğüm.
- **Topic (konu)** — düğümlerin veri alışverişi yaptığı isimli kanal.
  Yayıncı (publisher) yazar, abone (subscriber) okur. Birbirlerini tanımazlar.
- **Message (mesaj)** — topic'ten akan veri tipi. `sensor_msgs/LaserScan`,
  `geometry_msgs/Twist`.
- **TF2** — "lidar gövdenin 17 cm üstünde" gibi çerçeve dönüşümlerini tutan
  ağaç. `base_link`, `base_scan`, `odom`, `map`.
- **QoS** — mesajların teslim politikası. Sensör verisi için `best effort`
  (bir kare kaybetmek, geciken kareyi beklemekten iyidir); komut için
  `reliable`.

Bu depodaki akış:

```
Gazebo
  ├─ /scan   (sensor_msgs/LaserScan)  ──┐
  └─ /odom   (nav_msgs/Odometry)     ──┤
                                       ▼
                             ┌──────────────────────┐
                             │   mergen_avoider     │
                             │  perception→planner  │
                             │      →control        │
                             └──────┬───────────────┘
                                    ├─► /cmd_vel         (Twist)  → Gazebo
                                    ├─► /mergen/state    (String) → teşhis
                                    ├─► /mergen/sectors  (Float32MultiArray)
                                    └─► /mergen/markers  (MarkerArray) → RViz
```

### Gerçek Robotaksi aracında aynı akış

2026 "Hazır Araç Kullanıcı Dokümanı"ndan **gerçek topic isimleri**:

| İş | Topic | Not |
|---|---|---|
| Direksiyon hedefi | `/beemobs/steering_target_value` | Hazır PID devrede; **3 döngü mesaj gelmezse PID devreden çıkar** |
| Hız hedefi | `/beemobs/speed_target_value` | Hazır PID |
| Direksiyon geri besleme | `/beemobs/FeedbackSteeringAngle` | Kendi PID'ini yazacaksan bundan oku |
| Araç hızı | `/beemobs/FB_VehicleSpeed` | |
| Fren kontrolü | `/beemobs/AUTONOMOUS_BrakePedalControl` | |
| Direksiyon motoru | `/beemobs/AUTONOMOUS_SteeringMot_Control` | Önce `AUTONOMOUS_SteeringMot_EN = 1` |
| El freni | `/beemobs/AUTONOMOUS_HB_MotorControl` | |
| Kontak / RC birimi | `/beemobs/rc_unittoOmux` | `RC_Ignition` alanı |
| Canlılık sinyali | `/beemobs/AutonomousHeardBit` | **Kesilirse araç otonomdan çıkar** |

> O "3 döngü mesaj gelmezse PID devreden çıkar" ve "heartbeat kesilirse araç
> otonomdan çıkar" detayları tesadüf değil: güvenlik için öyle tasarlanmış.
> Senin kontrol döngün **sabit frekansta ve kesintisiz** çalışmak zorunda.
> Bu depodaki 20 Hz'lik `create_timer` ve güvenlik bekçisi aynı disiplinin
> karşılığı.

---

## Yol haritası — bu depoda ne YOK

Dürüst olmak, eksiği gizlemekten daha değerli. Şu an bu depo
**yerel planlama + karar + kontrol** katmanlarını kapsıyor. Eksikler,
önem sırasına göre:

| # | Eksik | Neden gerekli | Nasıl eklenir |
|---|---|---|---|
| 1 | **Nesne takibi** (tracking) | Dinamik engelin *hızını* ve *yolumu kesip kesmeyeceğini* bilmek. Şu an iki ardışık kareden kaba kapanma hızı çıkarıyoruz; cisimleri kimliklendirmiyoruz. | En-yakın-komşu eşleştirme + sabit hızlı Kalman; sonra yol kesişim tahmini |
| 2 | **Global planlayıcı** | Yerel minimum (`w7_trap_pocket`) salt tepkisel yöntemle çözülemez | Nav2: costmap + A\* global, bizim planlayıcı yerel kontrolcü olarak |
| 3 | **Konumlama füzyonu** | GPS ±1.5 m, görev toleransı ±1 m | `robot_localization` EKF + RTK |
| 4 | **Şerit tespiti** | Şerit ihlali ±50 puan | Kamera + perspektif dönüşümü (IPM) + eğri uydurma |
| 5 | **Tabela/ışık tanıma** | Her ihlal −50 puan | YOLO benzeri dedektör (araçta RTX 3060 var) |
| 6 | **Park manevrası** | +80 puan | Boş yer tespiti + RRT\* veya hazır manevra şablonu |
| 7 | **GEOJSON → rota** | Görev noktaları ±1 m | GPS→ENU dönüşümü + waypoint takibi |

Bu tablo aynı zamanda **takımda hangi işi isteyebileceğinin listesi.**
