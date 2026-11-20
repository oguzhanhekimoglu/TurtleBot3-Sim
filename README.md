# TurtleBot3-Sim

**Lidar tabanlı otonom engelden sakınma** — TurtleBot3 + Gazebo, ROS 2 Humble.
TEKNOFEST Robotaksi yarışması hazırlığı.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Gazebo Classic 11](https://img.shields.io/badge/Gazebo-Classic%2011-F58113)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![tests](https://img.shields.io/badge/birim%20testi-50%2F50-brightgreen)
![scenarios](https://img.shields.io/badge/Gazebo%20senaryosu-9%2F9%20temiz-brightgreen)
![license](https://img.shields.io/badge/lisans-MIT-blue)

---

| | |
|---|---|
| **Birim testi** | 50/50 — ROS ve Gazebo gerekmeden saniyeler içinde |
| **Gazebo senaryosu** | 9/9, sıfır temas |
| **Kontrol döngüsü** | 20 Hz |
| **Planlayıcı** | VFH türevi, 91 aday yön + 6 durumlu makine |
| **Parametre** | 47, tek dosyada, hepsi birimiyle belgeli |
| **Kinematik** | diferansiyel + Ackermann (gerçek araç kısıtı) |

Karar mantığının tamamı (**1555 satır**) ROS'tan bağımsız saf Python.
`rclpy` yalnızca üç ince düğüm dosyasında — bu sayede testler Gazebo
açmadan koşuyor, ROS 1 Noetic'e taşımak tek dosya yazmak demek
(`ros1_noetic/`), ve sensör değişimi tek modülü etkiliyor.

Senaryolar tek yerde tanımlı; Gazebo dünyaları oradan **üretiliyor**, böylece
2B ve 3B testler aynı parkuru ölçüyor.

```
┌─ ALGILAMA ──────┐  ┌─ PLANLAMA + KARAR ─┐  ┌─ KONTROL ────────┐
│ LaserScan       │  │ 91 aday yön        │  │ hız profili      │
│ → temizle       │─►│ → 3 terimli maliyet│─►│ → ivme limiti    │─► /cmd_vel
│ → gürbüzleştir  │  │ → 6 durumlu FSM    │  │ → kinematik kısıt│
│ → C-space şişir │  │                    │  │                  │
└─────────────────┘  └────────────────────┘  └──────────────────┘
   perception.py          planner.py              control.py
```

---

## Hızlı başlangıç

### ROS kurulu değilse — şimdi çalışan kısım

2B simülatör ve testler **ROS/Gazebo gerektirmez**, saf Python:

```bash
cd ros2_ws/src/mergen_avoidance
python3 test/run_tests.py            # 50 test, karar mantığı
cd ../../..
python3 scripts/sim2d_report.py      # 8 senaryo, sonuç tablosu
```

ROS kurulduysa, Gazebo'ya hâlâ gerek yok — ROS katmanı ayrıca test edilebiliyor:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws/src/mergen_avoidance
python3 test/test_ros_node.py        # sahte /scan → düğüm → /cmd_vel
```

### ROS kurulumu (3 adım)

```bash
sudo bash install/01_ros2_humble.sh      # ROS 2 Humble + Gazebo Classic 11 (~2.5 GB)
sudo bash install/02_turtlebot3_sim.sh   # TurtleBot3 + (ops.) Nav2
bash      install/03_env_and_build.sh    # .bashrc + colcon build   ← sudo YOK
```

Sonra **yeni bir terminal** aç.

### Çalıştır

```bash
bash scripts/run_scenario.sh w2_cone_slalom        # koni slalomu
bash scripts/run_scenario.sh w4_lane_block         # şeridi kapatan engel
bash scripts/run_scenario.sh w8_pedestrian rviz    # aniden çıkan yaya + RViz
bash scripts/run_scenario.sh w6_dense_clutter headless
```

veya doğrudan:

```bash
ros2 launch mergen_avoidance avoid.launch.py world:=w8_pedestrian rviz:=true
```

### Hepsini test et ve rapor üret

```bash
bash scripts/batch_test.sh 60                  # 8 senaryo, penceresiz, CSV
python3 scripts/plot_run.py results/*/*.csv    # rapor grafikleri
```

---

## Senaryolar

Tek yerde tanımlı (`sim2d.py` → `scenario()`), Gazebo dünyaları oradan
üretiliyor (`scripts/generate_worlds.py`).

| Dünya | Ne test ediyor | Şartname karşılığı |
|---|---|---|
| `w1_empty` | taban çizgisi: hız, düz gitme | — |
| `w2_cone_slalom` | art arda yön değiştirme, histerezis | statik engel |
| `w3_corridor` | şerit disiplini (koridor + variller) | şerit ihlali (−50) |
| `w4_lane_block` | şeridi **kısmen** kapatan blok | statik engelden sakınma (+50) |
| `w5_dead_end` | kısa çıkmaz koridor, etrafından dolaşma | yol kapalı → yeniden planlama |
| `w6_dense_clutter` | yoğun koniler, sürekli yeniden planlama | statik engel çeşitliliği |
| `w7_trap_pocket` | kapalı cep — **bilinen sınır** (yerel minimum) | — |
| `w8_pedestrian` | aniden karşıya geçen yaya | dinamik engel (+50 / diskalifiye) |

Güncel sonuçlar:

| Dosya | Ne ölçüyor |
|---|---|
| **`docs/test-sonuclari-gazebo.md`** | **Gazebo, gerçek fizik** — 8/8 temassız |
| `docs/test-sonuclari.md` | 2B simülatör, diferansiyel sürüş |
| `docs/test-sonuclari-ackermann.md` | 2B simülatör, gerçek araç kinematiği |

---

## Dokümanlar

| Dosya | İçerik |
|---|---|
| **[docs/01-sartname-ozeti.md](docs/01-sartname-ozeti.md)** | 2022→2026 şartname karşılaştırması, puanlama tabloları, GEOJSON, araç donanımı |
| **[docs/02-otonom-arac-nasil-yurur.md](docs/02-otonom-arac-nasil-yurur.md)** | Otonomi yığınının beş katmanı, bu depodaki karşılıkları, gerçek araç topic'leri, yol haritası |
| **[docs/03-terimler-sozlugu.md](docs/03-terimler-sozlugu.md)** | ROS + otonomi + yarışma terimleri (öğrenme sırasına göre) |
| `docs/sartnameler/` | Şartnamelerin metne çevrilmiş hâlleri (2022–2026). PDF'ler için `scripts/fetch_sartnameler.sh` |

---

## Depo yapısı

```
TurtleBot3-Sim/
├── install/                      kurulum scriptleri (3 adım)
├── docs/                         şartname özeti, mimari, sözlük, plan
│   └── sartnameler/              resmî PDF + .txt (2022–2026)
├── scripts/
│   ├── run_scenario.sh           tek senaryoyu Gazebo'da çalıştır
│   ├── batch_test.sh             8 senaryoyu penceresiz koştur + CSV
│   ├── generate_worlds.py        sim2d senaryolarından .world üret
│   ├── generate_params_yaml.py   AvoidParams'tan avoider.yaml üret
│   ├── sim2d_report.py           ROS'suz test tablosu üret
│   └── plot_run.py               CSV → rapor grafikleri
└── ros2_ws/src/mergen_avoidance/
    ├── mergen_avoidance/
    │   ├── geometry.py           açı/aralık matematiği
    │   ├── params.py             41 parametre, adıyla ve birimiyle
    │   ├── perception.py         ALGILAMA
    │   ├── planner.py            PLANLAMA + KARAR (durum makinesi)
    │   ├── control.py            KONTROL (hız, ivme, kinematik)
    │   ├── sim2d.py              2B kinematik simülatör + ışın izleme
    │   ├── avoider_node.py       ROS düğümü (ince kabuk)
    │   ├── metrics_logger.py     CSV kaydı + koşu özeti
    │   └── pedestrian_node.py    dinamik engel sürücüsü
    ├── config/avoider.yaml       (üretilmiş)
    ├── launch/avoid.launch.py
    ├── worlds/*.world            (üretilmiş, 8 dünya)
    ├── rviz/mergen.rviz
    └── test/
        ├── test_algorithm.py     50 test — karar mantığı (ROS gerekmez)
        ├── test_ros_node.py      ROS katmanı entegrasyonu (Gazebo gerekmez)
        └── run_tests.py          pytest'siz koşucu
```

---

## Mimari kararlar

**Karar mantığı ROS'tan ayrı.** `rclpy` sadece üç düğüm dosyasında.
Kazanç: testler Gazebo olmadan koşuyor; gerçek araca geçerken tek dosya
değişiyor; sensör değişimi `perception.py`'nin üst kısmıyla sınırlı kalıyor.

**Parametreler tek yerde.** Kodda çıplak sayı yok. 41 eşiğin hepsi
`params.py`'de, adıyla, birimiyle ve *neden o değer* açıklamasıyla.
`avoider.yaml` ve dokümantasyon oradan **üretiliyor**.

**Senaryolar tek yerde.** 2B senaryo ile Gazebo dünyası aynı tanımdan
geliyor. Aksi halde ikisi zamanla ayrışır ve "2B'de geçti, Gazebo'da
çarptı" olur.

**Testler niyeti kilitler.** Her test adı bir tasarım kararını ya da
gerçekten yaşanmış bir hatayı anlatıyor — örn.
`test_blocked_turn_direction_is_latched_and_does_not_chatter`.

---

## Bulunan ve düzeltilen hatalar

On hata, hepsi ölçümle bulundu. Her birinin ayrıntısı
[PROJE-REHBERI.md](PROJE-REHBERI.md) bölüm 7'de.

**Karar mantığında (2B simülatörde bulundu):**

1. **Dönüş yönü titremesi.** Simetrik koridorda "hangi taraf boş" her karede
   taraf değiştiriyordu; araç −19°/+2° arasında salınıp çıkmaz sokaktan
   kurtulamıyordu. → Yön bir kez seçilip kilitleniyor.
2. **Yaya gelince 180° dönme.** "Yol kapalı" iki farklı şey: duvar mı,
   geçen biri mi? → Sahne hareketliliği ölçülüp ayırt ediliyor; hareketliyse
   **bekleniyor**. (45.0 s vs 41.4 s — 3.6 saniye bekledi, çarpmadı.)
3. **Yanal yaklaşan engeli geç görme.** Planlayıcı her kareyi statik
   görüyordu. → Kapanma hızı + TTC tabanlı kademeli fren, ±75° gözetleme
   arkıyla. En yakın yaklaşma 0.00 m → **0.28 m**.

**Entegrasyonda (Gazebo ilk kez çalıştırılınca bulundu — hiçbiri birim
testlerinde görünmüyordu):**

4. **Launch kapsam sızıntısı.** gzserver include'una verilen `world` argümanı
   üst kapsamı ezdi; metrik düğümüne senaryo adı yerine tam dosya yolu gitti
   ve `os.path.join` absolute ikinci parça yüzünden çıktı dizinini **sessizce
   yuttu** → CSV paketin kurulum dizinine yazıldı. → `GroupAction(scoped=True)`
   + düğümde `_sanitize()`.
5. **Ctrl-C'de "dur" komutu hiç gitmiyordu.** rclpy'nin SIGINT işleyicisi
   context'i `halt()` çalışmadan kapatıyordu. Gazebo son `cmd_vel`'i koruduğu
   için robot son komutla devam ederdi. → `runner.py`: sinyal işleyicisini
   devralıp durdurmayı context geçerliyken yayınlıyoruz.
6. **İkinci SIGINT temizliği kesiyordu.** → İlk sinyalden sonra `SIG_IGN`.
7. **YAML `y` harfini boolean'a çevirdi.** `axis: "y"` → YAML 1.1'de `True` →
   yaya düğümü açılışta çöktü, yaya hiç yola çıkmadı. ("Norway problem"ın tek
   harfli hali.) → Değerler `lateral`/`longitudinal`.
8. **Yaya modeli hiç kımıldamıyordu ve test bunu "başarılı" sandı.** ODE
   hareketsiz gövdeyi uyku moduna alıyor; `SetLinearVel` uyandırmıyor
   (ölçüldü: 3 s komuttan sonra 0.0001 m). Araç yayayı hiç görmediği için
   "temas 0" çıkıyordu. → `allow_auto_disable=false` + `kinematic=true` +
   eksik `update_rate`/`publish_rate`. **Ayrıca rapor üretecine etkileşim
   kontrolü eklendi**: engel aracı etkilemediyse sonuç artık "BAŞARILI"
   değil "GEÇERSİZ".
9. **Tetikleme geometrisi testi imkânsız kılıyordu.** `trigger_x=3.2` ile yaya
   robotun tam önüne çıkıyor, 1.05 m uyarı kalıyordu — kaçınılmaz çarpışma.
   → Karşılaşma geometrisi hesaplandı, `trigger_x=1.8`.
10. **Geri giderken arkaya bakmıyordu.** Planlayıcı yalnızca öne bakıyor
   (±90°), `RECOVER` ise aracı geri alıyordu — yani bakmadığı yöne. Dar
   parkurda geri manevrada koniye çarptı (ölçüldü: `t=83 s, RECOVER,
   v=−0.08`). Yarışmada bu diskalifiye sebebi. → Geri almadan önce arka
   açıklık kontrolü; kapalıysa geri gitmiyor, yerinde dönüyor.
   Çarpışma gitti, BLOCKED 445→84, RECOVER 222→37.

---

## Bilinen sınırlar

Dürüst liste. Ayrıntılı yol haritası:
[docs/02](docs/02-otonom-arac-nasil-yurur.md)

- **Nesne takibi yok.** Kapanma hızı iki ardışık taramadan kaba olarak
  çıkarılıyor; cisimler kimliklendirilmiyor. Doğru çözüm: en-yakın-komşu
  eşleştirme + sabit hızlı Kalman + yol kesişim tahmini.
- **Global planlayıcı yok.** Yerel minimum (`w7_trap_pocket`) salt tepkisel
  yöntemle çözülemez. Çözüm: Nav2 global planlayıcı + bu planlayıcı yerel
  kontrolcü olarak.
- **Kamera katmanı yok** → şerit, tabela, trafik ışığı tanıma yok.
- **Konumlama füzyonu yok.** Gazebo `/odom`'u doğrudan kullanılıyor.
  Gerçek araçta `robot_localization` (EKF) + RTK gerekir; görev noktası
  toleransı ±1 m, GPS ise RTK'sız ±1.5 m.
- **Park manevrası yok.**
- **Lidar bekçisi 0.46 s sonra durduruyor.** Ölçüldü (`test_ros_node.py`).
  TurtleBot3'te 0.22 m/s → ~10 cm kör yol, kabul edilebilir. Gerçek araçta
  30 km/h'de aynı eşik **~3.8 m** eder; `scan_timeout` araç hızına göre
  küçültülmeli (VLP-16 20 Hz'e kadar çıkabiliyor, yani 0.1 s makul).
- `scripts/plot_run.py` **henüz çalıştırılmadı** — `matplotlib` kurulum
  scriptinin 2. adımıyla geliyor.
- Gazebo'da 8/8 senaryo temassız doğrulandı
  (`docs/test-sonuclari-gazebo.md`). 2B tahminle 3B ölçüm tutarlı:
  `w2_cone_slalom` için en yakın yaklaşma 2B'de 0.26 m, Gazebo'da **0.26 m**.
- **Gazebo arkada `gzserver` bırakabiliyor.** Yetim süreç 11345 portunu tutar
  ve sonraki koşu `Address already in use` ile sessizce ölür, boş CSV üretir.
  `run_scenario.sh` artık başlamadan kontrol edip soruyor; `batch_test.sh`
  hem başta hem her senaryodan sonra temizliyor. Elle bakmak için:
  `pgrep -x gzserver`.

---

## Sık kullanılan komutlar

```bash
# teşhis
ros2 node info /mergen_avoider
ros2 topic echo /mergen/state            # kararın gerekçesini canlı oku
ros2 topic hz /scan
ros2 run rqt_graph rqt_graph

# canlı parametre denemesi
ros2 param set /mergen_avoider max_linear_speed 0.15
ros2 param set /mergen_avoider w_smooth 0.0          # histerezisi kapat → titreme
ros2 param set /mergen_avoider bicycle_mode true     # Ackermann kısıtı

# kayıt (şartname: itirazlarda log istenebilir)
ros2 bag record /scan /odom /cmd_vel /mergen/state /mergen/sectors

# üreteçler
python3 scripts/generate_worlds.py
python3 scripts/generate_params_yaml.py
```

---

## Repoyu paylaşmak

```bash
cd /home/th3g3ntl3man/Desktop/mergen-01
git init && git add -A
git commit -m "mergen-01: lidar tabanli otonom engelden sakinma (TurtleBot3/ROS2)"
gh repo create mergen-01 --public --source=. --push     # gh kuruluysa
```

`.gitignore` hazır: `build/`, `install/`, `log/`, `results/`, `__pycache__/`
dışlanıyor. Şartname **PDF ve .docx** dosyaları da dışlanıyor (19 MB); metne
çevrilmiş `.txt` hâlleri repoda — aranabilir ve küçük. Depo toplam **0.87 MB**.

Orijinal dokümanları indirmek için:

```bash
bash scripts/fetch_sartnameler.sh
```
