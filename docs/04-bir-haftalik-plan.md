# 1 Haftalık Plan

Hedef: hafta sonunda kaptanın karşısına **çalışan bir simülasyon + okunabilir
bir repo + doğru terimlerle konuşabilen bir kafayla** çıkmak.

Varsayım: günde 4–6 saat. Daha azsa 2. ve 3. günü birleştir, 7. günü at.

> **Sıralamanın mantığı:** önce *çalışır hale getir* (Gün 1), sonra *anla*
> (Gün 2–3), sonra *değiştirerek öğren* (Gün 4–5), sonra *anlatmaya hazırlan*
> (Gün 6–7). Tersine çevirme — teoriyle başlayıp koda geçmek, bu işte en yavaş
> yoldur.

---

## Gün 1 — Kurulum ve ilk çalıştırma

Tek hedefi var: **robotun ekranda engelden kaçtığını görmek.** Anlamak Gün 2'nin işi.

```bash
sudo bash install/01_ros2_humble.sh      # ~2.5 GB, 10-40 dk
sudo bash install/02_turtlebot3_sim.sh
bash      install/03_env_and_build.sh    # sudo YOK
# yeni terminal aç
bash scripts/run_scenario.sh w2_cone_slalom
```

Sonra sırayla şunları dene (her biri 2 dakika):

```bash
bash scripts/run_scenario.sh w4_lane_block        # şeridi kapatan engel
bash scripts/run_scenario.sh w6_dense_clutter     # yoğun koniler
bash scripts/run_scenario.sh w8_pedestrian        # aniden çıkan yaya
bash scripts/run_scenario.sh w2_cone_slalom rviz  # RViz ile karar görselleri
```

**Günün sonunda yapabilmelisin:**
- Gazebo ve RViz'i açıp kapatmak
- `ros2 topic list`, `ros2 topic echo /scan --once`, `ros2 node list`
- `ros2 topic hz /scan` ile sensör frekansını görmek

**Takılırsan:** `install/README.md` içinde hibrit ekran kartı notu var
(bu makinede Intel + NVIDIA var; Gazebo akıcı değilse oraya bak).

---

## Gün 2 — ROS 2'nin beş kavramı

Bütün gün tek soruya ayrılıyor: **düğümler nasıl konuşuyor?**

Önce çalışan sistemi "kurcala":

```bash
ros2 node list                          # hangi düğümler var
ros2 node info /mergen_avoider          # neyi dinliyor, neyi yayınlıyor
ros2 topic info /scan --verbose         # tip + QoS
ros2 topic echo /mergen/state           # kararın gerekçesini canlı oku
ros2 param list /mergen_avoider
ros2 param get /mergen_avoider max_linear_speed
ros2 run rqt_graph rqt_graph            # düğüm-topic grafiğini GÖR
```

Sonra **canlı parametre değiştir** ve davranışın değiştiğini gör:

```bash
ros2 param set /mergen_avoider max_linear_speed 0.10    # yavaşlat
ros2 param set /mergen_avoider w_clearance 4.0          # daha korkak
ros2 param set /mergen_avoider w_smooth 0.0             # HİSTEREZİSİ KAPAT
```

Son komut önemli: `w_smooth = 0` yapıp robotun **titremeye** başladığını
izle. Bu, Gün 3'te okuyacağın `w_smooth` parametresinin neden var olduğunu
10 saniyede anlatır. Sonra `0.7`ye geri al.

**Okuma:** `docs/03-terimler-sozlugu.md` Bölüm 1. Ezberleme — yukarıdaki
komutları çalıştırırken tabloya bakarak eşleştir.

**Günün sonunda anlatabilmelisin:** node, topic, message, parameter, QoS.

---

## Gün 3 — Otonomi yığını

Bugün kod okuma günü. Sıra önemli, **bu sırayla oku:**

1. `mergen_avoidance/params.py` — bütün eşikler burada, birimleriyle.
   *Önce bunu oku: diğer dosyalar bu sayıların hikâyesi.*
2. `mergen_avoidance/perception.py` — LaserScan → engel aralıkları
3. `mergen_avoidance/planner.py` — aday rota + durum makinesi
4. `mergen_avoidance/control.py` — hız profili + ivme + kinematik
5. `mergen_avoidance/avoider_node.py` — ROS kabuğu (en son! en az ilginç olan)

**Okuma:** `docs/02-otonom-arac-nasil-yurur.md` — baştan sona.
Bu doküman kaptanın favori sorusunun cevabı.

**Pratik:** ROS'suz, saniyeler içinde kosan testleri çalıştır:

```bash
cd ros2_ws/src/mergen_avoidance
python3 test/run_tests.py -v
```

50 testin isimlerini oku. Her test adı bir tasarım kararını ya da gerçekten
yaşanmış bir hatayı anlatıyor — bu, kodu okumanın en hızlı yolu.

**Günün sonunda anlatabilmelisin:** algılama / konumlama / planlama / karar /
kontrol ayrımı ve her birinin bu depoda hangi dosya olduğu.

---

## Gün 4 — Şartname ve puanlama

Bugün "mühendislik" değil "kural" günü. Ama en çok puan getiren gün.

**Okuma:** `docs/01-sartname-ozeti.md`, sonra orijinali:
`docs/sartnameler/2026_robotaksi_sartname.txt`.

Puanlama tablosunu **ezberle değil, stratejik oku:**
- Hangi kalem en çok puan? (tünel +200, park +80, yolcu +70/nokta)
- Hangi hata diskalifiye? (çarpma, sakınmama, uzaktan bağlantı)
- Hangi hata en sinsi? (şerit ihlali: −50 **ve her 10 sn yeni ihlal**)

**Pratik:** şu soruyu kendine sor ve cevabını yaz: *"Bu depodaki kod bugün
yarışmaya girse hangi puanları alır, hangilerini alamaz?"*

Alır: statik engelden sakınma (+50), dinamik engele çarpmama (+50).
Alamaz: ışık, tabela, şerit, park, görev noktaları — çünkü kamera ve global
planlayıcı yok. Bu liste `docs/02`'deki yol haritası tablosu.

Bu cevabı hazırlamak, kaptanın "peki eksik ne?" sorusuna hazır olmak demek.

---

## Gün 5 — Değiştirerek öğren

Bugün kodu **bozup düzelt.** Öğrenme buradan gelir. Her deney için:
değiştir → `python3 test/run_tests.py` → Gazebo'da izle → geri al.

| Deney | Beklenen | Ne öğretiyor |
|---|---|---|
| `clean_scan` içinde `inf`'i atmayı dene (`continue`) | robot boş koridoru duvar sanar, kilitlenir | `inf` = boş alan |
| `robust_k = 1` yap | tek gürültülü ışın robotu durdurur | gürbüzleştirme |
| `blocked_intervals`'ta şişirmeyi kaldır (`half = 0`) | robot engele omuz atar | C-space |
| `w_smooth = 0` | karar titremesi | histerezis |
| `max_linear_accel = 100` | komut zıplar, tekerlek kayar | ivme limiti |
| `blocked_wait = 0` | yaya gelince araç 180° döner | "bekle mi, ara mı" |
| `bicycle_mode = true` | yerinde dönüş yok; bazı senaryolar çözülemez | Ackermann ≠ diferansiyel |

Son satır en değerli olanı. Çalıştır:

```bash
python3 scripts/sim2d_report.py --ackermann
```

ve `docs/test-sonuclari-ackermann.md`'yi oku. Bu sonuç kaptanla konuşacağın
en güçlü malzeme: *"simülasyonda çalışan kod gerçek araçta çalışmaz, ölçtüm."*

**Yeni senaryo ekleme pratiği** (15 dakika, çok öğretici):

```bash
# 1. sim2d.py içindeki scenario() fonksiyonuna yeni bir isim ekle
# 2. dünyayı üret:
python3 scripts/generate_worlds.py w9_benim_senaryom
# 3. Gazebo'da koş:
bash scripts/run_scenario.sh w9_benim_senaryom
```

Tek yerde tanımlayıp hem 2B hem 3B test alabildiğini görmek, "tek gerçek
kaynak" (single source of truth) fikrini somutlaştırır.

---

## Gün 6 — Ölçüm ve kanıt

Kaptanı etkileyen şey iddia değil, **sayı**.

```bash
bash scripts/batch_test.sh 60                 # 8 senaryo, penceresiz
python3 scripts/plot_run.py results/*/*.csv   # rapor grafikleri
python3 scripts/sim2d_report.py --out docs/test-sonuclari.md
```

Elde edeceğin şeyler:
- Her senaryo için CSV log (şartname: *"temel verileri kaydetmesi talep
  edilebilir"*)
- Yörünge / hız profili / engel mesafesi / karar grafikleri
- Markdown sonuç tablosu

**Ekran kaydı al.** 2–3 dakikalık, sesli anlatımlı bir kayıt:
`w2_cone_slalom` → `w4_lane_block` → `w8_pedestrian` (RViz açık, karar
okunabilir olsun). Bu kayıt hem kaptanla görüşmede hem ileride simülasyon
videosu teslimine temel olur.

---

## Gün 7 — Anlatmaya hazırlık

Kod yazma yok. Bugün **sunum** günü.

1. `docs/05-kaptanla-kod-gezintisi.md` dosyasını baştan sona oku —
   kaptanın soracağı soruların cevapları orada hazır.
2. Üç cümleyi ezberle (kelimesi kelimesine değil, fikir olarak):
   - *"Karar mantığını ROS'tan ayırdım; 50 birim testi Gazebo açmadan
     saniyeler içinde koşuyor."*
   - *"Üç gerçek hata buldum ve düzelttim: dönüş yönü titremesi,
     yaya gelince 180° dönme, yanal yaklaşan engeli geç görme."*
   - *"Ackermann kinematiğiyle test ettim; bazı senaryolar çözülemiyor,
     sebebini ve çözümünü biliyorum."*
3. **Eksikler listesini** hazırla (`docs/02` yol haritası tablosu).
   Eksiğini bilen, bilmeyenden daha güvenilirdir — ve o tablo aynı zamanda
   *"takımda şu işi alabilir miyim"* listesidir.
4. Repo'yu GitHub'a koy (bkz. `README.md` → "Repoyu paylaşmak").

---

## Hafta boyunca arkada açık kalacak şeyler

- **Günlük tut.** Her gün 5 satır: ne denedim, ne oldu, ne öğrendim.
  Kaptanla konuşmada bu günlük senin en güçlü kanıtın.
- **Hata mesajını kopyala, atma.** Çözdüğün her hatayı not et; aynı hata
  takımda başka birinin de karşısına çıkacak ve cevabı sende olacak.
- **Terimleri yüksek sesle kullan.** "Engelden kaçma" değil
  **"yerel planlama"**; "titreme" değil **"karar titremesi / limit çevrimi"**.

## Yapma

- **Nav2'yi hafta içinde kurcalamaya kalkma.** Güçlü ama büyük; şu an
  anlamadığın bir yapılandırma katmanı eklemek öğrenmeyi yavaşlatır.
  Gün 3'te ne olduğunu bil, kullanmayı sonraya bırak.
- **Derin öğrenmeye girme.** Bu hafta geometri ve kontrol haftası.
  YOLO ile tabela tanıma değerli ama sıra onda değil.
- **Kodu "güzelleştirmek" için yeniden yazma.** Çalışan ve test edilmiş
  bir temeli anlamadan değiştirmek, bir hafta sonunda elinde çalışmayan
  bir şey bırakır.
