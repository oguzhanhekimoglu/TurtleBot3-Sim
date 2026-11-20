# Bu Projede Ne Yaptık?

> Tek dosyada her şey. Baştan sona oku, sonra lazım oldukça geri dön.
> Terim görünce panikleme — her biri ilk geçtiği yerde açıklanıyor.

---

## 0. Üç cümlelik özet

TEKNOFEST Robotaksi yarışması için **lidar ile engelden kaçan otonom bir araç
yazılımı** yaptık. Gerçek araç yerine TurtleBot3 adlı küçük robotun Gazebo
simülasyonunda çalışıyor. 9 farklı parkurda test edildi, hiçbirinde çarpmadı —
ve yol boyunca **10 tane gerçek hata** bulup düzelttik.

Önemli olan son cümle. Çalışan kod herkeste var; **neyin neden bozulduğunu
bilmek** seni ayıran şey.

---

## 1. Önce kafandaki soruyu çözelim: otonom araç nasıl yürür?

Bir döngüdür. Saniyede 10–50 kez şu beş adım tekrarlanır:

```
  ALGILAMA  →  KONUMLAMA  →  PLANLAMA  →  KARAR  →  KONTROL
 "ne var?"    "neredeyim?"  "nereye?"   "şimdi ne?"  "direksiyon kaç derece?"
      ↑                                                      ↓
      └──────────── araç hareket eder, dünya değişir ────────┘
```

Buna **otonomi yığını** (autonomy stack) denir. Üç şeyi aklında tut:

1. **Kapalı çevrimdir.** Araç her hareket ettiğinde dünya değişir, döngü
   baştan başlar. Boru hattı değil, daire.
2. **Hız kritiktir.** Bir katman yavaşlarsa araç *eski bir dünya resmine*
   göre gaz verir. Otonom araçtaki en tehlikeli arıza biçimi budur.
3. **Bu beş başlık tesadüf değil.** TEKNOFEST Simülasyon Raporu Şablonu'nun
   7. bölümü aynı sırayı istiyor (7.1 Algılama → 7.5 Kontrol) ve şartname
   *"burada listelenen başlıklar üzerinden puanlama sağlanacaktır"* diyor.
   Bizim dosya isimlerimiz de bu yüzden böyle.

Bizim projede hangisi var:

| Katman | Bizde | Dosya |
|---|---|---|
| Algılama | ✅ lidar | `perception.py` |
| Konumlama | ❌ yok (simülasyon odometrisi) | — |
| Planlama | ✅ yerel | `planner.py` |
| Karar | ✅ 6 durumlu makine | `planner.py` |
| Kontrol | ✅ hız + ivme + kinematik | `control.py` |

Eksikleri saklamıyoruz — bölüm 9'da hepsi listeli.

---

## 2. Kullandığımız teknolojiler

### ROS 2 — "robot işletim sistemi"

İsmi yanıltıcı: işletim sistemi değil, **programların birbiriyle konuşmasını
sağlayan ara katman**. Dört kavram yeter:

| Terim | Ne | Örnek |
|---|---|---|
| **Node** (düğüm) | Bir iş yapan program | `mergen_avoider` |
| **Topic** (konu) | İsimli mesaj kanalı | `/scan`, `/cmd_vel` |
| **Publisher / Subscriber** | Yazan / okuyan. **Birbirlerini tanımazlar** | lidar yazar, biz okuruz |
| **Message** | Kanaldan akan veri tipi | `LaserScan`, `Twist` |

Son sütundaki "birbirlerini tanımazlar" kısmı ROS'un bütün değeri. Lidar
sürücüsü bizim kodumuzu bilmez. Biz lidarı değiştirirsek kodumuz aynı kalır.

**Sürüm:** Biz **Humble** kullanıyoruz. Yarışma aracında **Foxy** var
(Ubuntu 20.04). İkisinin Python API'si birebir aynı; kod iki yerde de çalışır.

### Gazebo — fizik simülatörü

Robotu, engelleri, yerçekimini, tekerlek sürtünmesini ve **lidar ışınlarını**
hesaplar. Bizim kodumuz Gazebo'nun gerçek robot mu simülasyon mu olduğunu
bilmez — aynı topic'leri görür. Gerçek araca geçerken kod değişmemesinin
sebebi bu.

> **3B pencere sadece izleyicidir.** Simülasyon onsuz da çalışır. Bu yüzden
> toplu testleri penceresiz koşuyoruz — 8 senaryo 10 dakikada bitiyor.

### TurtleBot3 Waffle — deney robotu

ROS dünyasının standart eğitim robotu. Bizim için önemli üç özelliği:

- **360° lidar**, 3.5 m menzil, 5 Hz
- **Diferansiyel sürüş**: iki tekerin hız farkıyla döner, **yerinde dönebilir**
- Boyut ~28×30 cm

Son madde kritik — bölüm 8'de anlatıyorum.

### Diğerleri

| Araç | Ne işe yarar |
|---|---|
| **RViz** | Robotun *ne gördüğünü ve ne düşündüğünü* gösterir. Gazebo "dışarıdan kamera", RViz "robotun kafasının içi" |
| **colcon** | Derleyici (`colcon build`) |
| **rclpy** | ROS 2'nin Python kütüphanesi |
| **DDS** | Altta mesajları taşıyan katman. `ROS_DOMAIN_ID` farklıysa düğümler birbirini görmez |

---

## 3. Veri nereden nereye akıyor?

```
Gazebo
  ├─ /scan  (LaserScan: 360 mesafe)  ──┐
  └─ /odom  (konum + hız)            ──┤
                                       ▼
                        ┌──────────────────────────┐
                        │      mergen_avoider      │   20 Hz
                        │  perception → planner    │
                        │        → control         │
                        └──────────┬───────────────┘
                                   ├─► /cmd_vel        → Gazebo (robot hareket eder)
                                   ├─► /mergen/state   → "neden böyle yaptım"
                                   └─► /mergen/markers → RViz görselleri
```

`Twist` mesajının içinde sadece iki sayı var:

- `linear.x` → ileri hız (m/s)
- `angular.z` → dönme hızı (rad/s)

**Otonom aracın tamamı bu iki sayıyı üretmekten ibaret.** Gerisi "hangi iki
sayı?" sorusunun cevabı.

---

## 4. Algoritma — robot nasıl karar veriyor?

### 4.1 Algılama: lidar verisini temizle

Lidar 360 sayı gönderir. Ham hâliyle kullanılmaz, üç kural var:

```python
inf  →  "orada bir şey YOK"  →  boş alan olarak yaz, ATMA
nan  →  "ölçemedim"          →  at
r < range_min  →  gövdeye çarpan ışın  →  at
```

> **En sık yapılan hata:** `inf` değerlerini atmak. Attığın anda robot boş
> koridoru **duvar** sanar ve kilitlenir. `inf` bir hata değil, bilgidir.

Sonra iki işlem daha:

**Gürbüzleştirme.** Bir yöndeki mesafeyi bulurken en küçüğü değil, *k'ıncı*
en küçüğü alıyoruz (`robust_k=2`). Sebebi: tek bir gürültülü ışın yüzünden
robot durmasın. Gerçek bir engel onlarca ışın düşürür, yani kaçırmayız.

**C-space şişirme** — en önemli 10 satır:

```
Robot bir NOKTA değil. Yarıçapı R = 0.26 m.
d mesafesinde gördüğün engel, o yönde asin(R/d) kadar AÇIYI da kapatır.
```

```
        engel
          ●
       ／  |  ＼        ← sadece engelin olduğu yön değil,
     ／    |    ＼         gövdenin sığmayacağı tüm açılar kapalı
   ╱______|______╲
          🤖
```

Bunu yapmazsan robot "boşluk var" der, girer ve **omzuyla sürter**.
Şartnamede *"sürterek sakındı → −10 puan"* maddesi tam olarak bu hata.

### 4.2 Planlama: 91 aday, 3 terimli maliyet

Yöntemin adı **VFH** (Vector Field Histogram) ailesinden bir aday-rota şeması:

```
1. Geçilemeyen açı aralıklarını çıkar          (yukarıdaki şişirme)
2. ±90° içinde 2° adımla 91 ADAY YÖN üret
3. Her adayı üç terimle puanla:

   maliyet(θ) = w_goal      · |θ − hedef|        ← rotadan sapma
              + w_clearance · (1 − açıklık/ufuk) ← kapalılık
              + w_smooth    · |θ − önceki|       ← HİSTEREZİS

4. En ucuz adayı seç
```

**Üçüncü terim hayati.** Onu sıfır yaparsan iki eşit boşluk arasında araç
saniyede 10 kez fikir değiştirip titrer. Buna **karar titremesi** denir ve
gerçek bir arızadır. Canlı deneyebilirsin:

```bash
ros2 param set /mergen_avoider w_smooth 0.0
```

> **Neden potansiyel alan değil?** "Hedef çeker, engel iter" yöntemi basit
> ama iki sorunu var: U şeklinde çıkmaz sokakta kilitlenir (yerel minimum),
> dar koridorda iki duvarın itmesi titreme üretir. Aday-rota şeması ikisini
> de doğrudan ele alıyor.

### 4.3 Karar: 6 durumlu makine

Davranış sadece "şu an ne görüyorum"a değil, **"hangi moddayım"a** da bağlı.
Kırmızı ışıkta duran araçla engel önünde duran araç aynı komutu üretir (v=0)
ama farklı koşullarda kalkar. Bunu açıkça yazmanın yolu durum makinesidir:

| Durum | Ne zaman | Ne yapar |
|---|---|---|
| `CRUISE` | ön > 2.0 m | tam hız, hedef yönde |
| `SLOW` | ön < 2.0 m | hız kademeli düşer |
| `AVOID` | ön < 1.4 m | boşluğa yönelir, yavaş ilerler |
| `BLOCKED` | geçilebilir yön yok | **önce durup bekler**, sonra yön arar |
| `EMERGENCY` | ön < 0.25 m veya TTC kritik | ani fren |
| `RECOVER` | 3 sn ilerleme yok | geri al + dön (**arkaya bakarak**) |

RViz'de üstteki renkli yazı tam olarak bu durumu gösteriyor.

### 4.4 Dinamik engel: kapanma hızı ve TTC

Buraya kadar her şey **statik** varsayımlıydı: "şu an ne görüyorum". Ama
yaya duruyor olmaz, **yürür**. Planlayıcı her kareyi ayrı ayrı görürse
yanal yaklaşan yayayı ancak önüne girdiğinde fark eder — ki o zaman geçtir.

Çözüm: iki ardışık taramayı karşılaştır.

```
kapanma hızı = (önceki_mesafe − şimdiki_mesafe) / dt     [m/s]
TTC = mesafe / kapanma hızı                              [s]
```

**TTC** = Time To Collision, çarpışmaya kalan süre. 3 saniyenin altına
inerse hız kesmeye başla, 1.5 saniyenin altına inerse tam fren.

Gözetleme arkı ön koniden (±20°) **çok daha geniş: ±75°**. Çünkü yandan
yaklaşan yaya ön koniye girdiğinde zaten çok yakındır.

Gerçek Gazebo koşusundan, aynen:

```
[CRUISE] v=0.22  ön 3.00 m                                  yol açık
[SLOW]   v=0.18  ön 1.42 m  kapanma +0.80 m/s  çarpan 0.18  yaya belirdi
[AVOID]  v=0.15  ön 1.17 m  yön +0.15 rad
[AVOID]  v=0.10  ön 1.03 m  yön −0.52 rad
[CRUISE] v=0.21  ön 3.00 m                                  yaya geçti, devam
```

### 4.5 Kontrol: kararı tekerleğe çevir

Üç iş:

**Hız profili** — `stop_distance`'ta 0, `slow_distance`'ta tam hız, arası
lineer. Keskin dönüşte ek kesinti.

**İvme limiti** — komut zıplamaz, *sürülür*. 0'dan 0.22 m/s'ye tek karede
çıkan bir komut gerçek araçta aktüatörü doyurur ve tekerlek kaydırır.
Şartnamedeki **"ergonomi"**/yolcu konforu maddesinin karşılığı.

**Kinematik kısıt** — bölüm 8.

---

## 5. Kodun mimarisi — ve neden böyle

```
mergen_avoidance/
├── geometry.py       açı/aralık matematiği          ← ROS YOK
├── perception.py     ALGILAMA                       ← ROS YOK
├── planner.py        PLANLAMA + KARAR               ← ROS YOK
├── control.py        KONTROL                        ← ROS YOK
├── sim2d.py          2B simülatör                   ← ROS YOK
├── avoider_node.py   ROS kabuğu  ◄── rclpy SADECE burada
├── metrics_logger.py ROS kabuğu
└── pedestrian_node.py ROS kabuğu
```

**Karar mantığının tamamı ROS'tan habersiz.** Bu bir tercih ve üç somut
kazancı var:

1. **Test:** 50 birim testi Gazebo açmadan **saniyeler içinde** koşuyor.
   Gazebo 15 saniyede açılıyor — geliştirme döngüsü farkı bu.
2. **Taşınabilirlik:** gerçek araca geçerken sadece `avoider_node.py`
   değişiyor (`/cmd_vel` → `/beemobs/speed_target_value`).
3. **Sensör değişimi:** VLP-16 lidara geçmek `perception.py`'nin üst
   kısmını etkiliyor, altındaki hiçbir şeyi değil.

### Tek gerçek kaynak (single source of truth)

Aynı bilgiyi iki yerde tutmak = zamanla ayrışmak demek. Bu yüzden:

| Kaynak | Türetilen |
|---|---|
| `sim2d.py` → `scenario()` | Gazebo dünyaları (`generate_worlds.py`) |
| `params.py` → `AvoidParams` | `config/avoider.yaml` (`generate_params_yaml.py`) |

Böylece "2B'de geçti ama Gazebo'da çarptı" veya "YAML'da ayarı değiştirdim
ama hiçbir şey olmadı" saatleri yaşanmıyor.

### Parametreler

**Kodda çıplak sayı yok.** 40+ eşiğin hepsi `params.py`'de, adıyla, birimiyle
ve *neden o değer* açıklamasıyla. Çalışırken değiştirebilirsin:

```bash
ros2 param set /mergen_avoider max_linear_speed 0.15
```

---

## 6. Nasıl test ediyoruz? (üç katman)

```
┌─ 1. BİRİM TESTİ ────────── saniyeler ── ROS yok, Gazebo yok ─┐
│  50 test. Saf matematik ve karar mantığı.                    │
├─ 2. ROS KATMANI ────────── ~30 sn ───── Gazebo yok ──────────┤
│  Sahte /scan yayınla → düğüm /cmd_vel üretiyor mu?           │
│  QoS, parametre, topic adı, güvenlik bekçisi.                │
├─ 3. GAZEBO ─────────────── dakikalar ── gerçek fizik ────────┤
│  9 parkur, gerçek lidar gürültüsü, gerçek tekerlek.          │
└──────────────────────────────────────────────────────────────┘
```

Üçü de gerekli. **2. ve 3. katmandaki hataların hiçbiri 1. katmanda
görünmüyordu** — bölüm 7'de göreceksin.

### En önemli test fikri

`scripts/gazebo_report.py` içinde bir **etkileşim kontrolü** var:

> Araç hiç tepki vermediyse **ve** engel hiç yaklaşmadıysa, sonuç
> "BAŞARILI" değil **"GEÇERSİZ ÖLÇÜM"**.

Çünkü: **engeli hiç görmeyen araç da çarpmaz.** "Çarpışma yok" tek başına
hiçbir şey kanıtlamaz. Bu kontrolü gerçek bir yanlış-pozitifi yakaladıktan
sonra ekledim (bölüm 7, hata #8).

---

## 7. On hata — projenin asıl hikâyesi

Bunları anlat. Hata bulmuş ve düzeltmiş olmak, hiç hata yapmamış görünmekten
**çok daha** inandırıcıdır.

### A. Karar mantığında (2B simülatörde bulundu)

**1 — Dönüş yönü titremesi.** Kapalı bir cepte robot 46 saniye boyunca
−19° ile +2° arasında salındı, net dönüş sıfır, hiç kurtulamadı.
*Neden:* "hangi taraf daha boş" sorusunu **her karede** yeniden soruyordum;
simetrik koridorda cevap sürekli taraf değiştiriyor.
*Çözüm:* yön bir kez seçilir ve `BLOCKED`'dan çıkana kadar **kilitlenir**.

**2 — Yaya gelince 180° dönüp ters yöne gitme.**
*Neden:* "yol kapalı" iki farklı şey olabilir. Önümü bir **duvar**
kapatıyorsa yön aramalıyım; önümden biri **geçiyorsa** beklemeliyim.
Ayırmıyordum.
*Çözüm:* sahnenin ne kadar değiştiğini ölç. Hareketliyse bekle.
*Sonuç:* 45.0 s (boş koridor 41.4 s) — 3.6 saniye bekledi, çarpmadı.
Şartname: *"engel tamamen geçene kadar bekledi → +50"*.

**3 — Yanal yaklaşan engeli geç görme.** Yayaya çarpıyordu (0.00 m).
*Neden:* planlayıcı her kareyi **statik** görüyordu, hız bilgisi yoktu.
*Çözüm:* kapanma hızı + TTC (bölüm 4.4).
*Ölçüm:* 0.00 m → 0.19 m → **0.28 m**.

### B. Entegrasyonda (Gazebo ilk kez çalıştırılınca)

Hiçbiri birim testlerinde görünmüyordu. Hepsi "yapıştırma" katmanında.

**4 — Launch kapsam sızıntısı + sessiz dosya yolu hatası.** CSV, `~/mergen_results`
yerine paketin kurulum dizinine yazıldı.
*Neden (iki hata üst üste):* launch'ta verdiğim `world` argümanı üst kapsamı
ezdi ve düğüme senaryo adı yerine **tam dosya yolu** gitti; `os.path.join`
da ikinci parça absolute olduğu için çıktı dizinini **sessizce yuttu**.
*Çözüm:* `GroupAction(scoped=True)` **ve** düğümde ad temizleme.

**5 — Ctrl-C'de "dur" komutu hiç gitmiyordu.**
*Neden:* `rclpy.init` kendi SIGINT işleyicisini kurar; sinyal gelince önce
ROS context'i kapanır, sonra `finally` bloğuna düşülür — orada yayıncı artık
geçersizdir. Gazebo son `cmd_vel`'i koruduğu için robot **son komutla devam
ederdi**.
*Çözüm:* sinyal işleyicisini devralıp durdurmayı context geçerliyken yayınla.

**6 — İkinci SIGINT temizliği kesiyordu.** Ctrl-C bir kezden fazla gelir.
*Çözüm:* ilk sinyalden sonra gerisini yoksay.

**7 — YAML `y` harfini boolean'a çevirdi.** Favorim.
Launch'ta `"axis": "y"` yazmıştım. YAML 1.1'de çıplak `y` bir **boolean**'dır
(`y`/`yes`/`n`/`no`/`true`/`false`). String bekleyen parametreye `True` geldi,
yaya düğümü açılışta çöktü, **yaya hiç yola çıkmadı**.
Bu, "Norway problem" diye bilinen klasik hatanın tek harfli hali — orada da
ülke kodu `NO`, YAML tarafından `False` okunur.
*Çözüm:* değerler `lateral`/`longitudinal`.

**8 — Yaya modeli hiç kımıldamıyordu, test bunu "başarılı" sandı.**
Koşu "temas 0, BAŞARILI" verdi. Ama CSV'de tüm koşu boyunca `front = 3.00`
(ufuk bomboş) ve araç %100 CRUISE'daydı.
*Neden:* ODE fizik motoru hareketsiz gövdeleri uyku moduna alır; yaya 16 sn
beklediği için uyudu ve hız komutu onu uyandırmadı. Ölçtüm: 3 saniyelik
0.45 m/s komuttan sonra yer değiştirme **0.0001 m**.
*Çözüm:* `allow_auto_disable=false` + `kinematic=true`.
*Asıl ders:* rapor üretecine **etkileşim kontrolü** eklendi (bölüm 6).

**9 — Tetikleme geometrisi testi imkânsız kılıyordu.**
Yaya `x=4.0`'da, 0.45 m/s ile geçiyor; şerit ortasına varması 4.44 s sürüyor.
Robot 0.22 m/s. `trigger_x=3.2` ile robot oraya vardığında yaya `y=−0.36` —
**tam önünde**, 1.05 m uyarıyla. Hiçbir algoritmanın kurtaramayacağı kurgu.
*Çözüm:* geometriyi **hesapladım**, `trigger_x=1.8`.

**10 — Geri giderken arkaya bakmıyordu.** (Bunu sen buldurdun.)
Yeni parkuru test ederken robot bir koniye çarptı:
`t=83s durum=RECOVER hız=−0.08` → **geri giderken**.
*Neden:* planlayıcı sadece **öne** bakıyor (±90°). `RECOVER` ise geri gider —
yani bakmadığı yöne.
*Çözüm:* geri almadan önce arkayı kontrol et; kapalıysa geri gitme, yerinde dön.
*Sonuç:* çarpışma gitti, BLOCKED 445→84, RECOVER 222→37.
Yarışmada geri manevrada engele çarpmak **diskalifiye** — "geri giderken oldu"
diye bir mazeret yok.

### Ortak ders

| Nerede bulundu | Kaç tane | Ne tür |
|---|---|---|
| 2B simülatör | 3 | algoritma mantığı |
| Gazebo entegrasyonu | 7 | yapıştırma katmanı |

**Saf mantığı test etmek gereklidir ama yeterli değildir.** Entegrasyon
hataları ancak sistem gerçekten ayağa kalkınca görünür.

---

## 8. Simülasyondan gerçeğe: en kritik fark

| | TurtleBot3 (diferansiyel) | Robotaksi (Ackermann) |
|---|---|---|
| Yerinde dönebilir mi? | **Evet** | **Hayır** |
| Minimum dönüş yarıçapı | 0 | ~4 m |
| `v=0, w=1.0` komutu | geçerli | **fiziksel olarak imkânsız** |

Takımların simülasyondan gerçek araca geçerken düştüğü tuzak bu. **Yerinde
dönmeye dayanan bir algoritma, gerçek araçta ilk kavşakta kilitlenir.**

Bu yüzden `bicycle_mode` parametresi var ve test edildi:

```bash
python3 scripts/sim2d_report.py --ackermann
```

Sonuç: 8 senaryodan 6'sı geçiyor, 2'si geçmiyor. **Ama bu algoritma hatası
değil, kinematik uyumsuzluk**: 0.8 m aralıklı konilerin arasından 4 m dönüş
yarıçapıyla geçmek imkânsız. Gerçek parkur da araca göre ölçekli.

### İkinci örnek: güvenlik bekçisi

Lidar susarsa araç duruyor. Ölçtüm: **0.46 saniye** sonra tam duruş.
TurtleBot3'te 0.22 m/s → ~10 cm kör yol, sorun değil.
**Gerçek araçta 30 km/h'de aynı eşik 3.8 metre eder.**

> Ders: simülasyon parametreleri gerçek araca **olduğu gibi taşınamaz**.
> Her eşik araç ölçeğine göre yeniden hesaplanmalı.

---

## 9. Ne YOK? (dürüst liste)

| # | Eksik | Neden gerekli | Nasıl eklenir |
|---|---|---|---|
| 1 | **Nesne takibi** | Dinamik engelin hızını ve yolumu kesip kesmeyeceğini bilmek. Şu an cisimleri kimliklendirmiyoruz | en-yakın-komşu eşleştirme + Kalman |
| 2 | **Global planlayıcı** | Çıkmaz sokak (yerel minimum) tepkisel yöntemle çözülemez | Nav2 + costmap + A* |
| 3 | **Konumlama füzyonu** | GPS RTK'sız ±1.5 m, görev toleransı ±1 m | `robot_localization` EKF + RTK |
| 4 | **Şerit tespiti** | Şerit ihlali −50, her 10 sn yeni ihlal | kamera + perspektif dönüşümü |
| 5 | **Tabela/ışık tanıma** | Her ihlal −50. Araçta RTX 3060 var | YOLO benzeri dedektör |
| 6 | **Park manevrası** | Başarılı park +80 | boş yer tespiti + RRT* |

Bu tablo aynı zamanda **takımda hangi işi isteyebileceğinin listesi.**

---

## 10. Yarışma tarafı — bilmen gereken minimum

**İki kategori:** Özgün Araç (kendi aracını üretirsin) / **Hazır Araç**
(organizasyon verir, iş tamamen yazılım). Senin odağın ikincisi.

**Araçta ne var:** Ubuntu 20.04, ROS1 Noetic + ROS2 Foxy (Docker içinde),
**Velodyne VLP-16** lidar, **ZED2** stereo kamera, **Xsens MTi-680** GPS/IMU,
Intel i9 + **RTX 3060**.

**Üç teslim aşaması:** TYF (Mart) → KTR (Mayıs) → Simülasyon/Test videosu
(Haziran–Temmuz) → Final (Ağustos).

**Puanlamadan aklında kalması gerekenler:**

| Kalem | Puan |
|---|---|
| Tünelden geçme (opsiyonel) | **+200** |
| Park görevi | +80 |
| Yolcu alma/indirme | +70 / nokta |
| Kırmızı ışıkta doğru durma | +60 |
| Şerit ihlali yapmama | +50 |
| Statik/dinamik engelden sakınma | +50 |
| **Şerit ihlali** | **−50** (ve her 10 sn yeni ihlal) |
| **Engele çarpma** | **TURDAN DİSKALİFİYE** |

Üç stratejik okuma:

1. **Çarpmamak birincil hedef**, hız değil. Tüm parametre seçimlerinde
   güvenlik tarafına yanaşmak doğru karar.
2. **Şerit disiplini** tek tek görevlerden değerli: "95 saniyelik şerit dışı
   hareket **10 ihlaldir**" → −500 puan.
3. **Tutarlılık** parlaklıktan değerli: ödül için *en az 2 turdan ihlalsiz*
   park bölgesine girmek şart.

**Anahtar ifade:** *"gerçek bir sürücü gibi"*. Şartname aracın davranışını
**insan sürücüyle kıyaslayarak** değerlendiriyor. Yani ani manevra, gereksiz
duraklama, çok erken sakınma puan kaybettirir — çarpmamış olsan bile. Bizim
ivme limiti ve histerezis "süsleme" değil, doğrudan puanlama kalemi.

---

## 11. Komut defteri

```bash
# --- önce bu: bir şey çalışmazsa ---
bash ogren/00_kontrol.sh

# --- öğrenme merdiveni ---
bash ogren/01_kaplumbaga.sh      # düğüm, topic, mesaj
bash ogren/02_lidar_gor.sh       # lidar verisi
python3 ogren/03_ilk_dugum.py    # kendi düğümün (02 açıkken)

# --- simülasyon ---
bash scripts/run_scenario.sh w9_parkur rviz      # Gazebo + RViz
bash scripts/run_scenario.sh w9_parkur viz       # sadece RViz (hafif)
bash scripts/run_scenario.sh w8_pedestrian rviz  # yaya senaryosu

# --- test ---
cd ros2_ws/src/mergen_avoidance
python3 test/run_tests.py        # 50 birim testi, ROS gerekmez
python3 test/test_ros_node.py    # ROS katmanı, Gazebo gerekmez

# --- toplu test + rapor ---
bash scripts/batch_test.sh 60
python3 scripts/gazebo_report.py --out docs/test-sonuclari-gazebo.md
python3 scripts/plot_run.py results/*/*.csv

# --- teşhis ---
ros2 topic echo /mergen/state    # kararın gerekçesini canlı oku
ros2 node info /mergen_avoider
ros2 param set /mergen_avoider w_smooth 0.0     # histerezisi kapat → titreme
```

---

## 12. Tuzaklar defteri

Bunları bilmek saatler kazandırır.

| Tuzak | Belirti | Çözüm |
|---|---|---|
| **Asılı `gzserver`** | Gazebo "açılmıyor", hata bile yok | `pgrep -x gzserver` → `bash ogren/00_kontrol.sh` |
| **`pkill -f` kendini öldürür** | Komut çıktı vermeden ölür | `pkill -x <ad>` kullan |
| **`turtle_teleop_key` odak** | Kaplumbağa kıpırdamıyor | O terminal **odakta** olmalı |
| **YAML `y` → boolean** | `InvalidParameterTypeException` | Tırnakla veya o değeri kullanma |
| **`os.path.join` absolute yutar** | Dosya yanlış yere yazılır, hata yok | İkinci parçayı temizle |
| **QoS uyuşmazlığı** | Topic var ama mesaj gelmiyor | Sensör verisi `best effort` |
| **`inf` ≠ engel** | Boş koridoru duvar sanar | `inf` = boş alan |
| **Secure Boot + NVIDIA** | Her şey yavaş, `nvidia-smi` çalışmaz | MOK kaydı veya Secure Boot kapat |
| **apport "çökme"** | Launch hatası "çökme" görünür | Gerçek sorun değil, terminali oku |

---

## 13. Sırada ne var?

Öncelik sırasıyla:

1. **Bu dosyayı bir kez daha oku**, bu sefer kodu açarak. Sıra:
   `params.py` → `perception.py` → `planner.py` → `control.py` → `avoider_node.py`
2. **`ogren/03_ilk_dugum.py`'yi değiştir.** İçinde `DENE:` yazan üç yer var;
   sonuncusunda tek bir `#` silince robot "duran"dan "engelden kaçan"a dönüşüyor.
3. **Bir senaryo ekle.** `sim2d.py` → `scenario()` içine yeni bir isim,
   sonra `python3 scripts/generate_worlds.py w10_benim`. Tek yerde tanımlayıp
   hem 2B hem 3B test alabildiğini görmek "tek gerçek kaynak" fikrini oturtur.
4. **Yol haritasından bir madde seç** (bölüm 9) ve takımda onu iste.

---

### Diğer dokümanlar

| Dosya | İçerik |
|---|---|
| `docs/01-sartname-ozeti.md` | 2022→2026 şartname karşılaştırması, puanlama tabloları, GEOJSON |
| `docs/02-otonom-arac-nasil-yurur.md` | Otonomi yığını detaylı, gerçek araç topic'leri |
| `docs/03-terimler-sozlugu.md` | Terimler + tuzaklar |
| `docs/test-sonuclari-gazebo.md` | Gazebo ölçüm tablosu |
| `ogren/README.md` | Öğrenme merdiveni |
