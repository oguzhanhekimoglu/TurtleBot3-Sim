# TEKNOFEST Robotaksi — Şartname Özeti (2022 → 2026)

Kaynak PDF'ler ve metne çevrilmiş halleri `docs/sartnameler/` içinde.
Bu özet dört yılın şartnamesini karşılaştırarak **neyin değiştiğini** ve
**2026'da neyin puanlandığını** çıkarıyor.

---

## Yarışma neyi ölçüyor?

> "Robotaksiden gerçek bir sürücünün davranışlarını taklit ederek tanımlanan
> senaryoları eksiksiz uygulaması; trafik kurallarına uyarak, belirtilen GPS
> noktalarına doğru kafa açılarıyla ulaşarak, güvenli ve hatasız bir sürüş
> deneyimi sergilemesi beklenmektedir." — 2026 şartnamesi

Anahtar kelime **"gerçek bir sürücü gibi"**. Hakemler "engelden kaçtı mı"
diye değil, **"bir insan sürücü böyle mi yapardı"** diye bakıyor:

> "Araçların engeller karşısındaki davranışları, insan sürücülerle
> kıyaslanarak değerlendirilir." — 2026, Engeller bölümü

Pratik sonucu: ani/sert manevra, gereksiz duraklama, çok erken sakınma
**puan kaybettirir** — teknik olarak çarpışmamış olsan bile.
Bu yüzden bu depodaki ivme limiti ve histerezis "süsleme" değil, doğrudan
puanlama kalemi.

---

## İki kategori

| | Özgün Araç | Hazır Araç |
|---|---|---|
| Araç | Takım kendi aracını üretir | Organizasyon aracı verir (BEEMOBS platformu) |
| Takım | en az 5, en fazla 20–25 kişi | en az 3, en fazla 20–25 kişi |
| Odak | mekanik + elektronik + yazılım | **tamamen yazılım/otonomi** |
| Ek aşama | — | Simülasyon videosu + araç eğitim kampı |

Yazılım/otonomi tarafında çalışacaksan **Hazır Araç** kategorisi senin
odağın: araç hazır geliyor, bütün iş otonomi yazılımında.

---

## 2026 takvimi (referans — bir sonraki yıl için şablon)

| Aşama | Tarih |
|---|---|
| Son başvuru | 28.02.2026 |
| **TYF** (Teknik Yeterlilik Formu) | 26.03.2026 — 17:00 |
| TYF sonuçları | 08.04.2026 |
| **KTR** (Kritik Tasarım Raporu) | 15.05.2026 — 17:00 |
| KTR sonuçları | 04.06.2026 |
| Simülasyon videosu (Hazır Araç) | 23.06.2026 |
| Araç test videosu (Özgün Araç) | 15.07.2026 |
| Finalistler | Haziran–Temmuz 2026 |
| Test dönemi | Ağustos 2026 |
| **Final** | **23–26.08.2026, Bilişim Vadisi (Kocaeli)** |

> 2026 finali geçti. Bir sonraki yıl için şablon aynı kalıyor: **Şubat
> başvuru → Mart TYF → Mayıs KTR → Haziran simülasyon → Ağustos final.**
> Yani ekibin asıl teslim baskısı **Mart–Mayıs**'ta. Senin bugün
> yapabileceğin en değerli şey, o raporlara girecek malzemeyi şimdiden
> üretmeye başlamak.

---

## Puanlama (2026 — final değerlendirmesi)

Bu tablo şartnameden birebir çıkarıldı. **Cebirsel olarak okumak önemli:**
bazı kalemler negatif, bazıları doğrudan diskalifiye.

| Görev | Koşul | Puan |
|---|---|---|
| **Kırmızı ışıkta durma** | 0 < mesafe < 5 m (tolerans içinde) | **+60** |
| | mesafe > 5 m (erken durdu) | +20 |
| | kırmızı ışıkta geçti | **−30** |
| **Yeşil ışıkta hareket** | 5 sn içinde | +40 |
| | 5–30 sn arası | +20 |
| | 30 sn içinde hareket etmedi | −20 |
| **Park bölgesine ulaşma** | noktadaki şeritten geçti | +20 |
| **Park görevi** | başarıyla tamamladı | **+80** |
| | eksik park (2 teker şerit dışı) | +20 |
| | uygun olmayan yere park | −20 |
| **Şerit ihlali** | ihlal yapmadan ilerledi | **+50** |
| | her ihlal (2 teker tamamen dışarı) | **−50** |
| **Dinamik engel** | sakındı / bekledi / hızını ayarladı | **+50** |
| | **çarptı** | **TURDAN DİSKALİFİYE** |
| **Statik engel** | güvenli şekilde sakındı | **+50** |
| | **sürterek** sakındı | **−10** |
| | sakınmadı / çarptı | **TURDAN DİSKALİFİYE** |
| **Trafik işaretleri** | tümüne uygun hareket etti | +50 |
| | her işaret/tabela ihlali | **−50** |
| **Görev noktaları** | her noktadan geçti (±1 m) | +30 / nokta |
| **Yolcu alma/indirme** | doğru nokta (±1 m), 15–20 sn duraklama | **+70 / nokta** |
| | hatalı veya yapmadı | −20 |
| **Tünelden geçme** | geçti (opsiyonel, bir kez) | **+200** |

### Bu tablodan çıkan stratejik okuma

1. **Tünel +200 puan** — tek kalemde en yüksek. Opsiyonel ama devasa.
2. **Çarpmak diskalifiye.** Yani "hızlı bitir" değil, **"çarpma"** birincil
   hedef. Tüm parametre seçimlerinde güvenlik tarafına yanaşmak doğru karar.
3. **Şerit ihlali −50 ve her 10 saniyelik periyot yeni bir ihlal.**
   Şartname: "şerit dışında kesintisiz 95 saniyelik bir hareket **10
   ihlaldir**" → −500 puan. Şerit disiplini tek tek görevlerden daha değerli.
4. **Yolcu alma/indirme +70/nokta** — en verimli pozitif kalem. Ama
   ±1 m tolerans ve 15–20 sn duraklama şartı var; bu bir **konumlama
   hassasiyeti** problemidir (bkz. RTK tartışması).
5. **Ödül için minimum koşul:** "en az 2 turdan ihlalsiz bir şekilde park
   bölgesine giriş yapmak." Yani **tutarlılık** tek seferlik parlaklıktan
   değerli.

---

## Kural ihlali tanımları (2026)

- **Işık İhlali** — kırmızıda geçmek, tolerans dışında durmak, yeşilde
  durmak/hareket etmemek. *Not: kırmızıda geçip sonra dursa bile "geçti"
  sayılır ve ayrıca "yeşilde hareket etmedi" olarak da puanlanır — çift ceza.*
- **Şerit İhlali** — aracın herhangi **2 tekerleğinin tamamen** şeritten
  çıkması. (Engelden sakınma ve dönüş manevraları hariç.)
- **Ters Yön İhlali** — manevra dışında trafik akışına ters hareket.
- **Engel İhlali** — statik veya dinamik engele **çarpma veya sürtme**.
- **Park İhlali** — uygun olmayan yere park veya iki tekerin şerit dışında
  kalması.
- **Trafik İşareti İhlali** — tabelaları ihlal etmek.
- **Durma İhlali** — duraklama noktalarında durmamak **veya gereksiz yere
  uzun süre durmak**. *(Yani aşırı temkinli olmak da cezalı.)*

### Doğrudan diskalifiye

> "Sakınma yapılmaması, çarpılması, kazaya sebebiyet vermek, **uzaktan
> bağlantı kurmak**..."

"Uzaktan bağlantı kurmak" maddesi önemli: start verildikten sonra araca
müdahale yok. Araç pistte **tamamen yalnız** çalışacak.

---

## Parkur ve engeller

**Pist:** tam ölçekli şehir içi parkur. Şeritler, trafik işaretleri,
trafik ışıkları, yaya geçitleri, park alanı (en az 3 araç kapasiteli, dik
park), opsiyonel tünel. Çevrede güvenlik bariyerleri — **konumları yarışma
anında değişebilir**.

**Hava/ışık koşulları değişken:** "yağmur, sis, güneş", "gölge".
Kamera tabanlı algılamanın bunlara dayanıklı olması gerekiyor —
lidar'ın öne çıktığı nokta burası.

**Statik engeller:** plastik yol bariyerleri, farklı boyutlarda trafik
konileri. Şeridi **kısmen veya tamamen** kapatabilir.
> "Eğer ilgili şerit veya diğer şerit üzerinde sakınmayı sağlayacak alan
> mevcut değilse aracın o yol üzerinden geçmemesi gerekmektedir."

Yani "geçemiyorsan geçme" de geçerli bir karardır — zorlamak diskalifiye.

**Dinamik engeller:** "ani ve rastgele", "aniden önüne çıkabilir",
hızı/yönelimi/ölçüsü değişken. İzin verilen tepkiler: sakınmak, **beklemek**,
tamamen durmak, hızı düşürmek. Yaya geçidi konumu dinamik güncellenebilir.

> Bu depodaki `w8_pedestrian` senaryosu ve `pedestrian_node.py` tam olarak
> bunu modelliyor; "dur ve bekle" davranışı da bu maddeden çıktı.

**Engel boyutları standart DEĞİL:**
> "Kullanılacak engellerin boyutları, şekilleri ve düzenlemeleri belirli bir
> standarda sahip olmayacağından, araçların bu çeşitlilik karşısında adapte
> olabilmesi beklenir."

Bu, **şablon eşlemeye (template matching) dayalı algılamayı eler.** Geometrik
yaklaşım (lidar'dan "burada bir şey var, şu kadar geniş") şart.

---

## GEOJSON — görev tanımı nasıl geliyor?

Yarışma günü, o tura ait tüm konum bilgileri GEOJSON dosyasıyla veriliyor:

```json
{
  "type": "FeatureCollection",
  "features": [
    { "type": "Feature",
      "properties": { "name": "start", "description": "Araç başlangıç konumu", "nokta_id": 0 },
      "geometry": { "type": "Point", "coordinates": [29.509014, 40.790343] } },
    { "type": "Feature",
      "properties": { "name": "gorev_1", "description": "1. görev noktası", "nokta_id": 0 },
      "geometry": { "type": "Point", "coordinates": [29.50861, 40.789785] } },
    { "type": "Feature",
      "properties": { "name": "park_giris", "description": "Araç otopark giriş bölgesi noktası", "nokta_id": 0 },
      "geometry": { "type": "Point", "coordinates": [29.509223, 40.790149] } }
  ]
}
```

Dikkat edilecek üç nokta:

1. **Koordinat sırası `[boylam, enlem]`** (GeoJSON standardı) — GPS
   kütüphanelerinin çoğu `(enlem, boylam)` bekler. Bu klasik bir hata kaynağı.
2. Konumlar **aracın ön ucunun en uç noktasını** ve **yönünü** ifade eder —
   araç merkezini değil.
3. Park giriş noktası **park edilecek yeri değil**, park bölgesinin girişini
   tanımlar. Uygun yeri araç kendisi bulacak.

> 2025'te bu dosya JSON, 2026'da **GEOJSON** olarak tanımlandı. Ayrıca
> 2026'da her tur için ayrı bir şema paylaşılacağı belirtiliyor.

---

## Yıllar arası değişim — neyin trendi var?

| | 2022–2023 | 2024–2025 | 2026 |
|---|---|---|---|
| Görev tanımı | Sabit: başla → ilerle → park et | Senaryo tabanlı (yolcu al/indir) | Senaryo + **GEOJSON** + tur başına değişken |
| Konum bilgisi | yok / pist planı | JSON kritik noktalar | **GEOJSON**, tur başına şema |
| Şerit ihlali | 1 teker çıkınca **yarış hakkı biter** | — | 2 teker, **−50/ihlal**, 10 sn periyotlu |
| Ani fren | ayrı "Ani Fren Görevi" | — | dinamik engel maddesine gömüldü |
| Dinamik engel | sınırlı | var | **ayrıntılı tanım + diskalifiye yaptırımı** |
| Tünel | yok | yok | **+200 opsiyonel** |
| Teknik şartname | şartname içinde | **ayrı doküman olarak sonra** | ayrı |

**Okunan trend:** yarışma her yıl **daha az önceden bilinebilir**,
**daha çok gerçek trafik** oluyor. Sabit bir parkuru ezberleyen çözümler
değersizleşiyor; **genel amaçlı algılama + karar** yetenekleri değer kazanıyor.

> Dikkat: 2026 ana şartnamesi puanlamayı veriyor ama "**Teknik Şartname
> daha sonra duyurulacaktır**" diyor; durma mesafesi, frenleme tepki süresi,
> UMS (Uzaktan Acil Müdahale Sistemi) gereksinimleri ve JSON şema detayları
> o dokümanda. Takıma katıldığında ilk soracağın şeylerden biri bu olmalı:
> *"Teknik şartname çıktı mı, elimizde mi?"*

---

## Hazır Araç platformu — donanım ve yazılım

2026 "Hazır Araç Mimari Tanımlama Dokümanı"ndan:

**İşlem birimi:** Advantech MIC-770V3H endüstriyel Box PC
— Intel Core i9-12900TE (16 çekirdek / 24 iş parçacığı), 32 GB RAM, 1 TB SSD
— MIC-75G20 GPU genişletme modülü + **NVIDIA RTX 3060 (12 GB)**

**Sensörler:**

| Sensör | Model | Önemli özellikler |
|---|---|---|
| Lidar | **Velodyne VLP-16** | 16 kanal, 100 m, ±3 cm, dikey FOV 30° (+15°/−15°), yatay 360°, yatay çözünürlük 0.1°–0.4°, 5–20 Hz |
| Kamera | **ZED2** stereo | RGB + derinlik haritası |
| GPS/IMU | **Xsens MTI-680-DK** | GPS L1C/A+L2C, GLONASS, Galileo, BeiDou; **RTK olmadan yatay doğruluk 1.5 m** |
| Ağ | WAVLINK AC1200 | PoE, dış ortam AP |

**Yazılım yığını:**

- İşletim sistemi: **Ubuntu 20.04 LTS (focal)**
- **ROS1 Noetic ve ROS2 Foxy ikisi birden kurulu**, seçimli başlatılıyor
- Her şey **Docker** içinde çalışıyor
- Sensör + CAN'ı başlatmak tek komut:
  - ROS1: `roslaunch smart_can_stuff start_can_bus.launch`
  - ROS2: `ros2 launch smart_can_stuff can_launch.xml`
- ROS1/ROS2 geçişi `~/.bashrc` içindeki `source` satırlarını yorumlayarak
  yapılıyor (dokümanda böyle anlatılıyor)

**Araç kontrolü CAN üzerinden, ROS topic'leriyle sarmalanmış.** Gerçek
topic isimleri ve kritik detaylar (PID devreden çıkma, heartbeat) için
bkz. `docs/02-otonom-arac-nasil-yurur.md`.

> **Bizim için pratik sonuç:** araçta **Foxy** var, bu makinede **Humble**
> kuruyoruz (Ubuntu 22.04'te Foxy binary'si yok). `rclpy` API'si ikisinde
> birebir aynı; bu depodaki kod iki dağıtımda da çalışır. Gerçek araca
> geçerken değişen tek şey **topic isimleri**.

---

## Raporlar — puan buradan da geliyor

Üç aşama: **TYF → KTR → Araç Test Videosu / Simülasyon + Sunum.**

**Simülasyon Raporu Şablonu (2026)** — maksimum 20 sayfa, ve şartname
açıkça diyor: *"Burada listelenen başlıklar üzerinden puanlama
sağlanacaktır."* İstenen başlıklar:

1. Kapak, içindekiler, şekil/tablo listesi
2. Giriş ve kapsam
3. **Simülasyon ortamı ve yazılım altyapısı** — platform seçimi ve *nedeni*
4. **Araç modeli ve sensörler** — fiziksel parametreler, dinamik model,
   sensör menzil/çözünürlük/gürültü; *KTR ile tutarlı olmalı, fazladan sensör olmamalı*
5. **Görev tanımı ve senaryolar** — varyasyonlar: hız limiti, hava, gündüz/gece
6. **Algoritma ve yazılım mimarisi** — 7.1 Algılama, 7.2 Konumlandırma,
   7.3 Planlama, 7.4 Karar Verme (durum makinesi!), 7.5 Kontrol
7. **Simülasyon testleri ve sonuçları** — test metodolojisi, başarı
   kriterleri, **tablo ve grafikler**, **log kayıtları**
8. Karşılaştırma ve analiz, iyileştirmeler
9. Sonuç ve gelecek çalışmalar
10. Ekler: parametre listesi, kod parçacıkları, kurulum notları, **repo adresi**

> Bu depo tam olarak bu şablona göre kurgulandı:
> - 4 → `worlds/` + TurtleBot3 model parametreleri
> - 5 → 8 senaryo (`w1`–`w8`), statik + dinamik engel varyasyonları
> - 6 → `perception.py` / `planner.py` / `control.py` (isimler şablonla aynı)
> - 7 → `metrics_logger.py` (CSV log), `scripts/plot_run.py` (grafikler),
>        `docs/test-sonuclari.md` (tablo)
> - 10 → `config/avoider.yaml` (parametre listesi), `install/` (kurulum notları)

---

## Kaynaklar

Resmî dokümanlar (indirilmiş halleri `docs/sartnameler/` içinde):

- [2026 Robotaksi Şartnamesi](https://cdn.teknofest.org/media/upload/userFormUpload/2026_Robotaksi_Binek_Otonom_Arac_Yar%C4%B1smas%C4%B1_Sartnamesi_SvtF2.pdf)
- [2026 Hazır Araç Mimari Tanımlama](https://cdn.teknofest.org/media/upload/userFormUpload/2026_Robotaksi_Binek_Otonom_Ara%C3%A7_Yar%C4%B1%C5%9Fmas%C4%B1_Haz%C4%B1r_Ara%C3%A7_Mimari_Tan%C4%B1mlama_Dok.pdf)
- [2026 Hazır Araç Kullanıcı Dokümanı](https://cdn.teknofest.org/media/upload/userFormUpload/2026_Robotaksi_Binek_Otonom_Ara%C3%A7_Yar%C4%B1%C5%9Fmas%C4%B1_Haz%C4%B1r_Ara%C3%A7_Kullan%C4%B1c%C4%B1_Dok%C3%BCman%C4%B1_b.pdf)
- [2025 Şartname](https://cdn.teknofest.org/media/upload/userFormUpload/2025_Robotaksi_Binek_Otonom_Arac_Yarismasi_Sartnamesi_sIYmR.pdf)
- [2024 Şartname v1.1](https://cdn.teknofest.org/media/upload/userFormUpload/Robotaksi_Binek_Otonom_Arac_Yarismasi_v1.1_442g9.pdf)
- [2023 Şartname](https://cdn.teknofest.org/media/upload/userFormUpload/Robotaksi_Binek_Otonom_2023_v1_XTjlV.pdf)
- [Yarışma sayfası](https://www.teknofest.org/tr/yarismalar/robotaksi-binek-otonom-arac-yarismasi/)
- [KGM Trafik İşaretleri El Kitabı 2015](https://www.kgm.gov.tr/SiteCollectionDocuments/KGMdocuments/Trafik/IsaretlerElKitabi/TrafikIsaretleriElKitabi2015.pdf) — şartname tabelalar için bunu referans alıyor
