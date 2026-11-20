# 2B Simulator Test Sonuclari

Kinematik: **Ackermann (bicycle_mode=True)**  |  hedef: x >= 9 m  |  sure siniri: 75 s  |  kontrol: 20 Hz

Bu tablo `python3 scripts/sim2d_report.py` ile uretildi. Gazebo gerektirmez;
ayni engel yerlesimleri `worlds/*.world` dosyalarina da bu senaryolardan uretilir
(`scripts/generate_worlds.py`), dolayisiyla 2B ve 3B testler ayni parkuru olcer.

| Senaryo | Ne test ediyor | Sonuc | Carpisma | En yakin yaklasma | Yol | Sure | Maks yan sapma | Baskin durumlar |
|---|---|---|---|---|---|---|---|---|
| `w1_empty` | Bos alan -- taban cizgisi (hiz, duz gitme) | **BASARILI** | yok | - | 9.01 m | 41.4 s | 0.11 m | CRUISE %100 |
| `w2_cone_slalom` | Koni slalomu -- art arda yon degistirme, histerezis | **BASARILI** | yok | 0.26 m | 9.10 m | 49.4 s | 0.64 m | CRUISE %59, AVOID %36, SLOW %5 |
| `w3_corridor` | Koridor + variller -- serit disiplini | **BASARILI** | yok | 0.40 m | 9.01 m | 41.8 s | 0.12 m | CRUISE %78, SLOW %16, AVOID %6 |
| `w4_lane_block` | Seridi kismen kapatan statik engel (sartname: statik engel) | **BASARILI** | yok | 0.48 m | 9.01 m | 41.5 s | 0.23 m | CRUISE %85, SLOW %15 |
| `w5_dead_end` | Kisa cikmaz koridor -- etrafindan dolasma | **KISMEN (hedefe varmadi)** | yok | 1.20 m | 4.25 m | 75.0 s | 0.02 m | CRUISE %59, BLOCKED %21, RECOVER %20 |
| `w6_dense_clutter` | Yogun dagilmis koniler -- surekli yeniden planlama | **BASARISIZ (carpisma)** | VAR | 0.19 m | 9.65 m | 60.1 s | 2.39 m | CRUISE %61, AVOID %38, SLOW %1 |
| `w7_trap_pocket` | Kapali cep -- BILINEN SINIR (yerel minimum) | **BEKLENEN (kapan farkedildi)** | yok | 0.94 m | 7.17 m | 75.0 s | 0.06 m | CRUISE %37, BLOCKED %33, RECOVER %30 |
| `w8_pedestrian` | Aniden karsiya gecen yaya (sartname: dinamik engel) | **BASARILI** | yok | 0.28 m | 9.00 m | 44.9 s | 0.11 m | CRUISE %90, SLOW %6, BLOCKED %4 |

**Ozet:** 8 senaryo, 1 carpisma.

Notlar:

- `w7_trap_pocket` kasitli olarak cozulemez bir parkurdur. Salt tepkisel
  (reactive) bir yerel planlayici yerel minimumdan kendi basina cikamaz; bu
  yontemin tanimli sinirudir, kusuru degil. Beklentimiz carpmamak ve kapani
  fark etmek. Cozum global planlayici (Nav2 / GEOJSON rotasi) eklemektir.
- `w8_pedestrian` senaryosunda sure uzamasi BEKLENIR: arac yaya gecene kadar
  durup bekler. Sartname puanlamasi tam olarak bunu odullendirir
  ('engel tamamen gecene kadar bekledi -> +50').
- 'En yakin yaklasma' robot MERKEZINDEN engel YUZEYINE olan mesafedir.
  Govde yaricapi ~0.22 m oldugundan 0.22 m'nin altindaki degerler temastir.

## Ackermann kosusu hakkinda -- bu testin ANLAMI

Bu kosuda `bicycle_mode=True` ve `min_turn_radius=4.0 m` kullanildi: yani TurtleBot3 govdesine
**gercek Robotaksi aracinin donus kisiti** dayatildi. Arac artik yerinde
donemez ve `|w| <= v / R_min` kisitina tabidir.

Bu bilerek ZOR bir kurgudur ve amaci su dersi olculebilir hale getirmek:

> Diferansiyel suruste kusursuz calisan bir algoritma, yerinde donmeye
> dayaniyorsa Ackermann bir aracta calismaz.

Basarisiz olan senaryolar bir ALGORITMA hatasi degil, **kinematik
uyumsuzluktur**: 0.8 m araliklı konilerin arasindan 4 m donus yaricapiyla
gecmek fiziksel olarak imkansizdir. Gercek yarismada parkur da araca gore
olceklidir (serit genisligi ~3 m, engeller metrelerce arali).

Dogru karsilastirma icin iki secenek var:

1. Parkuru araca gore olcekle (mesafeleri ~4x buyut) -- gercege en yakini.
2. TurtleBot3 olcegine uygun bir `min_turn_radius` kullan (~0.5 m).

Buradaki tablo 1. veya 2. secenek UYGULANMADAN alinmistir; yani en kotu
durumu gosterir. Yol haritasindaki is: parkuru olcekleyip Ackermann
kosusunu yeniden almak ve gerekirse planlayiciya donus yaricapi farkindali
aday rota uretimi (arc-based sampling) eklemek.
