# 2B Simulator Test Sonuclari

Kinematik: **diferansiyel surus**  |  hedef: x >= 9 m  |  sure siniri: 75 s  |  kontrol: 20 Hz

Bu tablo `python3 scripts/sim2d_report.py` ile uretildi. Gazebo gerektirmez;
ayni engel yerlesimleri `worlds/*.world` dosyalarina da bu senaryolardan uretilir
(`scripts/generate_worlds.py`), dolayisiyla 2B ve 3B testler ayni parkuru olcer.

| Senaryo | Ne test ediyor | Sonuc | Carpisma | En yakin yaklasma | Yol | Sure | Maks yan sapma | Baskin durumlar |
|---|---|---|---|---|---|---|---|---|
| `w1_empty` | Bos alan -- taban cizgisi (hiz, duz gitme) | **BASARILI** | yok | - | 9.01 m | 41.4 s | 0.10 m | CRUISE %100 |
| `w2_cone_slalom` | Koni slalomu -- art arda yon degistirme, histerezis | **BASARILI** | yok | 0.26 m | 9.07 m | 43.9 s | 0.45 m | CRUISE %68, AVOID %20, SLOW %12 |
| `w3_corridor` | Koridor + variller -- serit disiplini | **BASARILI** | yok | 0.42 m | 9.02 m | 41.6 s | 0.14 m | CRUISE %80, SLOW %16, AVOID %4 |
| `w4_lane_block` | Seridi kismen kapatan statik engel (sartname: statik engel) | **BASARILI** | yok | 0.49 m | 9.01 m | 41.5 s | 0.24 m | CRUISE %85, SLOW %15 |
| `w5_dead_end` | Kisa cikmaz koridor -- etrafindan dolasma | **BASARILI** | yok | 0.26 m | 10.52 m | 54.1 s | 1.82 m | CRUISE %72, AVOID %16, SLOW %10 |
| `w6_dense_clutter` | Yogun dagilmis koniler -- surekli yeniden planlama | **BASARILI** | yok | 0.26 m | 9.30 m | 44.9 s | 1.54 m | CRUISE %69, AVOID %21, SLOW %10 |
| `w7_trap_pocket` | Kapali cep -- BILINEN SINIR (yerel minimum) | **BEKLENEN (kapan farkedildi)** | yok | 0.82 m | 6.07 m | 75.0 s | 0.18 m | BLOCKED %41, CRUISE %22, RECOVER %20 |
| `w8_pedestrian` | Aniden karsiya gecen yaya (sartname: dinamik engel) | **BASARILI** | yok | 0.28 m | 9.00 m | 45.0 s | 0.10 m | CRUISE %90, SLOW %6, BLOCKED %4 |

**Ozet:** 8 senaryo, 0 carpisma.

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
