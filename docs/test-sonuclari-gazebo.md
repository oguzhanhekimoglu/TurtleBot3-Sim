# Gazebo Test Sonuçları

Koşu: `results/20261117_213505` · senaryo başına 60 s · TurtleBot3 Waffle · Gazebo Classic 11 · ROS 2 Humble

Bu tablo `bash scripts/batch_test.sh` çıktısından
`python3 scripts/gazebo_report.py` ile üretildi. Gerçek fizik motoru,
gerçek lidar gürültüsü, gerçek diferansiyel sürüş eklentisi.

| Senaryo | Ne test ediyor | Sonuç | Temas | En yakın | Yol | Ort. hız | Süre | Baskın durumlar |
|---|---|---|---|---|---|---|---|---|
| `w1_empty` | Boş alan — taban çizgisi | **BAŞARILI** | 0 | — | 12.54 m | 0.22 m/s | 57.8 s | CRUISE %99, SLOW %0, ? %0 |
| `w2_cone_slalom` | Koni slalomu — art arda yön değiştirme | **BAŞARILI** | 0 | 0.26 m | 11.54 m | 0.20 m/s | 58.0 s | CRUISE %63, AVOID %35, SLOW %2 |
| `w3_corridor` | Koridor + variller — şerit disiplini | **BAŞARILI** | 0 | 0.39 m | 12.43 m | 0.21 m/s | 57.9 s | CRUISE %85, SLOW %12, AVOID %3 |
| `w4_lane_block` | Şeridi kısmen kapatan statik engel | **BAŞARILI** | 0 | 0.46 m | 12.53 m | 0.22 m/s | 58.0 s | CRUISE %90, SLOW %9, NO_SCAN %0 |
| `w5_dead_end` | Kısa çıkmaz koridor | **BAŞARILI** | 0 | 0.26 m | 11.82 m | 0.20 m/s | 57.8 s | CRUISE %86, AVOID %8, SLOW %6 |
| `w6_dense_clutter` | Yoğun koniler — sürekli yeniden planlama | **BAŞARILI** | 0 | 0.25 m | 11.91 m | 0.21 m/s | 57.9 s | CRUISE %76, AVOID %16, SLOW %7 |
| `w7_trap_pocket` | Kapalı cep — bilinen sınır (yerel minimum) | **BEKLENEN (kapan farkedildi)** | 0 | 0.89 m | 5.34 m | 0.09 m/s | 57.8 s | BLOCKED %40, CRUISE %29, RECOVER %17 |
| `w8_pedestrian` | Aniden karşıya geçen yaya — dinamik engel | **BAŞARILI** | 0 | 1.00 m | 12.25 m | 0.21 m/s | 57.5 s | CRUISE %94, AVOID %4, SLOW %1 |
| `w9_parkur` | 20 m koridor, 10 engel, dar geçit — tam parkur | **BAŞARILI** | 0 | 0.24 m | 11.30 m | 0.20 m/s | 57.7 s | CRUISE %51, AVOID %38, SLOW %11 |

**Özet:** 9 senaryo, geçerli ve temiz: 9, sorunlu: 0.

**Dinamik engel:** yaya tetiklendi mi — evet.

**Kapanış sağlığı:** toplam traceback sayısı 0 (temiz).

Notlar:

- 'En yakın' robot gövdesinden engel yüzeyine ölçülen en küçük mesafedir
  (ham lidar taramasından, `robust_k` filtresi uygulanmadan — ölçüm ile
  karar aynı filtreyi paylaşmamalı).
- `w7_trap_pocket` kasıtlı olarak çözülemez: salt tepkisel bir yerel
  planlayıcı yerel minimumdan kendi başına çıkamaz. Beklenti çarpmamak
  ve kapanı fark etmek.
- Süreler 60 s'lik üst sınıra takılmış olabilir; 'BAŞARILI' burada
  'temassız ve düzgün ilerledi' demektir, '9 m'yi tamamladı' demek değil.
  2B tablosundaki (`docs/test-sonuclari.md`) hedefe varma ölçütüyle
  karıştırma.
