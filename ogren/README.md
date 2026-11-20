# Öğrenme Merdiveni

Bu klasör, `ros2_ws/` altındaki asıl koda **hazırlık**. Oradan başlanmaz —
burası başlangıç noktası.

Her adım **tek komut**. Sourcing, temizlik, hata kontrolü hepsi scriptin içinde.
Birden fazla terminal yönetmen gereken tek yer 3. adım, orası da açıkça yazıyor.

---

## Sıra

```bash
bash ogren/00_kontrol.sh        # bir şey çalışmazsa ÖNCE bunu çalıştır
bash ogren/01_kaplumbaga.sh     # düğüm, topic, mesaj — 5 dakika
bash ogren/02_lidar_gor.sh      # lidar verisi neye benziyor
python3 ogren/03_ilk_dugum.py   # kendi düğümün (02 açıkken, başka terminalde)
```

Sonra asıl kod okunabilir hale gelir:

```bash
bash scripts/run_scenario.sh w8_pedestrian viz
```

---

## Neden bu sırayla

| Adım | Ne öğretiyor | Asıl koddaki karşılığı |
|---|---|---|
| 01 | düğüm, topic, yayıncı/abone, `Twist` mesajı | `/cmd_vel` yayınlayan her şey |
| 02 | `LaserScan` yapısı, `inf`/`nan` anlamı | `perception.py` → `clean_scan()` |
| 03 | abone ol → karar ver → yayınla döngüsü | `avoider_node.py` → `_tick()` |

03'ü yazdığında otonom aracın **tüm iskeletini** yazmış olursun. Geri
kalan her şey o iskeletin içine daha iyi karar koymaktan ibaret.

---

## Sık karşılaşılan üç tuzak

**1. "Gazebo açılmıyor" / "hiçbir şey olmuyor"**

Arkada asılı kalan bir `gzserver` vardır. O süreç Gazebo'nun portunu tutar
ve açtığın her yeni Gazebo **sessizce** ölür — hata penceresi görmezsin.

```bash
pgrep -x gzserver        # varsa bir PID yazar
bash ogren/00_kontrol.sh # otomatik temizler
```

**2. "Kaplumbağa geldi ama hareket etmiyor"**

Resmî eğitimlerdeki `turtle_teleop_key`, **sadece kendi terminal penceresi
odaktayken** tuşları duyar. Başka pencereye tıkladıysan hiçbir şey olmaz.
`01_kaplumbaga.sh` bu yüzden klavye kullanmıyor — mesaj göndererek sürüyor,
ki zaten öğrenmen gereken şey o.

**3. "Komut bulunamadı" / düğümler birbirini görmüyor**

Yeni açtığın her terminal ROS ortamını `~/.bashrc`'den **otomatik** alır.
Elle `source /opt/ros/humble/setup.bash` yazmana gerek yok. Görmüyorsa
terminali kapatıp yenisini aç.

---

## Dosyayı değiştirerek öğren

`03_ilk_dugum.py` içinde **`DENE:`** diye işaretli üç yer var. En öğreticisi
sonuncusu: tek bir `#` işaretini kaldırdığında robot "duran" olmaktan çıkıp
"engelden kaçan" oluyor. Aradaki fark bir satır.

Oradan sonra `ros2_ws/src/mergen_avoidance/mergen_avoidance/planner.py`
dosyasını açıp bak: o bir satırın yerini 6 durumlu bir makine ve 91 aday
yönlü bir maliyet fonksiyonu almış. Ama yaptığı iş hâlâ aynı —
`/scan` oku, karar ver, `/cmd_vel` yaz.
