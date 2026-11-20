#!/usr/bin/env bash
# =============================================================================
# 00 - ORTAM KONTROLU
# =============================================================================
# "Hicbir sey calismiyor" dedigin anda ILK bunu calistir.
#
#     bash ogren/00_kontrol.sh
#
# Ne yapar: neyin bozuk oldugunu soyler ve duzeltebilecegini duzeltir.
# =============================================================================

echo "=============================================="
echo " mergen-01 ortam kontrolu"
echo "=============================================="
echo

SORUN=0

# --- 1) ROS kurulu mu? --------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
  echo "[OK]   ROS 2 Humble kurulu"
else
  echo "[HATA] ROS 2 Humble YOK"
  echo "       Cozum: sudo bash install/01_ros2_humble.sh"
  SORUN=1
fi

# --- 2) Terminal ortami dogru mu? ---------------------------------------
# Bunlar ~/.bashrc'den otomatik gelir. Yani yeni actigin HER terminalde
# hazirdir; elle "source ..." yazmana GEREK YOK.
if [ -n "${ROS_DISTRO:-}" ]; then
  echo "[OK]   Terminal ortami hazir (ROS_DISTRO=$ROS_DISTRO)"
  echo "       -> elle 'source' yazmana gerek yok, otomatik geliyor"
else
  echo "[HATA] Bu terminalde ROS ortami yok"
  echo "       Cozum: YENI bir terminal ac (Ctrl+Alt+T)"
  echo "       Hala olmazsa: source ~/.bashrc"
  SORUN=1
fi

echo "       ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-ayarsiz}  (tum terminallerde AYNI olmali)"
echo "       TURTLEBOT3_MODEL = ${TURTLEBOT3_MODEL:-ayarsiz}"

# --- 3) Calisma alani derlenmis mi? -------------------------------------
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$REPO/ros2_ws/install/setup.bash" ]; then
  echo "[OK]   Calisma alani derlenmis"
else
  echo "[HATA] Calisma alani derlenmemis"
  echo "       Cozum: bash install/03_env_and_build.sh"
  SORUN=1
fi

# --- 4) ASILI KALAN GAZEBO ----------------------------------------------
# En sinsi sorun bu. Gazebo'yu kapattigini sanirsin ama arkada gzserver
# kalir; o surec 11345 portunu tutar ve ACTIGIN HER YENI GAZEBO SESSIZCE
# OLUR. Hicbir hata penceresi gormezsin, sadece "acilmadi" dersin.
echo
if pgrep -x gzserver > /dev/null 2>&1 || pgrep -x gzclient > /dev/null 2>&1; then
  echo "[SORUN] Arkada asili kalan Gazebo var:"
  ps -eo pid,etime,comm | grep -E ' gz(server|client)$' | sed 's/^/        /'
  echo "        Bu surec Gazebo portunu tutuyor; yeni acacagin Gazebo"
  echo "        SESSIZCE olur. Temizleniyor..."
  pkill -x gzserver 2>/dev/null
  pkill -x gzclient 2>/dev/null
  sleep 2
  pkill -9 -x gzserver 2>/dev/null
  pkill -9 -x gzclient 2>/dev/null
  sleep 1
  if pgrep -x gzserver > /dev/null 2>&1; then
    echo "[HATA]  Temizlenemedi. Bilgisayari yeniden baslatmak kesin cozum."
    SORUN=1
  else
    echo "[OK]    Temizlendi."
  fi
else
  echo "[OK]   Asili Gazebo yok"
fi

# --- 5) ROS dugumleri birbirini goruyor mu? -----------------------------
echo
echo "Dugumler birbirini goruyor mu diye test ediliyor (5 sn)..."
ros2 topic pub -r 5 /ogren_test std_msgs/msg/String "{data: test}" >/dev/null 2>&1 &
PUB_PID=$!
sleep 2
if timeout 4 ros2 topic echo /ogren_test 2>/dev/null | head -4 | grep -q "data: test"; then
  echo "[OK]   Dugumler haberlesiyor"
else
  echo "[HATA] Dugumler birbirini GORMUYOR"
  echo "       Genelde sebep: terminallerde farkli ROS_DOMAIN_ID"
  echo "       Cozum: tum terminalleri kapat, yenilerini ac"
  SORUN=1
fi
kill $PUB_PID 2>/dev/null
wait $PUB_PID 2>/dev/null

# --- sonuc ---------------------------------------------------------------
echo
echo "=============================================="
if [ "$SORUN" -eq 0 ]; then
  echo " HER SEY HAZIR"
  echo
  echo " Sonraki adim:"
  echo "   bash ogren/01_kaplumbaga.sh"
else
  echo " YUKARIDAKI [HATA] satirlarini duzelt, sonra tekrar calistir."
fi
echo "=============================================="
