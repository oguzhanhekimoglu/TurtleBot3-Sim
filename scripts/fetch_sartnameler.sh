#!/usr/bin/env bash
# =============================================================================
# TEKNOFEST Robotaksi sartnamelerini indirir
# =============================================================================
#     bash scripts/fetch_sartnameler.sh
#
# PDF'ler depoya DAHIL DEGIL (19 MB). Metne cevrilmis .txt halleri repoda
# duruyor -- aranabilir ve kucuk. Orijinal PDF'lere ihtiyacin olursa bu
# script indirir.
#
# Not: TEKNOFEST bu baglantilari her yil yeniler. 404 alirsan
# https://www.teknofest.org/tr/yarismalar/robotaksi-binek-otonom-arac-yarismasi/
# adresinden guncel linkleri al.
# =============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEDEF="$REPO/docs/sartnameler"
mkdir -p "$HEDEF"
cd "$HEDEF"

BASE="https://cdn.teknofest.org/media/upload/userFormUpload"

indir() {
  local url="$1" ad="$2"
  if [ -f "$ad" ]; then
    echo "  zaten var : $ad"
    return
  fi
  if curl -fsSL --max-time 180 -o "$ad.tmp" "$url"; then
    mv "$ad.tmp" "$ad"
    echo "  indirildi : $ad  ($(du -h "$ad" | cut -f1))"
  else
    rm -f "$ad.tmp"
    echo "  BASARISIZ : $ad  (link eskimis olabilir)"
  fi
}

echo "Sartnameler indiriliyor -> docs/sartnameler/"
echo

indir "$BASE/2026_Robotaksi_Binek_Otonom_Arac_Yar%C4%B1smas%C4%B1_Sartnamesi_SvtF2.pdf" \
      "2026_robotaksi_sartname.pdf"
indir "$BASE/2026_Robotaksi_Binek_Otonom_Ara%C3%A7_Yar%C4%B1%C5%9Fmas%C4%B1_Haz%C4%B1r_Ara%C3%A7_Mimari_Tan%C4%B1mlama_Dok.pdf" \
      "2026_hazir_arac_mimari.pdf"
indir "$BASE/2026_Robotaksi_Binek_Otonom_Ara%C3%A7_Yar%C4%B1%C5%9Fmas%C4%B1_Haz%C4%B1r_Ara%C3%A7_Kullan%C4%B1c%C4%B1_Dok%C3%BCman%C4%B1_b.pdf" \
      "2026_hazir_arac_kullanici.pdf"
indir "$BASE/2026_Robotaksi_Binek_Otonom_Ara%C3%A7_Yar%C4%B1%C5%9Fmas%C4%B1_Haz%C4%B1r_Ara%C3%A7_Genel_Bilgilendirme_.pdf" \
      "2026_hazir_arac_genel.pdf"
indir "$BASE/2025_Robotaksi_Binek_Otonom_Arac_Yarismasi_Sartnamesi_sIYmR.pdf" \
      "2025_robotaksi.pdf"
indir "$BASE/Robotaksi_Binek_Otonom_Arac_Yarismasi_v1.1_442g9.pdf" \
      "2024_robotaksi_v1.1.pdf"
indir "$BASE/Robotaksi_Binek_Otonom_2023_v1_XTjlV.pdf" \
      "2023_robotaksi_v1.pdf"
indir "$BASE/ROBOTAKSI_BINEK_OTONOM_ARAC_YARISMA_SARTNAMESI_v7_9GcNL.pdf" \
      "2022_robotaksi_v7.pdf"

echo
echo "Rapor sablonlari (.docx) indiriliyor..."
indir "$BASE/Robotaksi_Sim%C3%BClasyon_%C5%9Eablon_rjtpc_bi9so.docx" \
      "2026_simulasyon_sablon.docx"
indir "$BASE/Robotaksi_Hazir_KTR_2026_bdpoi.docx" \
      "2026_hazir_KTR.docx"
indir "$BASE/Robotaksi-TYF-Hazir_Arac_QQApL.docx" \
      "2026_hazir_TYF.docx"
indir "$BASE/Robotaksi_Ozgun_KTR_2026_A0aSo.docx" \
      "2026_ozgun_KTR.docx"
indir "$BASE/Robotaksi-TYF-Ozgun_Arac_woPlC.docx" \
      "2026_ozgun_TYF.docx"

echo
echo "Metne cevirmek icin (pdftotext gerekir):"
echo "  for f in docs/sartnameler/*.pdf; do pdftotext -layout \"\$f\" \"\${f%.pdf}.txt\"; done"
echo
echo "Not: 2025 sartnamesi taranmis goruntulerden olusuyor, metin cikarimi sinirli."
