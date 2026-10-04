#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

command -v pio >/dev/null 2>&1 || { echo "ERROR: nie ma 'pio' w PATH"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: nie ma 'python3' w PATH"; exit 1; }

OUT_DIR="firmwares"
PIO_ENV="fw"

TXT_MODE="which_to_choose_mode.txt"
TXT_AUTOLOAD="which_to_choose_autoload.txt"
TXT_RGB="which_to_choose_filament_rgb.txt"
TXT_SLOTS="which_to_choose_slots.txt"
OUT_GUIDE="which_to_choose.txt"

[[ -f "${TXT_MODE}" ]]     || { echo "ERROR: brak ${TXT_MODE}"; exit 1; }
[[ -f "${TXT_AUTOLOAD}" ]] || { echo "ERROR: brak ${TXT_AUTOLOAD}"; exit 1; }
[[ -f "${TXT_RGB}" ]]      || { echo "ERROR: brak ${TXT_RGB}"; exit 1; }
[[ -f "${TXT_SLOTS}" ]]    || { echo "ERROR: brak ${TXT_SLOTS}"; exit 1; }

RETRACTS=(
  "0.095"
  "0.10"
  "0.20" "0.25" "0.30" "0.35" "0.40"
  "0.45" "0.50" "0.55" "0.60" "0.65"
  "0.70" "0.75" "0.80" "0.85" "0.90"
)

# dir|P1S|SOFT_LOAD
MODES=(
  "soft_load(A1)|0|1"
  "standard(A1)|0|0"
  "high_force_load(P1S)|1|0"
)

build_and_copy() {
  local out_path="$1"
  local ams_num="$2"
  local retract_len="$3"
  local dm="$4"
  local rgb="$5"
  local p1s="$6"
  local soft_load="$7"

  echo "=== BUILD: P1S=${p1s} SOFT_LOAD=${soft_load} DM=${dm} RGB=${rgb} AMS_NUM=${ams_num} RETRACT=${retract_len} -> ${out_path}"

  BAMBU_BUS_AMS_NUM="${ams_num}" \
  AMS_RETRACT_LEN="${retract_len}" \
  BMCU_DM_TWO_MICROSWITCH="${dm}" \
  BMCU_ONLINE_LED_FILAMENT_RGB="${rgb}" \
  DBMCU_P1S="${p1s}" \
  BMCU_SOFT_LOAD="${soft_load}" \
  pio run -e "${PIO_ENV}"

  local src=".pio/build/${PIO_ENV}/firmware.bin"
  [[ -f "${src}" ]] || { echo "ERROR: brak ${src}"; exit 1; }

  mkdir -p "$(dirname "${out_path}")"
  cp -f "${src}" "${out_path}"
}

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
cp -f "${TXT_MODE}" "${OUT_DIR}/${OUT_GUIDE}"

for mode_spec in "${MODES[@]}"; do
  IFS='|' read -r mode_dir p1s soft_load <<< "${mode_spec}"

  mode_base="${OUT_DIR}/${mode_dir}"
  mkdir -p "${mode_base}"
  cp -f "${TXT_AUTOLOAD}" "${mode_base}/${OUT_GUIDE}"

  for dm in 1 0; do
    if [[ "${dm}" == "1" ]]; then
      dm_dir="AUTOLOAD"
    else
      dm_dir="NO_AUTOLOAD"
    fi

    dm_base="${mode_base}/${dm_dir}"
    mkdir -p "${dm_base}"
    cp -f "${TXT_RGB}" "${dm_base}/${OUT_GUIDE}"

    for rgb in 1 0; do
      if [[ "${rgb}" == "1" ]]; then
        rgb_dir="FILAMENT_RGB_ON"
      else
        rgb_dir="FILAMENT_RGB_OFF"
      fi

      base="${dm_base}/${rgb_dir}"
      mkdir -p "${base}"/{AMS_AUTO,AMS_A,AMS_B,AMS_C,AMS_D}
      cp -f "${TXT_SLOTS}" "${base}/${OUT_GUIDE}"

      for slot in AUTO A B C D; do
        case "${slot}" in
          AUTO) ams_num=4; slot_dir="AMS_AUTO"; file_prefix="ams_auto" ;;
          A)    ams_num=0; slot_dir="AMS_A";    file_prefix="ams_a" ;;
          B)    ams_num=1; slot_dir="AMS_B";    file_prefix="ams_b" ;;
          C)    ams_num=2; slot_dir="AMS_C";    file_prefix="ams_c" ;;
          D)    ams_num=3; slot_dir="AMS_D";    file_prefix="ams_d" ;;
        esac

        for r in "${RETRACTS[@]}"; do
          build_and_copy \
            "${base}/${slot_dir}/${file_prefix}_${r}f.bin" \
            "${ams_num}" \
            "${r}f" \
            "${dm}" \
            "${rgb}" \
            "${p1s}" \
            "${soft_load}"
        done
      done
    done
  done
done

python3 - "${OUT_DIR}" > "${OUT_DIR}/manifest.txt" <<'PYMAN'
import sys, os, zlib, hashlib

root = sys.argv[1]
entries = []

for dirpath, _, filenames in os.walk(root):
    for fn in filenames:
        p = os.path.join(dirpath, fn)
        rel = os.path.relpath(p, root).replace(os.sep, "/")

        crc = 0
        size = 0
        h = hashlib.sha256()

        with open(p, "rb") as f:
            while True:
                b = f.read(1024 * 1024)
                if not b:
                    break
                size += len(b)
                crc = zlib.crc32(b, crc)
                h.update(b)

        entries.append((rel, h.hexdigest(), f"{crc & 0xffffffff:08X}", size))

entries.sort(key=lambda x: x[0])

out = sys.stdout
out.write("# format: SHA256_HEX CRC32_HEX SIZE_BYTES REL_PATH\n")
for rel, sha256_hex, crc32_hex, size in entries:
    out.write(f"{sha256_hex} {crc32_hex} {size} {rel}\n")
PYMAN

echo
echo "DONE. Wyniki w: ${OUT_DIR}/"
echo "Manifest: ${OUT_DIR}/manifest.txt"
