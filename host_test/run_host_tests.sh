#!/usr/bin/env bash
# Tes host firmware Omni-3X (PRD A.5.6): tanpa ESP32, tanpa PlatformIO, tanpa fork file firmware.
# Pakai: host_test/run_host_tests.sh [filter-nama-tes]
# Opsional: SIM_DIR=/jalur/omni_sim untuk memeriksa kin_golden.json terhadap firmware.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$(cd "$HERE/.." && pwd)"
BUILD="${BUILD:-$HERE/build}"
SIM_DIR="${SIM_DIR:-$FW/../../omni_sim}"
mkdir -p "$BUILD"

command -v g++ >/dev/null || { echo "GAGAL: g++ tidak ada"; exit 1; }

python3 "$HERE/extract_funcs.py" --src "$FW/src/motors.cpp" --out "$BUILD/kin_extracted.cpp" \
  --include config.h --include motors.h --sig "void kinInverse(" --sig "void kinForward("

python3 "$HERE/extract_funcs.py" --src "$FW/src/ultrasonic.cpp" --out "$BUILD/us_extracted.cpp" \
  --include math.h --include config.h --include ultrasonic.h --sig "bool usMedian3("

python3 "$HERE/extract_funcs.py" --src "$FW/src/ir.cpp" --out "$BUILD/ir_extracted.cpp" \
  --include config.h --include ir.h --sig "bool irDebounce("

g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -g \
  -I "$HERE/stub" -I "$FW/include" -I "$FW/src" -I "$HERE" \
  "$HERE"/test_main.cpp "$HERE"/host_stubs.cpp "$BUILD/kin_extracted.cpp" "$BUILD/us_extracted.cpp" "$BUILD/ir_extracted.cpp" \
  "$HERE"/test_kin.cpp "$HERE"/test_navigation.cpp "$HERE"/test_imuassist.cpp "$HERE"/test_settings.cpp "$HERE"/test_ramp.cpp "$HERE"/test_ultrasonic.cpp \
  -o "$BUILD/host_tests"

echo "--- tes host firmware ---"
"$BUILD/host_tests" "${1:-}"
RC=$?

echo "--- kin_golden.json vs firmware ---"
if [ -f "$SIM_DIR/src/omni3x_sim_description/tools/export_firmware_config.py" ]; then
  python3 "$HERE/check_golden_fresh.py" --firmware-dir "$FW" --sim-pkg "$SIM_DIR/src/omni3x_sim_description"
else
  echo "TIDAK DIJALANKAN: $SIM_DIR tidak ada (set SIM_DIR=/jalur/omni_sim). kin_golden.json belum dibandingkan dengan firmware."
fi
exit $RC
