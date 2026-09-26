#!/usr/bin/env bash
# Compile les deux firmwares pour Teensy 4.1 sans PlatformIO ni IDE Arduino :
# cœur Teensy officiel + bibliothèques (versions figées) + GCC ARM (arm-none-eabi-gcc).
#
#   sudo apt install gcc-arm-none-eabi      (ou la chaîne fournie avec Teensyduino)
#   bash tools/build-teensy.sh              → firmware/bin/*.hex (à flasher avec Teensy Loader)
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEPS=$ROOT/firmware/.deps
BUILD=$ROOT/firmware/.build
BIN=$ROOT/firmware/bin

fetch() {  # dépôt commit dossier
  if [ ! -d "$DEPS/$3/.git" ]; then
    git clone --quiet "$1" "$DEPS/$3"
  fi
  git -C "$DEPS/$3" fetch --quiet --depth 1 origin "$2" 2>/dev/null || true
  git -C "$DEPS/$3" checkout --quiet "$2"
}
mkdir -p "$DEPS" "$BIN"
fetch https://github.com/PaulStoffregen/cores.git 7f107ee0a309f3813ed13f0d8f615497eca2ee49 cores
fetch https://github.com/PaulStoffregen/EEPROM.git 9790da76d62bc633563f763c3dc1526539ed0a6b EEPROM
fetch https://github.com/tonton81/FlexCAN_T4.git b928bcb2f74fdf0ccb76660f52ca1a2819ff4e2c FlexCAN_T4

CORE=$DEPS/cores/teensy4
CPU="-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb"
DEFS="-D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO=10819 -DARDUINO_TEENSY41 -DF_CPU=600000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH"
CFLAGS="$CPU -O2 -g -Wall -ffunction-sections -fdata-sections $DEFS -I$CORE"
CXXFLAGS="-std=gnu++17 -felide-constructors -fno-exceptions -fpermissive -fno-rtti -Wno-error=narrowing -fno-threadsafe-statics"

# Cœur Teensy (compilé une fois)
if [ ! -f "$BUILD/core/.done" ]; then
  echo "• cœur Teensy 4"
  mkdir -p "$BUILD/core"
  for f in "$CORE"/*.c; do arm-none-eabi-gcc $CFLAGS -c "$f" -o "$BUILD/core/$(basename "$f").o"; done
  for f in "$CORE"/*.cpp; do arm-none-eabi-g++ $CFLAGS $CXXFLAGS -w -c "$f" -o "$BUILD/core/$(basename "$f").o"; done
  for f in "$CORE"/*.S; do arm-none-eabi-gcc $CFLAGS -x assembler-with-cpp -c "$f" -o "$BUILD/core/$(basename "$f").o"; done
  touch "$BUILD/core/.done"
fi

build() {  # sketch nom bibliothèques…
  local sk=$1 name=$2; shift 2
  local out=$BUILD/$name inc="" lib
  echo "• $name"
  rm -rf "$out" && mkdir -p "$out"
  for lib in "$@"; do inc="$inc -isystem $DEPS/$lib"; done
  for lib in "$@"; do
    for f in "$DEPS/$lib"/*.cpp; do
      [ -f "$f" ] && arm-none-eabi-g++ $CFLAGS $CXXFLAGS -w $inc -c "$f" -o "$out/lib_$(basename "$f").o"
    done
  done
  while IFS= read -r f; do
    arm-none-eabi-g++ $CFLAGS $CXXFLAGS -Wextra -Wno-unused-parameter $inc -I"$sk/src" -c "$f" -o "$out/$(basename "$f").o"
  done < <(find "$sk/src" -name '*.cpp')
  arm-none-eabi-gcc -O2 -Wl,--gc-sections,--relax -Wl,--no-warn-rwx-segments $CPU -T"$CORE/imxrt1062_t41.ld" \
    -o "$out/$name.elf" "$out"/*.o "$BUILD"/core/*.o -lm -lstdc++
  arm-none-eabi-size "$out/$name.elf" | tail -1
  arm-none-eabi-objcopy -O ihex -R .eeprom "$out/$name.elf" "$BIN/$name-teensy41.hex"
}
build "$ROOT/firmware/orion_fw" orion_fw EEPROM
build "$ROOT/firmware/mit_bridge" mit_bridge EEPROM FlexCAN_T4
echo "✓ $(ls "$BIN"/*.hex | tr '\n' ' ')"
