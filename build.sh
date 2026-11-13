#!/bin/bash
# Compile l'outil de test et le plugin Core Audio.
# LIBUSB=dynamic bash build.sh  -> libusb embarquée en .dylib plutôt que liée statiquement
set -e
cd "$(dirname "$0")"
BREW="$(brew --prefix 2>/dev/null || echo /opt/homebrew)"
INC="$BREW/include/libusb-1.0"
[ -f "$INC/libusb.h" ] || { echo "libusb introuvable : lance 'brew install libusb'"; exit 1; }
command -v clang >/dev/null || { echo "clang introuvable : lance 'xcode-select --install'"; exit 1; }

CFLAGS="-O2 -arch arm64 -mmacosx-version-min=12.0 -std=c11 -Wall -Wextra -Wno-unused-parameter -I$INC -Isrc"
FW="-framework IOKit -framework CoreFoundation -framework Security"
rm -rf build && mkdir -p build

STATIC="$BREW/lib/libusb-1.0.a"
if [ -f "$STATIC" ] && [ "${LIBUSB:-static}" != dynamic ]; then
  USBLIB="$STATIC"; MODE=statique
else
  USBLIB="-L$BREW/lib -lusb-1.0"; MODE=dynamique
fi
echo "==> libusb : liaison $MODE"

echo "==> outil de test"
clang $CFLAGS src/tascam_usb.c src/tascam_test.c $USBLIB $FW -o build/tascam_test

echo "==> plugin Core Audio"
B=build/US122MKII.driver
mkdir -p "$B/Contents/MacOS" "$B/Contents/Resources"
cp Info.plist "$B/Contents/Info.plist"
clang $CFLAGS -bundle src/tascam_usb.c src/TascamPlugIn.c $USBLIB $FW \
      -framework CoreAudio -o "$B/Contents/MacOS/US122MKII"

if [ "$MODE" = dynamique ]; then
  # on embarque libusb dans le plugin (coreaudiod ne lit pas /opt/homebrew)
  DYL="$(otool -L "$B/Contents/MacOS/US122MKII" | awk '/libusb/{print $1; exit}')"
  cp "$BREW/lib/libusb-1.0.0.dylib" "$B/Contents/MacOS/"
  install_name_tool -id @loader_path/libusb-1.0.0.dylib "$B/Contents/MacOS/libusb-1.0.0.dylib"
  install_name_tool -change "$DYL" @loader_path/libusb-1.0.0.dylib "$B/Contents/MacOS/US122MKII"
  codesign --force --sign - "$B/Contents/MacOS/libusb-1.0.0.dylib"
fi

# permissions lisibles par coreaudiod (utilisateur _coreaudiod), pas de symboles de débogage
rm -rf build/*.dSYM "$B"/Contents/MacOS/*.dSYM
chmod -R u=rwX,go=rX "$B" build/tascam_test
codesign --force --sign - build/tascam_test
codesign --force --sign - "$B"
plutil -lint "$B/Contents/Info.plist" >/dev/null
codesign --verify --deep --strict "$B" && echo "==> signature vérifiée"
echo "==> OK : build/tascam_test et $B"
