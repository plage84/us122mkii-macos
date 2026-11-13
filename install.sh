#!/bin/bash
# Installe le driver dans Core Audio (mot de passe administrateur demandé).
# Install the driver into Core Audio (asks for your admin password).
set -e
cd "$(dirname "$0")"
SRC=build/US122MKII.driver
DST=/Library/Audio/Plug-Ins/HAL/US122MKII.driver
[ -d "$SRC" ] || { echo "Lance d'abord : bash build.sh"; exit 1; }
xattr -dr com.apple.quarantine "$SRC" 2>/dev/null || true
sudo rm -rf "$DST"
sudo cp -R "$SRC" "$DST"
sudo chown -R root:wheel "$DST"
sudo chmod -R u=rwX,go=rX "$DST"
sudo codesign --verify --deep --strict "$DST" || { echo "Signature invalide, installation annulée"; exit 1; }
echo "==> redémarrage de Core Audio (le son du Mac coupe 2 secondes)"
sudo killall coreaudiod 2>/dev/null || true
sleep 3
echo "==> US-122 MKII installée. Branche la carte : elle apparaît dans Réglages Système > Son."
