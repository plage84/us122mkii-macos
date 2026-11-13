#!/bin/bash
# Diagnostic du chargement du plugin -> diagnostic.txt
cd "$(dirname "$0")/.."
P=/Library/Audio/Plug-Ins/HAL/US122MKII.driver
{
echo "=== diagnostic $(date) — macOS $(sw_vers -productVersion)"
echo; echo "### fichiers installés"; ls -laR "$P" 2>&1 | head -40
echo; echo "### signature"; codesign -dvvv "$P" 2>&1 | head -20
echo; codesign --verify --deep --strict -vvv "$P" 2>&1 | head -10
echo; echo "### symbole de la fabrique"; nm -gU "$P/Contents/MacOS/US122MKII" 2>&1 | grep -i create
echo; echo "### dépendances"; otool -L "$P/Contents/MacOS/US122MKII"
echo; echo "### Info.plist"; plutil -p "$P/Contents/Info.plist"
echo; echo "### attributs étendus (quarantaine ?)"; xattr -lr "$P" 2>&1 | head
echo; echo "### autres plugins HAL"; ls /Library/Audio/Plug-Ins/HAL/
echo; echo "### processus audio"; ps aux | grep -i -E "coreaudiod|Core-Audio-Driver|DriverService" | grep -v grep
echo; echo "### journal : mentions du plugin (10 min, tous processus)"
log show --last 10m --info --debug --style compact --predicate 'eventMessage CONTAINS[c] "US122" OR eventMessage CONTAINS[c] "plage84" OR subsystem == "fr.plage84.tascam.us122mkii"' 2>&1 | tail -80
echo; echo "### journal : coreaudiod / service de drivers, erreurs et plugins (10 min)"
log show --last 10m --style compact --predicate '(process == "coreaudiod" OR process CONTAINS "Core-Audio-Driver" OR process CONTAINS "DriverService") AND (messageType == error OR messageType == fault OR eventMessage CONTAINS[c] "plug" OR eventMessage CONTAINS[c] "bundle" OR eventMessage CONTAINS[c] "sandbox" OR eventMessage CONTAINS[c] "code sign" OR eventMessage CONTAINS[c] "driver")' 2>&1 | tail -120
echo; echo "### refus du bac à sable (10 min)"
log show --last 10m --style compact --predicate 'sender == "Sandbox" OR eventMessage CONTAINS "deny("' 2>&1 | grep -i -E "coreaudio|audio|driver" | tail -40
echo; echo "### rapports de plantage récents"
ls -lt /Library/Logs/DiagnosticReports ~/Library/Logs/DiagnosticReports 2>/dev/null | grep -i -E "coreaudio|audio|driver" | head
} > diagnostic.txt 2>&1
echo "Diagnostic écrit dans $(pwd)/diagnostic.txt"
