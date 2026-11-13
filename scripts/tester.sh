#!/bin/bash
# Série de tests du moteur USB. Les résultats vont dans resultats_tests.txt
cd "$(dirname "$0")/.."
OUT=resultats_tests.txt
T=build/tascam_test
{
  echo "=== $(date) — $(sw_vers -productVersion) — $(uname -m)"
  echo; echo "### info";              $T info
  echo; echo "### bip 48 kHz (3 s)";  $T bip 48000 3
  echo; echo "### bip 44,1 kHz (2 s)";$T bip 44100 2
  echo; echo "### bip 96 kHz (2 s)";  $T bip 96000 2
  echo; echo "### horloge 48 kHz (12 s)"; $T clock 48000 12
  echo; echo "### enregistrement 48 kHz (8 s) -> test_c.wav"; $T rec 48000 8 test_c.wav
  echo; echo "### retour direct entrée->sortie (10 s)"; $T thru 48000 10
} 2>&1 | tee "$OUT"
echo; echo "Résultats enregistrés dans $(pwd)/$OUT"
