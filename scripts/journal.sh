#!/bin/bash
# Enregistre le journal du driver (10 dernières minutes par défaut) dans journal.txt
# Usage : bash scripts/journal.sh [durée, ex. 30m]
cd "$(dirname "$0")/.."
log show --last ${1:-10m} --style compact --info --debug \
  --predicate 'subsystem == "fr.plage84.tascam.us122mkii"' 2>&1 | tail -400 | tee journal.txt
