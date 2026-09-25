#!/bin/bash
# usage: run.sh LABEL SCENARIO PLANT(A|B) [MODEL(A|B)] [extra args...]
SP=${AUDIT_SCRATCH:-/tmp/claude-0/-home-user-KSL/a350dcd7-29f8-556d-b0c6-c68a6908276f/scratchpad}
SEED=$SP/seed
label=$1; scen=$2; plant=$3; model=${4:-$3}; shift 4
aero(){ case $1 in A) echo $SEED/stsn_aero_seed.csv;; B) echo $SEED/stsn_aero_kspB.csv;; C) echo $SEED/stsn_aero_kspC.csv;; esac; }
cd /home/user/KSL
# Simulator plant = PLANT; MM305 terminal model = MODEL (override env after runner sets it)
export KSP_LANDER_ENTRY_TRACE=1
python3 ShuttleSim/scripts/run_guidance.py --scenario $scen --atmosphere $SEED/kerbin_atmosphere_seed.csv \
  --aero $(aero $plant) --aero-book ${BOOK:-$SEED/force_book_$plant.csv} --attitude $SEED/stsn_attitude_seed.ini --skip-build --no-mirror \
  --label $label --quiet-progress "$@" 2>&1 | tail -1
