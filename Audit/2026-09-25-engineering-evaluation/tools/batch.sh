#!/bin/bash
# batch.sh PLANT ENGAGE outprefix scen...
SP=${AUDIT_SCRATCH:-/tmp/claude-0/-home-user-KSL/a350dcd7-29f8-556d-b0c6-c68a6908276f/scratchpad}
plant=$1; eng=$2; pre=$3; shift 3
for f in "$@"; do b=$(basename $f .ini); echo "$SP/run.sh ${pre}-${b} $f $plant $plant --engage $eng > $SP/res/${pre}-${b}.json"; done | xargs -P 4 -I{} bash -c "{}"
