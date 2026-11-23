#!/bin/bash
# usage: run_goals.sh OUTDIR "scene-suffix ..." [goal names...]
B=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments; D=$B/goal1-phase3-dev
N=$B/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119/runtime
C=$N/workspaces/magnetic-blocks-l-stack/r9-ssik-speed/code/magnetic_blocks
OUT=$D/$1; SCENES=$2; shift 2
export RAX_ARTIFACT_ROOT=$D/art PYTHONDONTWRITEBYTECODE=1
mkdir -p $OUT $D/codes
GOALS=${@:-$(python3 -c "import json;print(' '.join(json.load(open('$D/goals.json'))))")}
jobs=""
for g in $GOALS; do
  rm -rf $D/codes/$g; mkdir -p $D/codes/$g; cp $C/*.py $D/codes/$g/; cp -r $C/native_fk $D/codes/$g/; rm -f $D/codes/$g/test_*.py
  python3 -c "import json;json.dump(json.load(open('$D/goals.json'))['$g'],open('$D/codes/$g/GOAL.json','w'))"
  for s in $SCENES; do run=$(grep -- "$s" $D/scenes-with-plan.txt | head -1); jobs="$jobs$g $run\n"; done
done
printf "$jobs" | xargs -P 4 -L 1 sh -c 'nice -n 5 timeout 400 /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python '$D'/plan_dump.py '$N' '$B'/$1 '$D'/codes/$0 '$OUT'/$0__$(basename $1).json 2>'$OUT'/$0__$(basename $1).err | tail -1'
