#!/bin/bash
# usage: run_sims.sh OUTDIR "scene ..." goal...
B=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments; D=$B/goal1-phase4-dev
N=$B/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119/runtime
S=$B/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-web-20261119/runtime/commissioning
OUT=$1; SCENES=$2; shift 2; mkdir -p $D/$OUT
export RAX_ARTIFACT_ROOT=$D/art PYTHONDONTWRITEBYTECODE=1
for s in $SCENES; do for g in $@; do echo $g $(ls -d $S/web-main-*$s*); done; done | xargs -P 4 -L 1 sh -c 'timeout 500 /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python '$D'/exec_sim.py '$N' '$B'/goal1-phase3-dev/codes/$0 $1 '$D'/'$OUT'/$0__$(basename $1 | cut -c19-24).json 2>'$D'/'$OUT'/$0__$(basename $1 | cut -c19-24).err | grep -v "^{" | tail -1'
