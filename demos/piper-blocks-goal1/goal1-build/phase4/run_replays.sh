#!/bin/bash
# usage: run_replays.sh CODE_DIR OUTDIR   (replays every recorded start1 run that has a controller report)
B=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments; D=$B/goal1-phase4-dev
S=$B/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-web-20261119/runtime/commissioning
mkdir -p $D/$2
ls -d $S/web-main-*/rax-session/agent-workspace/output/composition/runs/*/REPORT.json | sed 's|/output/composition/runs/.*||' | xargs -P 6 -I{} sh -c 'r=$(echo {} | sed "s|.*web-main-||; s|/rax-session.*||"); PYTHONDONTWRITEBYTECODE=1 timeout 600 /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python '$D'/exec_replay.py '$1' {} '$D'/'$2'/$r.json 2>'$D'/'$2'/$r.err | tail -1'
