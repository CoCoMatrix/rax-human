#!/bin/bash
# usage: run_dump.sh CODE_RUNTIME OUTDIR [extra args]  (reads run dirs from stdin)
B=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments
RT=$1; OUT=$2; shift 2
export RAX_ARTIFACT_ROOT=$B/goal1-phase3-dev/art PYTHONDONTWRITEBYTECODE=1
mkdir -p $OUT
xargs -P 4 -I{} sh -c "nice -n 5 timeout 400 /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python $B/goal1-phase3-dev/plan_dump.py $RT $B/{} $RT/workspaces/magnetic-blocks-l-stack/r9-ssik-speed/code/magnetic_blocks $OUT/\$(basename {}).json $* 2>$OUT/\$(basename {}).err | tail -1"
