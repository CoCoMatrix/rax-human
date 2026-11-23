import os, sys
os.chdir("/home/ubuntu/code_junqi/GPT-as-Policy")
python="/home/ubuntu/code_junqi/GPT-as-Policy/.venv-piper/bin/python"
os.execv(python, [python, "-m", "real_robot.piper.reviewed_prefix", "verify", "--bundle", sys.argv[1]])
