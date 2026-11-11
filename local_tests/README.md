# Regression tests

Run from the repository root in the robot Python environment:

```bash
$RAX_PYTHON -m pytest -q
```

The suite uses temporary artifact roots, the `mock` robot backend, simulated bridges and
fake model providers. It never moves hardware, opens cameras, calls a model or needs
credentials. Tests marked `hardware` are excluded by default; the archive tests skip
without `ffmpeg`.
