"""goal1 phase 4: execution checks follow the goal description. Run in the controller code dir."""
from pathlib import Path


def patch(path, pairs):
    p = Path(path); s = p.read_text()
    for old, new, *count in pairs:
        n = count[0] if count else 1
        assert s.count(old) == n, (path, old[:90], s.count(old))
        s = s.replace(old, new)
    p.write_text(s)


patch('task_spec.py', [
    ("TIGHT=GOAL['spacing']=='tight'\n",
     "TIGHT=GOAL['spacing']=='tight'\n"
     "# The blended empty move from the released green block to the yellow pickup was rehearsed\n"
     "# for the original structure only. Other goals run those two moves as separate checked steps.\n"
     "EMPTY_HANDOFF=('green','yellow') if IS_DEFAULT_GOAL else None\n"
     "COLOR_NAMES_ZH={'red':'红色','orange':'橙色','green':'绿色','yellow':'黄色'}\n"),
])
patch('assembly_controller.py', [
    ("from observations import identity\n", "from observations import identity\nimport task_spec as TS\n"),
    ("                            if color!='yellow':raise NoPlan('Only final yellow may finish without retreat')",
     "                            if color!=TS.FINAL_COLOR:raise NoPlan('Only the block placed last may finish without retreat')"),
    ("        if terminal and (execution.held is not None or (execution.release or {}).get('color')!='yellow'):\n"
     "            raise NoPlan('Final release-in-place lacks successful yellow release')",
     "        if terminal and (execution.held is not None or (execution.release or {}).get('color')!=TS.FINAL_COLOR):\n"
     "            raise NoPlan('Final release-in-place lacks successful release of the last block')"),
    ("            why='黄色已放置并释放；退开候选无解，保持最终张爪位置，未退开。' if terminal else",
     "            why=TS.COLOR_NAMES_ZH[TS.FINAL_COLOR]+'已放置并释放；退开候选无解，保持最终张爪位置，未退开。' if terminal else"),
])
patch('assembly_batch.py', [
    ("def empty_handoff(action):\n    return (action.get('kind')=='arm'",
     "def empty_handoff(action):\n    if not TS.EMPTY_HANDOFF:return False\n    return (action.get('kind')=='arm'"),
    ("def green_yellow_transition(actions):\n    return any(",
     "def green_yellow_transition(actions):\n    return bool(TS.EMPTY_HANDOFF) and any("),
    ("    handoff=any(a.get('stage')=='green-retreat' for a in actions)\n",
     "    handoff=bool(TS.EMPTY_HANDOFF) and any(a.get('stage')=='green-retreat' for a in actions)\n"),
    ("    if actions[0].get('stage')!='green-retreat':return\n",
     "    if not TS.EMPTY_HANDOFF or actions[0].get('stage')!='green-retreat':return\n"),
])
s = Path('assembly_batch.py').read_text()
first_import = next(line for line in s.splitlines() if line.startswith(('import ', 'from ')))
Path('assembly_batch.py').write_text(s.replace(first_import + '\n', first_import + '\nimport task_spec as TS\n', 1))
patch('ordinary_checks.py', [
    ("from task_spec import SUPPORTS\n", "from task_spec import SUPPORTS\nimport task_spec as TS\n"),
    ("        neighbor=SUPPORTS.get(color, 'red' if color=='orange' else 'orange')\n"
     "        block=scene.get('blocks',{}).get(neighbor)\n"
     "        if block is not None:\n",
     "        # Its support, or for a table block a goal neighbour on the table (the other base block before).\n"
     "        beside=[c for c in TS.goal_neighbors(color) if TS.LEVEL[c]==TS.LEVEL[color]]\n"
     "        for neighbor in ([SUPPORTS[color]] if color in SUPPORTS else beside):\n"
     "          block=scene.get('blocks',{}).get(neighbor)\n"
     "          if block is not None and pair.get('ok') is not True:\n"),
])
print('execution patched')
