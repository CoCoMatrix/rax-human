"""RAX real/rainbow startup and recovery pointers, using native CLI compaction."""
import json
import re
from pathlib import Path

START = """# One entry for startup and native compaction
Run `python code/rax_support/context_entry.py` first. It emits initial/changed
required references once and preserves their content hashes in memory/CONTEXT_READS.json.
Native SessionStart(compact/resume) supplies the current structured continuation automatically.
Use context_entry only at initial startup or if that automatic context is unavailable.
AGENTS.md is already in context. Do not print it, repeat startup or reload all skills.
Call live status and reconcile the compact round/receipt digest. Pending or unknown
execution blocks a new action until resolved; observation loss never undoes execution.

Use memory/INDEX.md and skill-library/INDEX.md to retrieve only relevant experience.
Read the skill for the next bounded step; do not load all skills or controller sources.
A short task-stage outline is enough. Implement one bounded primitive, execute it,
inspect its raw head/wrist views and receipt, then improve the next primitive at known rest.
Do not require all future IK or a finished controller before the first safely recoverable step.
Immediate clearance, support/retention and safe recovery still must be established.

When a new finding changes the next step, update memory/PROGRESS.md before the next capture, with: current step, last actual receipt, tried/rejected hypothesis and reason, next concrete
step, and evidence paths. This includes planning-only findings before the first movement.
Do not redo a rejected hypothesis without new evidence or changed inputs. Geometry and IK
CLI diagnostics persist their results; context_entry includes these after compaction.
No scene facts or permissions come from read receipts or historical memory.

Finish analysis and code edits BEFORE current capture. Inspect returned review_frames;
use an action's post-observation as the next checkpoint while fresh. If expired, observe
anew, never refresh timestamps. SAM3 stays initial-only when INITIAL_SCENE is complete.
Keep reusable findings in TIPS and indexed task skills; outcomes in RUNS. Use official
continual-learning when publishing draft/verified reusable code; it is not a prerequisite
for each exploratory action. Freeze remains separate complete-task verification.
A normal successful action already journals its receipt and next contract; do not duplicate it
in a separate plan/progress write. After compaction the contract includes camera-first ROI
shape, failed hypotheses and lift comparison; use those without rereading their implementations.
Native auto compaction remains 100000 total tokens; RESUME.md uses this same entry.
"""

RESUME = """# Resume the unfinished step, not the startup sequence
Use the automatically supplied RAX continuation. If absent, run
`python code/rax_support/context_entry.py` as fallback. Read progress, last actual
receipt and failed geometry/IK hypotheses. Unchanged required references are already
recorded: do not reread them merely because context compacted. Retrieve a specific
section only if the next decision needs detail not retained in context.
Check live status, resolve pending/unknown execution, and continue the next unfinished
bounded step. Never replay an executed action, reset the owner or restart segmentation.
Useful analysis and code edits precede the fresh head/wrist action checkpoint.
Actual operator replies are injected from the protected feedback inbox at tool and final-response boundaries.
Timeout is unanswered, not failure. A later reply does not restart a stopped run or authorize motion.
Never fabricate facts; preserve assistance and require current observations before motion.
"""

def launch_prompt(workspace, *, resume=False):
    workspace=Path(workspace)
    marker=workspace/'.codex/rax-workflow.json'
    if not marker.exists():
        return None
    mode=json.loads(marker.read_text()).get('mode')
    if mode not in ('explore','frozen_test'):
        raise ValueError('Invalid workbench workflow mode')
    route='RESUME.md' if resume else 'START_HERE.md'
    return (f'AGENTS.md is already supplied. At first activation, '
            'run python code/rax_support/context_entry.py. It selects only unread/changed '
            f'references and restores the unfinished step; {route} describes this entry. '
            'After compaction use the automatically injected RAX continuation; context_entry is fallback only. Do not rerun startup. '
            'Use live status and measured receipts; preserve pending/unknown actions. '
            + ('This is editable real/rainbow exploration: execute, observe and improve one bounded '
               'primitive at a time. A complete controller is not a prerequisite for the first '
               'safely recoverable step. Keep physical guards and meaningful visual checkpoints. '
               if mode=='explore' else
               'This is a frozen end-to-end test: no code or parameter revisions or hand-finishing. ')
            +'Use the task memory index and native compaction, not a new custom summarizer.\n')

def install(workspace):
    workspace=Path(workspace)
    # Preserve original upstream text in the generated workspace; defer authoring sections.
    skill=workspace/'.agents/skills/continual-learning/SKILL.md'
    if skill.is_file():
        original=skill.read_text();marker='## 2. What to contribute'
        if marker in original:
            skill.with_name('AUTHORING.md').write_text(original)
            skill.write_text(original[:original.index(marker)]+
                '## When contributing or updating code\n\n'
                'Read [AUTHORING.md](AUTHORING.md) for modular primitives, contracts, evidence, '
                'versioning, validation and skill_library_merge. The original instructions are '
                'preserved there. Retrieve them when contribution begins, not at every action checkpoint.\n')
    agents=workspace/'AGENTS.md'
    if agents.exists():
        text=agents.read_text()
        replacement=('## Start here\n\nFollow START_HERE.md once; it is this workbench\'s canonical '
                     'startup order. AGENTS.md is already supplied in native context.\n'
                     'After compaction use the injected current continuation and task memory index; RESUME.md is fallback.\n'
                     'Read only current-step references, preserving all safety rules below and above.\n\n')
        text=re.sub(r'## Start here\n.*?(?=## Workflow)',replacement,text,count=1,flags=re.S)
        marker='## Workbench context recovery'
        section=('\n\n'+marker+'\nNative compaction injects actual receipts and current decision contract automatically. Explore one bounded primitive, '
                 'inspect it, and revise at known rest; do not require all future code/IK up front. '
                 'The task policy separates editable exploration from frozen verification.\n')
        if marker in text:
            text=re.sub(r'\n*## Workbench context recovery\n.*?(?=\n## |\Z)',section,text,count=1,flags=re.S)
        else:text+=section
        agents.write_text(text)
    (workspace/'START_HERE.md').write_text(START)
    (workspace/'RESUME.md').write_text(RESUME)
