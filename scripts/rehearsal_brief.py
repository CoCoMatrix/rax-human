"""Prepare compact native function-rehearsal context; files only, robot agnostic."""
import ast
import hashlib
import json
from pathlib import Path


def install(workspace, source):
    workspace=Path(workspace);source=Path(source)
    refs=workspace/'references';refs.mkdir(exist_ok=True)
    entries=[]
    for p in sorted((workspace/'code').rglob('*.py')):
        if p.name.startswith('test_'):continue
        tree=ast.parse(p.read_text())
        functions=[]
        for node in tree.body:
            if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef)) and not node.name.startswith('_'):
                functions.append({'name':node.name,'line':node.lineno,'signature':ast.unparse(node.args)})
        entries.append({'path':str(p.relative_to(workspace)),
                        'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'functions':functions})
    (refs/'rehearsal-code-index.json').write_text(json.dumps(entries,indent=2))
    metadata=json.loads((source/'WORKSPACE.json').read_text())
    (refs/'rehearsal-source.json').write_text(json.dumps(metadata,indent=2))
    brief='''# Current stage: function rehearsal only

Use RAX real/franka autonomous-control: successful experience -> one useful
fn(ctx, inputs) -> run it -> inspect its report/images -> repair -> next function.
Do not compose main(ctx) or run scale tests in this session.

## Startup once
Read TASK.md, memory/INDEX.md, memory/PROGRESS.md and this brief. PROGRESS is
the canonical engineering checkpoint, including work done BEFORE a function call.
REHEARSAL_STATE.json is the separate last-dispatch/result ledger, not a replacement
for development progress. Obtain live status/current views.
Read autonomous-control and the current tool contract once. Retrieve only tips
matching the NEXT function. references/rehearsal-code-index.json lists signatures
and source identities; open only the next implementation or a failing branch.
Do not repeatedly cat all skills, tool history, full source or old model transcripts.

## Ordinary function calls
Use one outer api.runtime.Context for the function and pass the same ctx to its
helpers. Run ordinary Python once; the function returns a report. No persistent
command inbox, approval files, token-polling loop or wait for the main Agent INSIDE
a function. A synchronous ordinary check may use current observations, feedback,
geometry and task facts; Gemini only if simple checks cannot answer the fact.
Never import the offline true-assessor fixture into a hardware call.

Routine numerical arrivals are checked by tool receipts. Normal small segments
stay within a bounded function; do not request Agent approval at every jaw step
or waypoint. Return needs_review immediately for a genuinely missing fact. Agent
inspection happens AFTER return. Missing result evidence is not failed motion:
preserve executed/arrival/current pose and enter the next appropriate function,
never repeat a completed grasp/lift/placement to obtain a new picture.

## Resume after compaction or interruption
Read memory/PROGRESS.md first, then memory/REHEARSAL_STATE.json and live status.
PROGRESS names the current function, exact pending test/failure, files already read,
last change and next command. Update it after a meaningful edit/test as well as after
a physical result. The automatic ledger only records dispatch and return; it must
not reset engineering progress to a historical function result. If status is in_progress
or motion_dispatch_pending, inspect the native receipt/current state and stop if
uncertain; never rerun by inference. If action completed but needs_review, inspect
the referenced current result or obtain only the missing observation. Unchanged
guidance is already read; do not repeat startup. Log failure/repair before continuing.

## Continue the rehearsal, then close out
One successful function is a checkpoint, not the end of this task. Save a short
PROGRESS entry and select the next necessary function while the live budget permits.
Missing ordinary checks are work to implement and test. Stop for a concrete unresolved
physical fact, user stop, actual budget boundary or completion of the task scope.
Read experience-memory formatting and continual-learning merge instructions only
when distilling/merging, not before every physical function. Full documentation,
library packaging and commits belong to the separate software closeout, if enabled.
Do not invent a new fixed600s/32-call reserve inside the controller: consume the
current status.wall_budget contract. Do not read all source to prepare a summary.

## Reporting and reserve
Each call records code identity, JSON inputs, full result path, executed state,
function wall time, motion RPC time and auxiliary count. Print only the compact
summary to the main Agent; frame paths are opened selectively. Tool RPC time is
not pure physical motion and model time can overlap it. Keep raw timelines.
Use the existing task wall_budget for motion and a concise checkpoint/independent
stop. AGENT_OWNERSHIP.md assigns full software closeout to a separate RAX Agent
continuation after hardware shutdown; it consumes none of this task budget. Update task tips, RUNS and output/rehearsal/SUMMARY.json
truthfully. Freeze is preserved; this session uses only its writable imported copy.
'''
    (workspace/'REHEARSAL_BRIEF.md').write_text(brief)
    agents=workspace/'AGENTS.md'
    original=agents.read_text()
    (refs/'base-agents-before-rehearsal.md').write_text(original)
    agents.write_text('''# Active session scope: function rehearsal

REHEARSAL_BRIEF.md selects the current stage and startup/resume sequence. The
general workflow below is context, not an instruction to redo manual exploration,
compose main, or start unattended tests now. Startup skill reads are once per
session; after compaction use PROGRESS (engineering) plus the dispatch ledger and live state.
Robot safety, current tool checks and native stop rules below remain in force.

'''+original)
    (workspace/'START_PROMPT.md').write_text('Read REHEARSAL_BRIEF.md and its compact startup list once. '
       'Rehearse the next function through ordinary Python, inspect its returned report, and continue from actual state. '
       'Do not add an in-function main-Agent approval loop.\n')
    return {'stage':'function_rehearsal','modules':len(entries),'brief':'REHEARSAL_BRIEF.md'}
