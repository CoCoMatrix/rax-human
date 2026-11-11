"""The workspace recipe for the agent that controls the real robot."""
from .recipe import PROJECT, Recipe, SkillRule, Vocab

VOCAB = Vocab(memory='memory', notes='memory/PROGRESS.md', frames='runtime/frames/', inventory='status')

# Skill sources by axis. Every workspace takes the task skills, the perception skills of
# its observation mode ('depth' or 'rgb-only') and robot, the control skills of its robot,
# and the auto-research skills. Agents see one flat set with robot-neutral names.
SKILL_LAYERS = ('task', 'perception/common', 'perception/{perception}', 'perception/{perception}-{robot}',
                'control/common', 'control/{robot}', 'auto-research')

# No OpenRouter key, no gemini_generate tool: leave its skill out instead of advertising it.
AGENT = Recipe('real-robot/agent', PROJECT / 'auto_research_agent', SKILL_LAYERS, vocab=VOCAB,
               omit_skills=(SkillRule('gemini', lambda setup: not setup.gemini),
                            SkillRule('codex-vision', lambda setup: not setup.vision),
                            SkillRule('continual-learning', lambda setup: not setup.library)))


def recipe_for(options=frozenset()):
    return AGENT
