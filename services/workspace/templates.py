"""Jinja rendering of trusted workspace sources.

Sources are markdown with Jinja statements on their own lines, e.g.

    {% if camera_geometry %}
    ...text for depth-capable profiles...
    {% else %}
    ...RGB-only text...
    {% endif %}

trim_blocks/lstrip_blocks drop the statement lines entirely, so rendered text keeps
the source's line layout. Unknown names fail the build (StrictUndefined).

Reusable passages are registered blocks (agent_blocks/blocks.toml) inserted with
{{ block("<id>") }} on a line of its own; the setup's facts, the recipe's `vocab`
and any keyword parameters are available inside a block. Every block a file uses
is recorded, so a build can report which blocks produced each file.
"""
from functools import lru_cache
from pathlib import Path
try:
    import tomllib
except ImportError:  # Python 3.10 (the robot environment)
    import tomli as tomllib

from jinja2 import FileSystemLoader, StrictUndefined, TemplateSyntaxError, UndefinedError
from jinja2.sandbox import SandboxedEnvironment

BLOCKS = Path(__file__).resolve().parents[2] / 'agent_blocks'
LEFTOVERS = ('{%', '{{', '{#', '<!-- capability:', '<!-- end-capability')
TIERS = ('core', 'mutable')


@lru_cache(maxsize=1)
def environment():
    return SandboxedEnvironment(loader=FileSystemLoader(str(BLOCKS)), undefined=StrictUndefined,
                                trim_blocks=True, lstrip_blocks=True, keep_trailing_newline=True,
                                autoescape=False)


@lru_cache(maxsize=1)
def registry():
    """Block id -> entry; ids are unique and every path exists."""
    entries = tomllib.loads((BLOCKS/'blocks.toml').read_text()).get('block', [])
    result = {}
    for entry in entries:
        if set(entry) != {'id', 'path', 'tier'} or entry['tier'] not in TIERS:
            raise ValueError(f'Invalid block registry entry: {entry}')
        if entry['id'] in result:
            raise ValueError(f'Duplicate block id: {entry["id"]}')
        if not (BLOCKS/entry['path']).is_file():
            raise ValueError(f'Block {entry["id"]} has no file {entry["path"]}')
        result[entry['id']] = entry
    return result


def tidy(text):
    """Collapse blank-line runs left by empty conditionals; fenced examples stay verbatim."""
    lines, fenced, blank = [], False, False
    for line in text.split('\n'):
        if line.strip().startswith('```'):
            fenced = not fenced
        if not fenced and not line.strip():
            if blank:
                continue
            blank = True
        else:
            blank = False
        lines.append(line)
    return '\n'.join(lines)


def render(text, setup, *, vocab=None, used=None):
    """Render one source. `used`, when given, collects the block ids it inserted."""
    if vocab is None:
        from .recipes import VOCAB as vocab
    context = {**setup.context(), 'vocab': vocab}
    used = set() if used is None else used

    def block(ident, **parameters):
        entry = registry().get(ident)
        if entry is None:
            raise ValueError(f'Unknown workspace block: {ident}')
        used.add(ident)
        return environment().get_template(entry['path']).render(
            **context, **parameters, block=block).rstrip('\n')

    try:
        rendered = environment().from_string(text).render(**context, block=block)
    except UndefinedError as exc:
        raise ValueError(f'Unknown workspace template name: {exc}') from None
    except TemplateSyntaxError as exc:
        raise ValueError(f'Malformed workspace template: {exc}') from None
    if any(marker in rendered for marker in LEFTOVERS):
        raise ValueError('Malformed workspace template: markup survived rendering')
    return tidy(rendered)
