"""Turn one goal photo into the fixed goal description with Gemini 3.8 Flash.

Host-side only (the controller sandbox has no network). Uses RAX's existing Google
router, key file and proxy. The model's answer is data: it is validated by goal_spec
and must be confirmed by the operator before any run uses it.
"""
from __future__ import annotations

import base64
import io
import json
import re
import time

from .goal_spec import gemini_schema

MAX_UPLOAD_BYTES = 3 * 1024 * 1024
MAX_SIDE = 1600

PROMPT = """You read one photo of toy cube blocks on a table and describe their arrangement as data.

The photo is upright: unless it was taken from straight above, gravity points to the bottom edge of the image and the cubes rest on a surface below them.

The scene should contain exactly four cubes of about 6 cm, one each of red, orange, green and yellow. Each cube has round white stickers on its faces; ignore the stickers. Ignore robot arms, grippers, hands, shadows and anything that is not one of the four cubes.

Work in this order.

Step 1. Find the cubes and their colours (visible_colors).

Step 2. List every pair of cubes that are in contact (contacts):
- "on_top": cube "a" rests on top of cube "b". Check which cube really carries it: the cube directly underneath, whose top face it covers.
- "side_by_side": cubes "a" and "b" stand at the same height next to each other, face to face, touching or less than half a cube apart. Two cubes that only meet diagonally at an edge or corner are NOT side by side.

Step 3. Place the stacks on a grid (stacks) so that the grid agrees with step 2: side_by_side cubes are in cells that share an edge (same row and neighbouring col, or same col and neighbouring row), and a cube resting on another cube is in the same cell.
- One stack is one grid cell. "blocks" lists that stack from the bottom cube to the top cube. A cube standing alone on the table is a stack of one.
- The grid lines follow the faces of the cubes, not the edges of the photo.
- Frontal view (you look straight at one face of the cubes): col grows from left to right, row grows from near the camera to far away.
- Corner-on view (the cubes are turned about 45 degrees to the camera, so you see two side faces of a cube and the rows of cubes run diagonally in the photo): one step toward the upper right of the photo increases row by 1; one step toward the upper left decreases col by 1; one step toward the lower right increases col by 1.
- Leave one empty cell for a gap of about one cube or more.
- Use the smallest grid: the smallest col is 0 and the smallest row is 0.
- A lower cube can be partly hidden by the cube on top of it or in front of it. Use the visible side faces to identify it.

Other fields:
- image_usable: false when you cannot find all four colours exactly once, or the arrangement cannot be read.
- spacing: "tight" when neighbouring cubes touch or nearly touch, "loose" when neighbouring cubes are clearly apart.
- viewpoint: "top_down", "frontal" (straight at a face, from the side or from above at an angle) or "corner_on".
- confidence: 0 to 1 for the whole description.
- uncertain: short notes on anything you had to guess, empty when nothing.
- summary: one short sentence in Chinese describing the arrangement.

Report what the photo shows. Do not invent cubes that are not there."""


def load_image(data_url):
    """Validate an uploaded data URL and return clean JPEG bytes (EXIF applied, bounded size)."""
    from PIL import Image, ImageOps
    match = re.fullmatch(r'data:(image/jpeg|image/png);base64,([A-Za-z0-9+/=]+)', data_url) if isinstance(data_url, str) else None
    if not match:
        raise ValueError('请上传 JPG 或 PNG 图片')
    raw = base64.b64decode(match[2], validate=True)
    if not raw or len(raw) > MAX_UPLOAD_BYTES:
        raise ValueError('图片为空或超过 3 MB')
    with Image.open(io.BytesIO(raw)) as probe:
        if probe.format not in ('JPEG', 'PNG') or getattr(probe, 'n_frames', 1) != 1 or probe.width * probe.height > 6000 * 6000:
            raise ValueError('不支持的图片格式或尺寸')
        probe.verify()
    with Image.open(io.BytesIO(raw)) as frame:
        frame = ImageOps.exif_transpose(frame).convert('RGB')
        if min(frame.size) < 200:
            raise ValueError('图片太小，请上传原始照片')
        frame.thumbnail((MAX_SIDE, MAX_SIDE))
        out = io.BytesIO()
        frame.save(out, 'JPEG', quality=88)
        return out.getvalue(), {'width': frame.width, 'height': frame.height}


def parse_answer(text):
    if not isinstance(text, str) or not text.strip():
        raise ValueError('Gemini 没有返回内容')
    body = text.strip()
    fenced = re.fullmatch(r'```(?:json)?\s*(.*?)\s*```', body, re.S)
    value = json.loads(fenced[1] if fenced else body)
    if not isinstance(value, dict):
        raise ValueError('Gemini 返回的不是 JSON 对象')
    return value


def _ask(jpeg, system, question, name, schema, *, key_file, proxy_url, effort, send):
    from services.controller.gemini import load_key
    from services.controller.google_gemini import GoogleGeminiRouter, GOOGLE_MODEL
    router = GoogleGeminiRouter(load_key(key_file), proxy_url=proxy_url, send=send)
    arguments = {
        'messages': [
            {'role': 'system', 'content': system},
            {'role': 'user', 'content': [
                {'type': 'text', 'text': question},
                {'type': 'image', 'data': base64.b64encode(jpeg).decode('ascii'), 'mimeType': 'image/jpeg'}]}],
        # Thinking counts against this limit: a corner-on photo used the whole 4096 and cut the JSON off.
        'max_tokens': 16000,
        'response_format': {'type': 'json_schema', 'json_schema': {'name': name, 'strict': True, 'schema': schema}},
        'reasoning': {'effort': effort},
    }
    started = time.monotonic()
    try:
        payload, reservation = router.prepare(arguments)
        value, used = router.execute(payload, reservation)
    finally:
        router.close()
    if value.get('finish_reason') == 'length':
        raise ValueError('Gemini 的回答被截断，请重试')
    answer = parse_answer(value['message'].get('content'))
    return {'answer': answer, 'model': GOOGLE_MODEL, 'tokens': used,
            'cost_upper_bound_usd': value.get('cost_upper_bound_usd'),
            'duration_s': time.monotonic() - started, 'finish_reason': value.get('finish_reason')}


def analyze(jpeg, *, key_file, proxy_url, effort='low', send=None):
    """One model call. Photos are taken as upright (operator rule); no orientation step."""
    return _ask(jpeg, PROMPT, 'Describe the block arrangement in this goal photo.', 'block_goal', gemini_schema(),
                key_file=key_file, proxy_url=proxy_url, effort=effort, send=send)


def analyze_votes(jpeg, *, key_file, proxy_url, votes=3, send=None):
    """Read the same photo several times in parallel. One reading of a corner-on photo was
    wrong about half the time; disagreement between readings is how the operator finds out."""
    from concurrent.futures import ThreadPoolExecutor
    started = time.monotonic()

    def one(_):
        try:
            return analyze(jpeg, key_file=key_file, proxy_url=proxy_url, send=send)
        except Exception as exc:  # noqa: BLE001 - one failed reading must not hide the others
            return {'error': type(exc).__name__ + ': ' + str(exc)[:200]}
    with ThreadPoolExecutor(votes) as pool:
        results = list(pool.map(one, range(votes)))
    good = [r for r in results if 'answer' in r]
    costs = [r.get('cost_upper_bound_usd') for r in good]
    return {'results': results, 'calls': votes, 'duration_s': time.monotonic() - started,
            'tokens': sum(r['tokens'] for r in good),
            'cost_upper_bound_usd': None if (not good or None in costs) else sum(costs)}
