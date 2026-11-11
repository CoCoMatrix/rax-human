"""Request same-frame SAM3 through the file-only host broker; never motion."""
import json
from pathlib import Path
import time
import uuid

from scene_contract import bind_result, digest


def analyze(snapshot_path, timeout_s=20, queue='output/perception-queue', prompts=None, boxes=None):
    if not 0 < timeout_s <= 30: raise ValueError('PERCEPTION_TIMEOUT_LIMIT')
    snapshot_path = Path(snapshot_path).resolve()
    snapshot = json.loads(snapshot_path.read_text())
    queue = Path(queue)
    if not (queue/'ready.json').is_file():
        raise ValueError('SAM3_BROKER_UNAVAILABLE; no silent no-segmentation fallback')
    ident = uuid.uuid4().hex
    request = {'id': ident, 'created_ns': time.time_ns(), 'snapshot': str(snapshot_path),
               'snapshot_sha256': digest(snapshot_path)}
    if prompts is not None: request['prompts'] = prompts
    if boxes is not None: request['boxes'] = boxes
    path = queue/'requests'/(ident+'.json'); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.partial'); temporary.write_text(json.dumps(request)); temporary.replace(path)
    response = queue/'responses'/(ident+'.json'); deadline = time.monotonic()+timeout_s
    while not response.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError('SAM3_RESULT_TIMEOUT; no motion submitted')
        time.sleep(.05)
    result = json.loads(response.read_text())
    if result.get('id') != ident or not result.get('ok'):
        raise ValueError('SAM3_RESULT_FAILED: '+str(result.get('error')))
    if result.get('snapshot_sha256') != digest(snapshot_path):
        raise ValueError('SAM3_SNAPSHOT_CHANGED')
    result=bind_result(snapshot,result)
    if snapshot['observation'].get('head',{}).get('native_geometry'):
        from native_geometry import associate_segmentation
        result['native_geometry']=associate_segmentation(snapshot,result)
    return result
