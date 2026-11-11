"""Persistent, file-only SAM3 broker. No SDK, network, robot or motion access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time

MODEL = Path('/home/ubuntu/code_junqi/model-store/sam3/sam3.pt')
MODEL_SHA = '9999e2341ceef5e136daa386eecb55cb414446a00ac2b55eb2dfd2f7c3cf8c9e'
DEFAULT_PROMPTS = {'cup': 'cup', 'coaster': 'round coaster'}
CONFIDENT_SCORE = .5
CANDIDATE_SCORE = .05


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def atomic(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.partial')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    os.replace(tmp, path)


def contained(workspace, value, roots=('output/runs', 'runtime/frames')):
    p = Path(value)
    p = (p if p.is_absolute() else workspace / p).resolve(strict=True)
    if not any(p.is_relative_to((workspace / r).resolve()) for r in roots):
        raise ValueError('PERCEPTION_INPUT_OUTSIDE_FRAME_ROOTS')
    if not p.is_file() or p.stat().st_size > 16 * 1024 * 1024:
        raise ValueError('PERCEPTION_INPUT_SIZE')
    return p


def validate_request(workspace, request, now_ns=None):
    if not re.fullmatch('[a-f0-9]{32}', request.get('id', '')):
        raise ValueError('PERCEPTION_REQUEST_ID')
    age = ((time.time_ns() if now_ns is None else now_ns)-request['created_ns'])/1e9
    if not 0 <= age <= 30:
        raise ValueError('PERCEPTION_REQUEST_EXPIRED')
    p = contained(workspace, request['snapshot'], ('output/runs',))
    if digest(p) != request['snapshot_sha256']:
        raise ValueError('PERCEPTION_SNAPSHOT_CHANGED')
    snapshot = json.loads(p.read_text())
    images = {'head': contained(workspace, snapshot.get('geometry_head') or snapshot['frames']['head']),
              'left': contained(workspace, snapshot['frames']['left'])}
    prompts = request.get('prompts', DEFAULT_PROMPTS)
    if (set(prompts) != set(DEFAULT_PROMPTS) or
            any(not isinstance(v, str) or not 1 <= len(v) <= 100 for v in prompts.values())):
        raise ValueError('PERCEPTION_PROMPTS')
    # Paths are data only. They are never evaluated or passed to a shell.
    boxes = request.get('boxes', {})
    if not isinstance(boxes, dict) or set(boxes)-{'head','left'}:
        raise ValueError('PERCEPTION_BOX_CAMERAS')
    for view in boxes.values():
        if not isinstance(view, dict) or set(view)-set(DEFAULT_PROMPTS):
            raise ValueError('PERCEPTION_BOX_CONCEPTS')
        for box in view.values():
            if (not isinstance(box, list) or len(box)!=4 or
                    any(type(v) not in (int,float) or not 0<=v<=1 for v in box) or
                    box[2]<=0 or box[3]<=0 or box[0]-box[2]/2<0 or box[0]+box[2]/2>1 or
                    box[1]-box[3]/2<0 or box[1]+box[3]/2>1):
                raise ValueError('PERCEPTION_BOX_NORMALIZED_CXCYWH')
    return snapshot, images, prompts, boxes


class Engine:
    def __init__(self):
        if digest(MODEL) != MODEL_SHA:
            raise ValueError('SAM3_CHECKPOINT_IDENTITY')
        import torch
        from sam3.model_builder import build_sam3_image_model
        from sam3.model.sam3_image_processor import Sam3Processor
        self.torch = torch
        self.processor = Sam3Processor(build_sam3_image_model(
            checkpoint_path=str(MODEL), load_from_HF=False, device='cuda'), confidence_threshold=CANDIDATE_SCORE)

    def infer(self, images, output, prompts=None, boxes=None):
        from PIL import Image, ImageDraw
        import numpy as np
        output = Path(output); output.mkdir(parents=True, exist_ok=False)
        result = {'model_sha256': MODEL_SHA, 'views': {}, 'motion_permit': False,
                  'metric_geometry': False, 'contact_proof': False, 'directory': str(output),
                  'confident_score_threshold':CONFIDENT_SCORE, 'candidate_score_threshold':CANDIDATE_SCORE}
        started = time.monotonic()
        with self.torch.inference_mode(), self.torch.autocast('cuda', dtype=self.torch.bfloat16):
            for camera, path in images.items():
                data = Path(path).read_bytes()
                import io
                image = Image.open(io.BytesIO(data)).convert('RGB')
                state = self.processor.set_image(image)
                overlay = image.copy(); draw = ImageDraw.Draw(overlay)
                objects = []; candidates = []
                for i, (concept, prompt) in enumerate((prompts or DEFAULT_PROMPTS).items()):
                    # Important if a previous concept used geometric prompts.
                    self.processor.reset_all_prompts(state)
                    box = (boxes or {}).get(camera, {}).get(concept)
                    if box is not None:
                        if len(box) != 4 or any(not 0 <= v <= 1 for v in box):
                            raise ValueError('SAM3_NORMALIZED_BOX')
                        prediction = self.processor.add_geometric_prompt(box=box, label=True, state=state)
                    else:
                        prediction = self.processor.set_text_prompt(state=state, prompt=prompt)
                    scores = prediction['scores'].float().cpu().numpy()
                    coords = prediction['boxes'].float().cpu().numpy()
                    masks = prediction['masks'].cpu().numpy()
                    for j in np.argsort(-scores)[:8]:
                        name = f'{camera}-{i}-{int(j)}-mask.png'
                        mask = np.squeeze(masks[j]) > 0
                        Image.fromarray(mask.astype('uint8')*255).save(output/name)
                        xyxy = [float(v) for v in coords[j]]
                        if box is not None:
                            # SAM3 can return another instance outside a positive box.
                            # Keep only instances whose centre belongs to the reviewed ROI.
                            cx, cy = (xyxy[0]+xyxy[2])/2/image.width, (xyxy[1]+xyxy[3])/2/image.height
                            if not (box[0]-box[2]/2 <= cx <= box[0]+box[2]/2 and
                                    box[1]-box[3]/2 <= cy <= box[1]+box[3]/2):
                                (output/name).unlink()
                                continue
                        destination = objects if float(scores[j]) >= CONFIDENT_SCORE else candidates
                        destination.append({'confidence_status':'confident' if destination is objects else 'low_confidence_candidate',
                            'concept': concept, 'prompt': prompt, 'score': float(scores[j]),
                            'box_xyxy': xyxy, 'mask': name, 'mask_sha256': digest(output/name),
                            'mask_area_px': int(mask.sum()), 'geometric_prompt_used': box is not None,
                            'semantic_source': 'reviewer_box' if box is not None else 'model_text_prompt'})
                        draw.rectangle(xyxy, outline=(60, 220, 130), width=2)
                        draw.text((xyxy[0], max(0, xyxy[1]-12)), f'{concept} {float(scores[j]):.2f}' + (' candidate' if destination is candidates else ''), fill=(255, 80, 80))
                overlay.save(output/(camera+'-segmentation.jpg'))
                result['views'][camera] = {'source_sha256': hashlib.sha256(data).hexdigest(),
                    'shape_wh': list(image.size), 'objects': objects, 'candidates':candidates,
                    'missing': [c for c in (prompts or DEFAULT_PROMPTS) if not any(o['concept']==c for o in objects)]}
        result['inference_s'] = time.monotonic()-started
        return result


def pending_requests(queue):
    queue=Path(queue)
    return [p for p in sorted((queue/'requests').glob('*.json'))
            if not (queue/'responses'/(p.stem+'.json')).exists()][:32]


def serve(workspace, stop_after_s=0):
    workspace = Path(workspace).resolve()
    if not workspace.is_relative_to('/home/ubuntu/code_junqi'):
        raise ValueError('PERSONAL_WORKSPACE_REQUIRED')
    queue = workspace/'output/perception-queue'
    requests = queue/'requests'; requests.mkdir(parents=True, exist_ok=True)
    engine = Engine()
    atomic(queue/'ready.json', {'ready': True, 'pid': os.getpid(), 'at_ns': time.time_ns(),
                               'model_sha256': MODEL_SHA, 'motion_permit': False})
    deadline = time.monotonic()+stop_after_s if stop_after_s else float('inf')
    while time.monotonic() < deadline:
        for p in pending_requests(queue):
            if not re.fullmatch('[a-f0-9]{32}', p.stem):
                continue
            response = queue/'responses'/(p.stem+'.json')
            if response.exists(): continue
            try:
                if p.is_symlink() or p.stat().st_size > 16384:
                    raise ValueError('PERCEPTION_REQUEST_FILE')
                req = json.loads(p.read_text())
                if req['id'] != p.stem: raise ValueError('PERCEPTION_REQUEST_ID')
                snapshot, images, prompts, boxes = validate_request(workspace, req)
                output = queue/'artifacts'/req['id']
                result = engine.infer(images, output, prompts, boxes)
                result.update(id=req['id'], frame_id=snapshot['observation']['frame_id'],
                    snapshot_q=snapshot['q'], snapshot_sha256=req['snapshot_sha256'],
                    source_received_ns=snapshot['observation'].get('head', {}).get('host_received_unix_ns'),
                    finished_ns=time.time_ns(), historical_snapshot=True, ok=True)
                atomic(output/'result.json', result)
            except Exception as exc:
                result = {'ok': False, 'error': str(exc), 'motion_permit': False, 'id': p.stem}
            atomic(response, result)
            if result.get('ok'):
                atomic(queue/'latest.json', {'id':p.stem,'finished_ns':result['finished_ns']})
        time.sleep(.05)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--workspace', required=True)
    p.add_argument('--seconds', type=int, default=0); a = p.parse_args()
    serve(a.workspace, a.seconds)
