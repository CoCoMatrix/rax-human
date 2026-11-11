"""Offline/read-only SAM3 evidence on saved views, in its own existing environment."""
import argparse
import hashlib
import json
from pathlib import Path
import time

MODEL = Path('/home/ubuntu/code_junqi/model-store/sam3/sam3.pt')
MODEL_SHA = '9999e2341ceef5e136daa386eecb55cb414446a00ac2b55eb2dfd2f7c3cf8c9e'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--prompts', default='cup,round coaster')
    a = p.parse_args(); source = Path(a.source).resolve(); output = Path(a.output).resolve()
    workspace = Path('/home/ubuntu/code_junqi')
    if not source.is_relative_to(workspace) or not output.is_relative_to(workspace) or output.exists():
        raise ValueError('Use saved workspace images and a new output directory')
    prompts = [s.strip() for s in a.prompts.split(',') if s.strip()]
    if not 1 <= len(prompts) <= 5 or any(len(s) > 100 for s in prompts): raise ValueError('1–5 short text prompts required')
    started = time.monotonic()
    with MODEL.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != MODEL_SHA: raise ValueError('SAM3 checkpoint identity mismatch')
    import torch
    import numpy as np
    from PIL import Image, ImageDraw
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    model = build_sam3_image_model(checkpoint_path=str(MODEL), load_from_HF=False, device='cuda')
    processor = Sam3Processor(model, confidence_threshold=.5)
    loaded = time.monotonic(); output.mkdir(parents=True)
    result = {'source': str(source), 'model_sha256': MODEL_SHA, 'load_s': loaded-started,
              'metric_geometry': False, 'contact_proof': False, 'motion_permit': False, 'views': {}}
    colors = [(64, 190, 129), (246, 184, 64), (74, 147, 233), (219, 96, 150), (160, 109, 218)]
    with torch.inference_mode(), torch.autocast('cuda', dtype=torch.bfloat16):
        for camera in ('head', 'left'):
            path = source / (camera + '.jpg')
            if not path.exists(): continue
            image = Image.open(path).convert('RGB'); overlay = image.copy(); draw = ImageDraw.Draw(overlay)
            state = processor.set_image(image); found = []
            for i, prompt in enumerate(prompts):
                prediction = processor.set_text_prompt(state=state, prompt=prompt)
                boxes = prediction['boxes'].float().cpu().numpy()
                scores = prediction['scores'].float().cpu().numpy()
                masks = prediction['masks'].cpu().numpy()
                for j, (box, score) in enumerate(zip(boxes[:8], scores[:8])):
                    mask = np.squeeze(masks[j])
                    mask_path = f'{camera}-{i}-{j}-mask.png'
                    Image.fromarray((mask > 0).astype('uint8') * 255).save(output / mask_path)
                    coords = [float(v) for v in box]
                    draw.rectangle(coords, outline=colors[i], width=3)
                    draw.text((coords[0]+3, max(0, coords[1]-12)), f'{prompt} {float(score):.2f}', fill=colors[i])
                    found.append({'prompt': prompt, 'score': float(score), 'box_xyxy': coords, 'mask': mask_path})
            overlay.save(output / (camera + '-segmentation.jpg'))
            result['views'][camera] = {'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'shape_wh': list(image.size), 'objects': found}
    result.update(total_s=time.monotonic()-started, inference_s=time.monotonic()-loaded, finished_ns=time.time_ns())
    (output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({'passed': True, 'views': {k:len(v['objects']) for k,v in result['views'].items()}, 'total_s':result['total_s']}))


if __name__ == '__main__': main()
