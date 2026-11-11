"""Read-only supported head depth queries; keep failures explicit."""
import json,sys
from api.runtime import Context
frame=sys.argv[1]
pixels=json.loads(sys.argv[2])
with Context() as ctx:
    for pixel in pixels:
        try:
            r=ctx.value('piper_pixel_to_position',camera='head',frame_id=frame,pixels=[pixel],radius=3)
            print(json.dumps(r['points']))
        except Exception as e: print(json.dumps({'pixel':pixel,'error':str(e)}))
