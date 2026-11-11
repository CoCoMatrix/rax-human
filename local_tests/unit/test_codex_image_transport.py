import base64
import copy
import cv2
import numpy as np
from harness.codex_cli.relay import native_jpeg_transport


def png(channels=3):
    a=np.random.default_rng(9).integers(0,256,(720,1280,channels),dtype=np.uint8)
    ok,b=cv2.imencode('.png',a)
    return 'data:image/png;base64,'+base64.b64encode(b).decode()


def test_native_shape_preserved_and_text_history_not_removed():
    url=png();p={'input':[{'role':'user','content':[{'type':'input_text','text':'exact original text'},
         {'type':'input_image','image_url':url,'detail':'original'}]},
         {'type':'reasoning','encrypted_content':'unchanged'},
         {'type':'function_call_output','call_id':'id','output':'unchanged'}]}
    before=copy.deepcopy(p);stats=native_jpeg_transport(p)
    assert stats['converted_images']==1 and stats['saved_bytes']>0
    out=p['input'][0]['content'][1]['image_url']
    assert out.startswith('data:image/jpeg;base64,')
    a=cv2.imdecode(np.frombuffer(base64.b64decode(out.split(',',1)[1]),np.uint8),cv2.IMREAD_UNCHANGED)
    assert a.shape==(720,1280,3)
    p['input'][0]['content'][1]['image_url']=url
    assert p==before


def test_transparency_remote_urls_and_nonimage_fields_untouched():
    p={'input':[{'content':[{'type':'input_image','image_url':png(4)},
       {'type':'input_image','image_url':'https://invalid.example/x.png'},
       {'type':'input_text','text':png()}]}]}
    before=copy.deepcopy(p);assert native_jpeg_transport(p)['converted_images']==0
    assert p==before


def test_corrupt_and_small_pngs_are_not_silently_changed():
    ok,b=cv2.imencode('.png',np.zeros((10,10,3),np.uint8))
    for value in ('data:image/png;base64,not_base64','data:image/png;base64,'+base64.b64encode(b).decode()):
        p={'input':[{'type':'input_image','image_url':value}]};before=copy.deepcopy(p)
        assert native_jpeg_transport(p)['converted_images']==0;assert p==before
