"""Run native Codex harness through CoCoMatrix Sub2API."""
from pathlib import Path
import os, sys
from ..teacher_client import load_api_key
from .__main__ import main

def run():
    home=Path('/home/ubuntu/code_junqi/.codex-astra-fastwam-api');home.mkdir(mode=0o700,parents=True,exist_ok=True)
    key=load_api_key(Path(os.environ.get('JUNQI_ASTRA_KEY_PATH','/home/ubuntu/code_junqi/webkey.py')))
    model=os.environ.get('JUNQI_CODEX_MODEL','gpt-6-astra'); effort=os.environ.get('JUNQI_CODEX_EFFORT','high')
    config=f'''model = "{model}"
model_provider = "OpenAI"
model_reasoning_effort = "{effort}"
openai_base_url = "https://sub2api.cocomatrix.cn"
forced_login_method = "api"

[model_providers.OpenAI]
name = "CoCoMatrix Sub2API"
base_url = "https://sub2api.cocomatrix.cn"
wire_api = "responses"
requires_openai_auth = false
experimental_bearer_token = "{key}"
http_headers = {{ "x-openai-actor-authorization" = "local-image-extension" }}
'''
    path=home/'config.toml'; prior=path.read_text() if path.exists() else ''
    if '[projects.' in prior: config += '\n\n'+prior[prior.index('[projects.'):]
    path.write_text(config);path.chmod(0o600)
    os.environ['JUNQI_ASTRA_KEY_PATH']='/home/ubuntu/code_junqi/webkey.py'
    os.environ['HTTPS_PROXY']='http://127.0.0.1:7890';os.environ['HTTP_PROXY']='http://127.0.0.1:7890';os.environ['NO_PROXY']='127.0.0.1,localhost,::1'
    main([*sys.argv[1:],'--codex-home',str(home),'--model-provider','OpenAI'])
if __name__=='__main__':run()
