import json
import pytest
from services.controller.google_gemini import GoogleGeminiRouter, GOOGLE_MODEL
from services.controller.gemini import MODEL
from services.controller.config import Configuration
from services.controller.session import Session
from services.controller.frontend import Frontend
from harness.codex_cli.launcher import prepare


def response(model=GOOGLE_MODEL):
    return {'model':model,'choices':[{'message':{'role':'assistant','content':'{"ok":true}',
        'extra_content':{'google':{'thought_signature':'opaque'}}},'finish_reason':'stop'}],
        'usage':{'prompt_tokens':200,'completion_tokens':20,'total_tokens':250}}


def test_google_transport_pins_model_preserves_signature_and_counts_thinking():
    sent=[]
    router=GoogleGeminiRouter('private',send=lambda p:(sent.append(p) or response()))
    payload,n=router.prepare({'messages':[{'role':'user','content':'Inspect'}],
        'reasoning':{'effort':'low'},'response_format':{'type':'json_object'}})
    v,_=router.execute(payload,n)
    assert sent[0]['model']==GOOGLE_MODEL and sent[0]['reasoning_effort']=='low'
    assert 'provider' not in sent[0] and 'reasoning' not in sent[0] and sent[0]['stream'] is False
    assert v['model']==MODEL and v['provider_model']==GOOGLE_MODEL
    assert v['cost_usd'] is None and v['cost_upper_bound_usd']==pytest.approx(.0003375)
    assert v['message']['extra_content']['google']['thought_signature']=='opaque'
    router.prepare({'messages':[{'role':'user','content':'Inspect'},v['message']]})


@pytest.mark.parametrize('args',[{'temperature':1},{'reasoning':{'effort':'minimal'}},
    {'reasoning':{'max_tokens':10}},{'reasoning':{'effort':'low','exclude':True}}])
def test_unsupported_official_options_fail_before_io(args):
    r=GoogleGeminiRouter('private',send=lambda p:pytest.fail('No request allowed'))
    with pytest.raises(ValueError):r.prepare({'messages':[{'role':'user','content':'x'}],**args})


def test_unexpected_model_is_not_normalized_into_success():
    r=GoogleGeminiRouter('private',send=lambda p:response('other-model'))
    p,n=r.prepare({'messages':[{'role':'user','content':'x'}]})
    with pytest.raises(ValueError,match='unexpected model'):r.execute(p,n)


def test_native_frontend_accounts_estimates_separately_and_keeps_failed_reservations(tmp_path,monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT',str(tmp_path))
    c=Configuration(root=tmp_path/'s',task='gemini',robot='mock')
    class Backend:
        mutating=set()
        def tools(self):return []
        def status(self):return {}
    s=Session(c)
    try:
        f=Frontend(s,Backend(),GeminiRouterForTest())
        schema=next(t for t in f.tools() if t['name']=='gemini_generate')
        assert 'Google official' in schema['description']
        assert 'temperature' not in schema['inputSchema']['properties']
        assert schema['inputSchema']['properties']['reasoning']['properties']['effort']['enum']==['low','medium','high']
        from services.controller.frontend import GEMINI
        assert 'temperature' in GEMINI['inputSchema']['properties']  # Upstream transport unchanged.
        result=f.call('gemini_generate',{'messages':[{'role':'user','content':'x'}]})
        v=json.loads(result['content'][0]['text'])
        assert not result.get('isError')
        assert v['budget']['session_cost_reported_usd']==0
        assert v['budget']['session_cost_estimated_upper_bound_usd']>0
        assert v['budget']['session_cost_reserved_usd']==0
        assert v['budget']['session_cost_remaining_usd']<10
        s.charge(100,1000000)
        s.refund(50,{'total_tokens':50},1000000)
        assert s.budget()['session_cost_reserved_usd']==.001
    finally:s.close()


def GeminiRouterForTest():
    return GoogleGeminiRouter('unused',send=lambda p:response())


def test_estimated_budget_is_durable_and_overrun_blocks(tmp_path,monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT',str(tmp_path))
    c=Configuration(root=tmp_path/'s',task='gemini',robot='mock')
    s=Session(c)
    s.charge(100,1000000)
    s.refund(50,{'total_tokens':50},1000000,cost_upper_bound=.0005)
    s.close()
    s=Session(c)
    try:
        assert s.budget()['session_cost_estimated_upper_bound_usd']==.0005
        s.charge(100,1000000)
        with pytest.raises(RuntimeError,match='exceeded reservation'):
            s.refund(50,{'total_tokens':50},1000000,cost_upper_bound=.002)
        assert s.budget()['session_cost_blocked']
        with pytest.raises(RuntimeError,match='exhausted'):s.charge(1,1)
    finally:s.close()


@pytest.mark.parametrize('prompt',[None,True,-1,251])
def test_invalid_google_usage_cannot_settle_budget(prompt):
    r=response();r['usage']['prompt_tokens']=prompt
    router=GoogleGeminiRouter('unused',send=lambda p:r)
    payload,n=router.prepare({'messages':[{'role':'user','content':'Inspect'}]})
    with pytest.raises(ValueError,match='prompt usage'):router.execute(payload,n)


def test_codex_primary_and_native_gemini_workspace_are_separate(tmp_path,monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT',str(tmp_path))
    c=Configuration(root=tmp_path/'s',task='gemini',robot='piper',agent_provider='codex-relay',agent_model='gpt-6-astra')
    w=prepare(c,tmp_path/'main-key','https://relay.example/v1',gemini_key_file=tmp_path/'gemini-key')
    tools=json.loads((w/'MCP_CONTRACT.json').read_text())['tools']
    assert 'gemini_generate' in tools and 'codex_vision_generate' not in tools
    assert (w/'.agents/skills/gemini/SKILL.md').exists()
    assert not (w/'.agents/skills/codex-vision').exists()
    assert 'Google official' in (w/'.agents/skills/gemini/references/api.md').read_text()
    settings=json.loads((c.root/'codex-relay.json').read_text())
    assert settings['primary_model']=='gpt-6-astra' and settings['vision_provider']=='google'
    assert not (c.root/'state.json').exists()
