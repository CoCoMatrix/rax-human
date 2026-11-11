import json
import pytest
from services.controller import operator
from conftest import frontend


class Clock:
    now = 0.
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


def expire(root, request):
    clock = Clock()
    return operator.wait(root, request, 1, clock=clock, sleep=clock.sleep)


def test_late_reply_is_queryable_without_reopening_or_auto_approval(tmp_path):
    r = operator.open_request(tmp_path, 'help', 'Are the fingers clear for this one approach?')
    timeout = expire(tmp_path, r)
    assert timeout['request_id'] == r['id'] and timeout['done'] is False
    operator.reply(tmp_path, r['id'], 'Yes, this approach is clear.', done=True, by='onsite')
    result = operator.wait(tmp_path, r, 1)
    assert result['status'] == 'answered' and result['late'] is True
    assert result['request_id'] == r['id'] and result['done'] is True
    assert result['requires_fresh_observation'] is True
    saved = json.loads((operator.directory(tmp_path)/(r['id']+'.json')).read_text())
    assert saved['timed_out_at'] and saved['status'] == 'answered'
    with pytest.raises(ValueError, match='already answered'):
        operator.reply(tmp_path, r['id'], 'overwrite')


def test_negative_late_reply_never_becomes_approval(tmp_path):
    r=operator.open_request(tmp_path,'help','clear?');expire(tmp_path,r)
    operator.reply(tmp_path,r['id'],'Cannot confirm; hold.',done=False)
    result=operator.wait(tmp_path,r,1)
    assert result['done'] is False and result['reply'].startswith('Cannot')


def test_status_and_existing_request_rejoin_use_same_request(tmp_path):
    f=frontend(tmp_path);root=f.s.config.root
    try:
        r=operator.open_request(root,'help','one current approach');expire(root,r)
        operator.reply(root,r['id'],'Observed clear.',done=True)
        status=json.loads(f.call('status',{})['content'][0]['text'])
        assert status['operator_requests'][0]['request_id']==r['id']
        value=json.loads(f.call('request_operator',{'request_id':r['id'],'timeout_s':30})['content'][0]['text'])
        assert value['reply']=='Observed clear.' and value['late']
        assert len(list(operator.directory(root).glob('????????????.json')))==1
        assert f.backend.position==[0,0,0]
    finally:f.close()


def test_wait_timeout_preserves_an_answer_at_deadline(tmp_path):
    r=operator.open_request(tmp_path,'help','clear?');clock=Clock()
    def sleep(t):
        clock.sleep(t);operator.reply(tmp_path,r['id'],'At deadline',done=True)
    result=operator.wait(tmp_path,r,1,clock=clock,sleep=sleep)
    assert result['status']=='answered' and result['done']


def test_request_identity_cannot_change_or_cross_sessions(tmp_path):
    f=frontend(tmp_path)
    try:
        r=operator.open_request(tmp_path/'other','help','different scene')
        assert f.call('request_operator',{'request_id':r['id'],'timeout_s':30})['isError']
        assert f.call('request_operator',{'request_id':'../outside','timeout_s':30})['isError']
    finally:f.close()
