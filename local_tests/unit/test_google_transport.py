import io,json,ssl
import pytest
from urllib.error import HTTPError
from services.controller import google_gemini as g

class Sock:
    def settimeout(self,x):self.timeout=x
class Response:
    status=200;reason='OK';headers={}
    def read(self,n):return b'{"model":"gemini-3.8-flash"}'
class Connection:
    def __init__(self,connect_error=None,send_error=None,response=None):
        self.sock=None;self.connect_error=connect_error;self.send_error=send_error;self.reply=response or Response();self.posts=[];self.closed=False
    def set_tunnel(self,*args):self.tunnel=args
    def connect(self):
        if self.connect_error:raise self.connect_error
        self.sock=Sock()
    def request(self,*args,**kwargs):
        self.posts.append((args,kwargs))
        if self.send_error:raise self.send_error
    def getresponse(self):return self.reply
    def close(self):self.closed=True;self.sock=None

def setup(monkeypatch,connections):
    made=[]
    def factory(*a,**kw):
        c=connections[len(made)];made.append(c);return c
    monkeypatch.setattr(g.http.client,'HTTPSConnection',factory)
    monkeypatch.setattr(g.time,'sleep',lambda _:None)
    return g.GoogleGeminiRouter('secret-test',proxy_url='http://127.0.0.1:17890'),made

def test_each_request_connects_and_closes_independently(monkeypatch,capsys):
    c=Connection();second=Connection();r,made=setup(monkeypatch,[c,second]);r._google_http({'x':1});r._google_http({'x':2})
    assert len(made)==2 and len(c.posts)==len(second.posts)==1
    assert c.closed and second.closed and r._connection is None
    assert c.tunnel==second.tunnel==('generativelanguage.googleapis.com',443)
    logs=[json.loads(x) for x in capsys.readouterr().err.splitlines()]
    assert all(not x['reused_connection'] for x in logs)
    assert all(x['connection_policy']=='per_request' for x in logs)
    assert 'secret-test' not in json.dumps(logs)
    r.close();assert c.closed and r._connection is None

def test_connect_retry_never_sends_failed_connection(monkeypatch):
    bad=Connection(connect_error=ssl.SSLEOFError('EOF'));good=Connection();r,made=setup(monkeypatch,[bad,good]);r._google_http({})
    assert len(made)==2 and bad.closed and not bad.posts and len(good.posts)==1

def test_connect_attempts_are_bounded(monkeypatch):
    cs=[Connection(connect_error=TimeoutError()) for _ in range(3)];r,made=setup(monkeypatch,cs)
    with pytest.raises(TimeoutError):r._google_http({})
    assert len(made)==3 and all(c.closed and not c.posts for c in cs)

def test_certificate_failure_is_not_retried(monkeypatch):
    c=Connection(connect_error=ssl.SSLCertVerificationError('bad cert'));r,made=setup(monkeypatch,[c])
    with pytest.raises(ssl.SSLCertVerificationError):r._google_http({})
    assert len(made)==1 and not c.posts

def test_post_failure_is_not_replayed(monkeypatch):
    c=Connection(send_error=TimeoutError());r,made=setup(monkeypatch,[c])
    with pytest.raises(TimeoutError):r._google_http({})
    assert len(made)==1 and len(c.posts)==1 and c.closed

def test_http_quota_error_preserved_not_retried(monkeypatch):
    resp=Response();resp.status=429;resp.reason='Too Many Requests';resp.headers={'Retry-After':'30'}
    resp.read=lambda n:b'{"error":{"status":"RESOURCE_EXHAUSTED"}}'
    c=Connection(response=resp);r,made=setup(monkeypatch,[c])
    with pytest.raises(HTTPError) as e:r._google_http({})
    assert e.value.code==429 and e.value.headers['Retry-After']=='30'
    assert b'RESOURCE_EXHAUSTED' in e.value.read() and len(c.posts)==1 and c.closed

def test_read_timeout_not_replayed(monkeypatch):
    resp=Response()
    def fail(n):raise TimeoutError('read timed out')
    resp.read=fail;c=Connection(response=resp);r,made=setup(monkeypatch,[c])
    with pytest.raises(TimeoutError):r._google_http({})
    assert len(made)==1 and len(c.posts)==1 and c.closed


def test_disconnect_is_not_replayed_but_next_call_can_connect(monkeypatch):
    class ClosedResponse(Connection):
        def getresponse(self):raise g.http.client.RemoteDisconnected('closed idle connection')
    bad=ClosedResponse();good=Connection();r,made=setup(monkeypatch,[bad,good])
    with pytest.raises(g.http.client.RemoteDisconnected):r._google_http({})
    assert len(made)==1 and len(bad.posts)==1 and bad.closed
    r._google_http({})
    assert len(made)==2 and len(good.posts)==1 and good.closed
