import http.client,json,socket,threading,subprocess,sys,time
from pathlib import Path
from harness.codex_cli.relay import ResponsesRelay

def test_outer_relay_preserves_affinity_and_response_headers(tmp_path,monkeypatch):
 captured=[]
 class Reply:
  status=200
  headers={'Content-Type':'text/event-stream','x-request-id':'upstream-id','x-codex-turn-state':'opaque-next-state','Retry-After':'2'}
  def __enter__(self):return self
  def __exit__(self,*a):pass
  def read1(self,n):
   if getattr(self,'done',False):return b''
   self.done=True;return b'data: {"type":"response.completed","response":{"status":"completed"}}\n\n'
 class Opener:
  def open(self,request,timeout):captured.append(request);return Reply()
 monkeypatch.setattr('harness.codex_cli.relay.urllib.request.build_opener',lambda *a:Opener())
 relay=ResponsesRelay(tmp_path/'r.sock','sk-test-secret','https://up.example/v1','gpt-6-astra','high',tmp_path,3,proxy_url='')
 relay.start();c=http.client.HTTPConnection('private');c.sock=socket.socket(socket.AF_UNIX);c.sock.connect(str(relay.path))
 try:
  c.request('POST','/v1/responses',json.dumps({'model':'gpt-6-astra','input':'test'}),{'Content-Type':'application/json','session_id':'session-A','x-codex-turn-state':'opaque-state','x-custom-future':'future','Connection':'close, x-hop','x-hop':'remove'})
  rep=c.getresponse();rep.read();h={k.lower():v for k,v in captured[0].header_items()}
  assert h['session_id']=='session-A'
  assert h['x-codex-turn-state']=='opaque-state'
  assert h['x-custom-future']=='future'
  assert 'x-hop' not in h
  assert h['authorization']=='Bearer sk-test-secret'
  assert rep.getheader('x-codex-turn-state')=='opaque-next-state'
  assert rep.getheader('x-request-id')=='upstream-id'
  assert rep.getheader('Retry-After')=='2'
 finally:c.close();relay.close()

def test_inner_proxy_forwards_headers_and_exact_body():
 # Execute definitions only; never start entrypoint/Codex or bind production ports.
 from types import ModuleType
 source=(Path(__file__).resolve().parents[2]/'harness/codex_cli/sandbox_entry.py').read_text()
 module=ModuleType('entry_test');exec(source.split('\nserver = ',1)[0],module.__dict__)
 captured={}
 class Reply:
  status=200
  def getheader(self,k,default=None):return {'Content-Type':'text/event-stream'}.get(k,default)
  def getheaders(self):return [('Content-Type','text/event-stream'),('x-codex-turn-state','next-state')]
  def read1(self,n):return b''
 class Conn:
  def __init__(self,*a,**k):pass
  def request(self,method,path,body,headers):captured.update(body=body,headers={k.lower():v for k,v in headers.items()})
  def getresponse(self):return Reply()
  def close(self):pass
 module.UnixConnection=Conn
 server=module.http.server.ThreadingHTTPServer(('127.0.0.1',0),module.Handler)
 thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
 c=http.client.HTTPConnection(*server.server_address)
 try:
  body=b'{ "model": "test", "input": "same bytes" }'
  c.request('POST','/v1/responses',body,{'session_id':'session-B','x-codex-turn-state':'state-B','x-new-header':'future'})
  rep=c.getresponse();rep.read()
  assert captured['body']==body
  assert captured['headers']['session_id']=='session-B'
  assert captured['headers']['x-codex-turn-state']=='state-B'
  assert captured['headers']['x-new-header']=='future'
  assert rep.getheader('x-codex-turn-state')=='next-state'
 finally:c.close();server.shutdown();server.server_close();thread.join()
