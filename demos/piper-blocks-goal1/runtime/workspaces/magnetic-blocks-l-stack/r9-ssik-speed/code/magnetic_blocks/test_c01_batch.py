from types import SimpleNamespace
from unittest.mock import patch
import pytest
import assembly_batch as b
from assembly_plan import NoPlan

def actions(n):
 rows=[];a=[0.]*14
 for i in range(n):
  q=a.copy();q[0]+=.1;rows.append(dict(kind='arm',stage='green-hover',color='green',start14=a,target14=q));a=q
 return rows

def test_long_batch_splits_to_checked_batches_not_all_singles():
 a=actions(6);seen=[];e=SimpleNamespace(payload=None,scene={},planner=SimpleNamespace(check=lambda *a,**kw:pytest.fail('All fit as batches')))
 def preview(e,g,start):
  seen.append((len(g),start[0]))
  if len(g)>3:raise ValueError('Merged path exceeds native600-sample horizon')
  return {'target14':g[-1]['target14']}
 with patch.object(b,'preview',side_effect=preview):chunks,end=b.checked_partition(e,a,a[0]['start14'],known_oversized=True)
 assert [len(g) for g in chunks]==[3,3] and end==a[-1]['target14']
 assert seen==[(3,0.),(3,pytest.approx(.3))]

def test_non_horizon_refusal_is_not_hidden_by_partition():
 a=actions(6);e=SimpleNamespace()
 with patch.object(b,'preview',side_effect=NoPlan('curve collision')),pytest.raises(NoPlan,match='curve collision'):
  b.checked_partition(e,a,a[0]['start14'])

def test_partition_never_dispatches_and_validates_single_remainder():
 a=actions(3);calls=[];e=SimpleNamespace(payload=None,scene={},planner=SimpleNamespace(check=lambda *a,**kw:calls.append(a)))
 with patch.object(b,'preview',return_value={'target14':a[1]['target14']}):chunks,end=b.checked_partition(e,a,a[0]['start14'],known_oversized=True)
 assert [len(g) for g in chunks]==[2,1] and len(calls)==1 and end==a[-1]['target14']

# Existing direct-batch tests cover ambiguous dispatch no replay; add recursive
# partition coverage with the same execution harness and a synthetic failure.
def test_failed_first_partition_never_dispatches_second_or_replays():
 import test_direct_batch as tb
 h=tb.Execution();h.setUp()
 try:
  a=actions(4);e=h.e
  with patch.object(b,'preview',side_effect=ValueError('Merged path exceeds native600-sample horizon')),patch.object(b,'checked_partition',return_value=([a[:2],a[2:]],a[-1]['target14'])),patch.object(e,'run',return_value={'ok':False,'why':'uncertain','control_uncertain':True}) as single:
   # Recursed two-action run takes original single-leg fallback, then fails.
   r=b.run(e,a)
  assert not r['ok'] and single.call_count==1
 finally:h.doCleanups()
