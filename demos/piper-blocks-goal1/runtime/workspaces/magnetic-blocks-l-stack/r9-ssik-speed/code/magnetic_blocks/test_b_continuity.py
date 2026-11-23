import copy
from unittest.mock import patch
import pytest
import assembly_batch as ab
from assembly_plan import NoPlan
from test_feedback_repair import BatchBinding


def test_lift_transfer_blends_but_keeps_contact_and_jaw_boundaries():
    def arm(stage, **kw):
        return dict(kind='arm',color='orange',stage='orange-'+stage,payload={'color':'orange'},**kw)
    actions=[dict(kind='jaw',stage='orange-close'),arm('raise',pickup=True),arm('transfer'),arm('transfer'),arm('lower',contact=True),dict(kind='jaw',stage='orange-release'),arm('retreat',withdraw=True)]
    assert [len(g) for g in ab.groups(actions)]==[1,3,1,1,1]
    assert len(ab.groups([arm('transfer'),arm('raise',pickup=True)]))==2
    other=arm('transfer');other['color']='green'
    assert len(ab.groups([arm('raise',pickup=True),other]))==2


def test_phase_checks_receive_pickup_only_for_departure():
    e,actions,start=BatchBinding().make(.0511,.05,{'color':'green'})
    actions[0].update(stage='green-raise',pickup=True)
    seen=[]
    e.planner._check=lambda *args,**kw:seen.append(kw) or {}
    with patch('assembly_batch.boxes',return_value=[{'synthetic':True}]),patch('payload_replan.bind_pickup',return_value=(actions,{'synthetic':True})):
        result=ab.preview(e,actions,start)
    assert [v['pickup'] for v in seen]==[True,False]
    assert result['stages']==['green-raise','green-transfer']
    assert all(v['reserve_m']==.003 for v in seen)
    assert all(len(v['curve14'])==3 for v in seen)


@pytest.mark.parametrize('bounds',[[],[{'sample_end':0},{'sample_end':2}],[{'sample_end':3},{'sample_end':4}]])
def test_missing_or_invalid_progress_rejected_before_dispatch(bounds):
    e,actions,start=BatchBinding().make(.0511,.05,{'color':'green'})
    original=e.ctx.value
    def value(*args,**kw):
        row=original(*args,**kw);row['waypoints']=bounds;return row
    e.ctx.value=value
    with patch('assembly_batch.boxes',return_value=[{'synthetic':True}]),pytest.raises(NoPlan):
        ab.preview(e,actions,start)
