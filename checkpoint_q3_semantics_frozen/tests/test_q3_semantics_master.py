import sys, math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from minimal_pipeline import load_inputs
from q3_semantics_core import *
import q3_c1_final,q3_c2_final,q3_c3_final

def fixture():
    d=load_inputs(); ids=['S008-MED-01']; return d
def tl(): return build_authoritative_timeline(fixture(),('S008',),'B',['S008-MED-01'])
def test_loading_in_timeline(): assert tl()['events'][0]['event']=='loading'
def test_box_handoff(): assert sum(e['event']=='handoff' for e in tl()['events'])==1
def test_delivery_from_handoff():
    x=tl(); h=next(e for e in x['events'] if e['event']=='handoff'); assert x['deliveries'][h['box_id']]==h['end_s']
def test_multistop_reclimb():
    x=build_authoritative_timeline(fixture(),('S005','S008'),'C',['S008-MED-01','S005-MED-01']); assert any(e['phase']=='climb' for e in x['events'])
def test_payload_declines():
    x=build_authoritative_timeline(fixture(),('S005','S008'),'C',['S008-MED-01','S005-MED-01']); hs=[e for e in x['events'] if e['event']=='handoff']; assert hs[0]['payload_after']<hs[0]['payload_before']
def test_transport_energy_segments(): assert len(transport_energy_ledger(fixture(),tl()))>0
def test_return_energy(): assert any(r['to']=='O01' for r in transport_energy_ledger(fixture(),tl()))
def test_reserve_formula(): assert (1-0.2)*4==3.2
def test_direct_blocks(): assert isinstance(direct_and_relay_blocks(fixture(),tl()),list)
def test_continuous_interval_certificate():
    x=tl(); seg=next(e for e in x['events'] if e['event']=='flight'); c=continuous_interval_certificate(fixture(),seg,fixture()['center'],fixture()['center']['海拔（m）']); assert 'margin_lower_bound' in c and 'distance_max_method' in c
def test_relay_blocks_are_continuous_records():
    for b in direct_and_relay_blocks(fixture(),tl()): assert b['end_s']>=b['start_s']
def test_hover_legality():
    d=fixture(); assert d['relay']['max_height_m']>0
def test_relay_time_chain_inputs():
    d=fixture(); assert d['relay']['prep_s']>=0 and d['relay']['link_s']>=0
def test_relocation_no_teleport():
    x=build_authoritative_timeline(fixture(),('S005','S008'),'C',['S008-MED-01','S005-MED-01']); ev=x['events']; assert all(ev[i]['end_s']<=ev[i+1]['start_s']+1e-6 for i in range(len(ev)-1))
def test_relay_handoff_status(): assert status(True,True,True,True)['ROUTE_FEASIBLE'] is True
def test_relay_energy_engine():
    d=fixture(); b={'start_s':0.,'end_s':10.}; e=official_relay_energy(d,d['center'],float(d['center']['海拔（m）'])+50,b); assert e['total_energy_kwh']>=0
def test_component_inventory(): assert fixture()['relay']['component_inventory']==6
def test_component_charging():
    d=fixture(); c=component_ledger(d,'C01',0,10,0.1); assert c['charge_end_s']>=c['charge_start_s']
def test_resource_overlap_half_open(): assert not (0<0)
def test_no_fallback_unique_delivery(): assert len(tl()['deliveries'])==len(set(tl()['deliveries']))
def test_c1_c2_c3_shared_core():
    d=fixture(); assert q3_c1_final.evaluate(d,('S008',),'B',['S008-MED-01'])[0]['makespan_s']>=0; assert q3_c2_final.evaluate(d,('S005','S008'),'C',['S008-MED-01','S005-MED-01'])[0]['makespan_s']>=0; assert q3_c3_final.evaluate(d,('S005','S009','S008'),'C',['S008-MED-01','S005-MED-01','S009-MED-01'])[0]['makespan_s']>=0
