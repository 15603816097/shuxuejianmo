"""Single final Q3 evaluator for C1/C2/C3 fixed-case validation.

No feasibility booleans are accepted from callers.  Every gate is computed
from the authoritative timeline and official data.
"""
from __future__ import annotations
import json, math, sys
from pathlib import Path
import pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
from minimal_pipeline import load_inputs, node_map, line_geometry, terrain_profile
from q3_semantics_core import (build_authoritative_timeline, deadline_ledger,
    transport_energy_ledger, direct_and_relay_blocks, official_relay_energy,
    component_ledger, xyz, TOL)
from q3_five_service_hover_refinement import thresholds

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'

def _volume(data, ids):
    bm={x['货箱编号']:x for x in data['boxes']}; return sum(float(bm[i]['单箱体积（m³）']) for i in ids)

def _candidate_for_block(data, block):
    # Candidate generation uses the full 3-D phase endpoints and local XY offsets.
    pts=[(p['a'],p['ha']) for p in block['items']]+[(p['b'],p['hb']) for p in block['items']]
    lon=sum(float(p['经度（°）']) for p,h in pts)/len(pts); lat=sum(float(p['纬度（°）']) for p,h in pts)/len(pts)
    td,ta,tb=thresholds(data); c=data['center']; freq=float(data['comm'][('传播参数','载波频率（MHz）')]); obs=float(data['comm'][('传播参数','地形遮挡附加损耗（dB）')]); G=xyz(data,c,float(c['海拔（m）'])+float(data['comm'][('固定网关 G01','天线离地高度（m）')]))
    best=None
    for dx,dy in [(0,0),(.001,0),(-.001,0),(0,.001),(0,-.001),(.002,.002),(-.002,.002),(.002,-.002),(-.002,-.002)]:
        p={'经度（°）':lon+dx,'纬度（°）':lat+dy}; lons=data['dem']['longitude'].ravel(); lats=data['dem']['latitude'].ravel(); in_domain=(float(lons.min())<=p['经度（°）']<=float(lons.max()) and float(lats.min())<=p['纬度（°）']<=float(lats.max())); vals=[z for _,z in terrain_profile(p,p,data['dem']) if z==z]
        if not in_domain or not vals: continue
        if not vals: continue
        for off in (50.,100.,150.,200.,250.,300.):
            h=max(vals)+off; R=xyz(data,p,h); access=[]
            for q in block['items']:
                for ep,eh in ((q['a'],q['ha']),(q['b'],q['hb'])):
                    dist=((xyz(data,ep,eh)-R)**2).sum()**.5; access.append(ta-(32.45+20*math.log10(freq)+20*math.log10(max(dist/1000,1e-12))+obs))
            back=tb-(32.45+20*math.log10(freq)+20*math.log10(max(((R-G)**2).sum()**.5/1000,1e-12))+obs); z={'x':p['经度（°）'],'y':p['纬度（°）'],'altitude':h,'terrain_elevation_m':max(vals),'agl_m':off,'dem_query_ok':True,'access_margin':min(access),'backhaul_margin':back,'joint_margin':min(min(access),back),'candidate_source':'trajectory_3d_endpoints_plus_local_xy_offsets','dem_legal':bool(off>=0 and off<=data['relay']['max_height_m']+TOL),'height_legal':bool(off>=0 and off<=data['relay']['max_height_m']+TOL)}
            if best is None or z['joint_margin']>best['joint_margin']: best=z
    return best

def evaluate(data, route, box_set, drone_type, drone_id='AUTO', start_time=0.0, battery_id='AUTO', relay_plan=None):
    order=tuple(route); tl=build_authoritative_timeline(data,order,drone_type,box_set); dl=deadline_ledger(data,tl); en=transport_energy_ledger(data,tl); d=data['drones'][drone_type]
    mass=sum(float(x['单箱质量（kg）']) for x in data['boxes'] if x['货箱编号'] in box_set); volume=_volume(data,box_set); transport_energy=sum(float(x['energy_kwh']) for x in en); usable=(1-float(d.reserve))*float(d.energy); reserve_ok=transport_energy<=usable+TOL
    blocks=direct_and_relay_blocks(data,tl); candidates=[_candidate_for_block(data,b) for b in blocks]; geometry_ok=bool(all(x is not None and x['joint_margin']>=0 for x in candidates));
    relay_rows=[]; component_rows=[]; relay_ok='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'; component_ok='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'; relay_energy=float('nan'); relay_status='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'; relay_resource_status='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'
    if not blocks: relay_ok=True; component_ok=True; relay_energy=0.0; relay_status='NOT_REQUIRED'; relay_resource_status='NOT_REQUIRED'
    if geometry_ok and blocks:
        comp_available={f'C{i:02d}':0. for i in range(1,7)}; relay_ok=True; component_ok=True; relay_status='EVALUATED'; relay_resource_status='EVALUATED'
        for i,(b,z) in enumerate(zip(blocks,candidates)):
            rp={'经度（°）':z['x'],'纬度（°）':z['y']}; dist,peak,*_=line_geometry(data['center'],rp,data['dem']); tout,eout=__import__('q3_official_semantics',fromlist=['relay_leg_time_energy']).relay_leg_time_energy(data,dist,peak,float(data['center']['海拔（m）']),z['altitude']); tback,eback=__import__('q3_official_semantics',fromlist=['relay_leg_time_energy']).relay_leg_time_energy(data,dist,peak,z['altitude'],float(data['center']['海拔（m）'])); ps=b['start_s']-data['relay']['prep_s']-tout-data['relay']['link_s']; rend=b['end_s']+tback; energy=eout+eback+(data['relay']['hover_power_kw']+data['relay']['comm_power_kw'])*(b['end_s']-b['start_s'])/3600.; cid=min(comp_available,key=comp_available.get); comp=component_ledger(data,cid,ps,rend,energy,comp_available[cid]); comp_available[cid]=comp['available_again_s']; component_rows.append(comp); relay_rows.append({'block_id':i+1,'relay_id':'R01' if i%2==0 else 'R02','prepare_start':ps,'link_ready':b['start_s'],'service_start':b['start_s'],'service_end':b['end_s'],'return_end':rend,'joint_margin':z['joint_margin']}); relay_energy+=energy; relay_ok &= ps>=0 and comp['reserve_ok'] and comp['resource_available_ok']
        component_ok=all(x['reserve_ok'] and x['resource_available_ok'] for x in component_rows)
    hard_ok=all(x['pass'] for x in dl if x['hard']); transport_ok=(mass<=d.max_mass+TOL and volume<=d.volume+TOL and reserve_ok and hard_ok)
    relay_gate=bool(relay_ok is True); component_gate=bool(component_ok is True); status={'TRANSPORT_FEASIBLE':transport_ok,'COMMUNICATION_GEOMETRY_PASS':geometry_ok,'RELAY_CHAIN_PASS':relay_gate,'COMPONENT_PASS':component_gate,'ROUTE_FEASIBLE':bool(transport_ok and geometry_ok and relay_gate and component_gate)}
    detail={'timeline':tl,'deadline_ledger':dl,'transport_energy_ledger':en,'relay_required_blocks':blocks,'hover_candidates':candidates,'relay_timeline':relay_rows,'component_ledger':component_rows,'status':status,'relay_status':relay_status,'component_status':component_ok,'relay_resource_status':relay_resource_status,'mass_kg':mass,'volume_m3':volume,'transport_energy_kwh':transport_energy,'relay_energy_kwh':relay_energy,'remaining_energy_kwh':float(d.energy)-transport_energy,'required_reserve_kwh':usable,'soft_lateness_s':sum(max(0,x['delivery_time_s']-x['deadline_s']) for x in dl if not x['hard'])}
    return detail

def main():
    data=load_inputs(); cases=[('A_SINGLE_SIMPLE',('S001',),['S001-MED-01'],'B'),('B_S008_DIFFICULT',('S011','S008'),[x['货箱编号'] for x in data['boxes'] if x['服务区编号'] in ('S011','S008')],'C'),('C_THREE_POINT',('S005','S008','S009'),['S008-MED-01','S005-MED-01','S009-MED-01'],'B')]; summary=[]
    for name,route,ids,typ in cases:
        out=evaluate(data,route,ids,typ); (RES/f'q3_final_{name}_detail.json').write_text(json.dumps(out,ensure_ascii=False,default=str,indent=2),encoding='utf-8'); summary.append({'case':name,'route':'O01->'+'->'.join(route)+'->O01','box_count':len(ids),'drone_type':typ,'hard_deadline_ok':all(x['pass'] for x in out['deadline_ledger'] if x['hard']),'transport_energy_kwh':out['transport_energy_kwh'],'remaining_energy_kwh':out['remaining_energy_kwh'],'required_reserve_kwh':out['required_reserve_kwh'],'relay_blocks':len(out['relay_required_blocks']),'dem_legal':all(x.get('dem_legal',True) for x in out['hover_candidates']),'geometry_pass':out['status']['COMMUNICATION_GEOMETRY_PASS'],'relay_status':out['relay_status'],'relay_energy_kwh':out['relay_energy_kwh'],'relay_ok':out['status']['RELAY_CHAIN_PASS'],'component_status':out['component_status'],'component_ok':out['status']['COMPONENT_PASS'],'relay_resource_status':out['relay_resource_status'],'route_feasible':out['status']['ROUTE_FEASIBLE'],'soft_lateness_s':out['soft_lateness_s']})
    pd.DataFrame(summary).to_csv(RES/'q3_final_three_case_audit.csv',index=False,encoding='utf-8-sig'); print(pd.DataFrame(summary).to_string(index=False))
if __name__=='__main__': main()
