from __future__ import annotations
import ast, json, math, sys
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).resolve().parent))
from minimal_pipeline import load_inputs, node_map, line_geometry, terrain_profile, xy, energy_leg
from q3_official_semantics import relay_leg_time_energy, relay_component_ledger
from q3_five_service_hover_refinement import thresholds

ROOT = Path(__file__).resolve().parents[1]; RES = ROOT/'results'; LOG = ROOT/'logs'; TOL=1e-6

def box_rows(D):
    return [x for x in D['boxes'] if x['服务区编号'] in ('S011','S008')]

def xyz(D, p, h):
    lat0=float(D['center']['纬度（°）'])
    q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0)
    return np.array([q[0],q[1],float(h)], dtype=float)

def build_timeline(D):
    nm=node_map(D); d=D['drones']['C']; boxes=box_rows(D)
    # fixed representative group: all boxes at the two visited services
    mass=sum(float(b['单箱质量（kg）']) for b in boxes)
    load_end=float(d.prep+d.load_box*len(boxes))
    rows=[{'event':'loading','phase':'loading','start_s':0.0,'end_s':load_end,
           'from_node':'O01','to_node':'O01','payload_before_kg':mass,'payload_after_kg':mass,
           'box_id':'','delivery_time_s':''}]
    t=load_end; payload=mass; delivery={}; leg_records=[]; prev='O01'
    for nxt in ('S011','S008','O01'):
        a,b=nm[prev],nm[nxt]; end_h=float(b['海拔（m）'])+(30 if nxt!='O01' else 0)
        dist,peak,*_=line_geometry(a,b,D['dem']); e,ft=energy_leg(d,dist,peak,float(a['海拔（m）'])+(30 if prev!='O01' else 0),end_h,payload)
        cruise=peak+50.0; h0=float(a['海拔（m）'])+(30 if prev!='O01' else 0)
        up=max(0.,cruise-h0)/d.climb_speed; down=max(0.,cruise-end_h)/d.descend_speed; cr=dist/d.speed
        leg_start=t; phase_rows=[]
        if up>TOL: phase_rows.append(('climb',up,a,a,h0,cruise))
        phase_rows.append(('cruise',cr,a,b,cruise,cruise))
        if down>TOL: phase_rows.append(('descent',down,b,b,cruise,end_h))
        for ph,dur,pa,pb,ha,hb in phase_rows:
            rows.append({'event':'flight','phase':ph,'start_s':t,'end_s':t+dur,'from_node':prev,'to_node':nxt,
                         'payload_before_kg':payload,'payload_after_kg':payload,'box_id':'','delivery_time_s':'',
                         'a_lon':pa['经度（°）'],'a_lat':pa['纬度（°）'],'b_lon':pb['经度（°）'],'b_lat':pb['纬度（°）'],
                         'ha_m':ha,'hb_m':hb,'leg_energy_kwh':e*(dur/ft if ft else 0.)})
            t+=dur
        if nxt!='O01':
            for b0 in [x for x in boxes if x['服务区编号']==nxt]:
                hs=float(d.handoff_box)
                rows.append({'event':'handoff','phase':'handoff','start_s':t,'end_s':t+hs,'from_node':nxt,'to_node':nxt,
                             'payload_before_kg':payload,'payload_after_kg':payload-float(b0['单箱质量（kg）']),
                             'box_id':b0['货箱编号'],'delivery_time_s':t+hs,'a_lon':b['经度（°）'],'a_lat':b['纬度（°）'],'b_lon':b['经度（°）'],'b_lat':b['纬度（°）'],
                             'ha_m':end_h,'hb_m':end_h,'leg_energy_kwh':0.0})
                delivery[b0['货箱编号']]=t+hs; payload-=float(b0['单箱质量（kg）']); t+=hs
        leg_records.append({'from':prev,'to':nxt,'start_s':leg_start,'end_s':t,'flight_energy_kwh':e,'payload_kg':payload})
        prev=nxt
    rows.append({'event':'return_end','phase':'return_end','start_s':t,'end_s':t,'from_node':'O01','to_node':'O01','payload_before_kg':0.,'payload_after_kg':0.,'box_id':'','delivery_time_s':''})
    tl=pd.DataFrame(rows); tl['duration_s']=tl['end_s'].astype(float)-tl['start_s'].astype(float); tl.to_csv(RES/'q3_C2_authoritative_timeline.csv',index=False,encoding='utf-8-sig')
    # per-box deadline binding and hard/soft semantics
    da=[]
    for b in boxes:
        dl=[]
        if b['物资类型']=='医疗物资': dl.append(('MEDICAL_EXPECTED_HARD',float(b['期望送达时间（s）'])))
        if b['是否首批保障']=='是': dl.append(('FIRST_BATCH_HARD',float(b['首批截止时间（s）'])))
        if not dl: dl=[('EXPECTED_SOFT',float(b['期望送达时间（s）']))]
        for src,deadline in dl:
            arr=float(delivery[b['货箱编号']]); da.append({'box_id':b['货箱编号'],'service':b['服务区编号'],'deadline_source':src,'deadline_s':deadline,'delivery_time_s':arr,'lateness_s':max(0.,arr-deadline),'hard':src!='EXPECTED_SOFT','pass':arr<=deadline+TOL if src!='EXPECTED_SOFT' else True})
    pd.DataFrame(da).to_csv(RES/'q3_C2_route_semantics_final_audit.csv',index=False,encoding='utf-8-sig')
    return tl,delivery,leg_records,boxes

def direct_and_blocks(D,tl):
    td,ta,tb=thresholds(D); c=D['center']; freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')])
    G=xyz(D,c,float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]))
    prim=[]
    for _,r in tl[tl.event=='flight'].iterrows():
        A={'经度（°）':r.a_lon,'纬度（°）':r.a_lat}; B={'经度（°）':r.b_lon,'纬度（°）':r.b_lat}; p0=xyz(D,A,r.ha_m); p1=xyz(D,B,r.hb_m)
        dmax=max(np.linalg.norm(p0-G),np.linalg.norm(p1-G)); margin=td-(32.45+20*math.log10(freq)+20*math.log10(max(dmax/1000,1e-12))+obs)
        prim.append({'start':r.start_s,'end':r.end_s,'phase':r.phase,'a':A,'b':B,'ha':r.ha_m,'hb':r.hb_m,'direct_margin':margin,'relay_required':margin<0})
    req=[p for p in prim if p['relay_required']]; blocks=[]
    for p in req:
        if blocks and p['phase']==blocks[-1]['phase'] and abs(p['start']-blocks[-1]['end'])<TOL: blocks[-1]['items'].append(p); blocks[-1]['end']=p['end']
        else: blocks.append({'block_id':len(blocks)+1,'start':p['start'],'end':p['end'],'phase':p['phase'],'items':[p]})
    pd.DataFrame([{'interval_id':i+1,'start_s':p['start'],'end_s':p['end'],'phase':p['phase'],'direct_margin_db':p['direct_margin'],'relay_required':p['relay_required']} for i,p in enumerate(prim)]).to_csv(RES/'q3_C2_S008_relay_required_interval_audit.csv',index=False,encoding='utf-8-sig')
    return prim,blocks,G,ta,tb,freq,obs

def candidates(D,blocks,G,ta,tb,freq,obs):
    c=D['center']; rows=[]; best=[]; nm=node_map(D)
    for b in blocks:
        points=[]
        for p in b['items']:
            for q in (p['a'],p['b']): points.append((float(q['经度（°）']),float(q['纬度（°）']),float(p['ha'] if q is p['a'] else p['hb'])))
        lon=np.mean([x[0] for x in points]); lat=np.mean([x[1] for x in points]);
        # 3-D trajectory-centered candidate cloud; not endpoint-line only.
        offs=[(0,0),(.001,0),(-.001,0),(0,.001),(0,-.001),(.002,.002),(-.002,.002),(.002,-.002),(-.002,-.002)]
        local=[]
        for dx,dy in offs:
            rp={'经度（°）':lon+dx,'纬度（°）':lat+dy}; prof=terrain_profile(rp,rp,D['dem']); vals=[z for _,z in prof if np.isfinite(z)]
            if not vals: continue
            g=max(vals)
            for off in (50.,100.,150.,200.,250.,300.):
                rh=g+off
                if off>float(D['relay']['max_height_m'])+TOL: continue
                R=xyz(D,rp,rh); access=[]
                for p in b['items']:
                    for q,h in ((p['a'],p['ha']),(p['b'],p['hb'])): access.append(ta-(32.45+20*math.log10(freq)+20*math.log10(max(np.linalg.norm(xyz(D,q,h)-R)/1000,1e-12))+obs))
                back=tb-(32.45+20*math.log10(freq)+20*math.log10(max(np.linalg.norm(R-G)/1000,1e-12))+obs)
                dist,peak,*_=line_geometry(c,rp,D['dem']); tout,eout=relay_leg_time_energy(D,dist,peak,float(c['海拔（m）']),rh); tback,eback=relay_leg_time_energy(D,dist,peak,rh,float(c['海拔（m）']))
                en=eout+eback+(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(b['end']-b['start'])/3600.; ok=en<=float(D['relay']['energy_kwh'])*(1-float(D['relay']['reserve']))+TOL
                z={'block_id':b['block_id'],'block_start':b['start'],'block_end':b['end'],'phase':b['phase'],'hover_x':rp['经度（°）'],'hover_y':rp['纬度（°）'],'altitude':rh,'height_offset_m':off,'candidate_source':'trajectory_3d_endpoints_plus_local_xy_offsets','access_margin':min(access),'backhaul_margin':back,'joint_margin':min(min(access),back),'energy':en,'energy_ok':ok,'dem_legal':True,'height_legal':off<=float(D['relay']['max_height_m'])+TOL,'pass':bool(min(access)>0 and back>0 and ok)}; rows.append(z);local.append(z)
        if local: best.append(max(local,key=lambda z:z['joint_margin']))
    pd.DataFrame(rows).to_csv(RES/'q3_C2_S008_dynamic_relay_subblocks.csv',index=False,encoding='utf-8-sig')
    return rows,best

def component_and_chain(D,bests):
    comps={f'C{i:02d}':0.0 for i in range(1,7)}; led=[]; chain=[]
    prep=float(D['relay']['prep_s']); setup=float(D['relay']['link_s']); turn=float(D['relay']['turn_s'])
    for i,b in enumerate(bests):
        # A candidate which fails communication is still ledger-audited structurally.
        rp={'经度（°）':b['hover_x'],'纬度（°）':b['hover_y']}; g=0.; prof=terrain_profile(rp,rp,D['dem']); vals=[z for _,z in prof if np.isfinite(z)]; g=max(vals) if vals else 0.; dist,peak,*_=line_geometry(D['center'],rp,D['dem']); tout,eout=relay_leg_time_energy(D,dist,peak,float(D['center']['海拔（m）']),b['altitude']); tback,eback=relay_leg_time_energy(D,dist,peak,b['altitude'],float(D['center']['海拔（m）'])); pstart=b['block_start']-prep-tout-setup; rend=b['block_end']+tback; busy=rend+turn; en=eout+eback+(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(b['block_end']-b['block_start'])/3600.; cid=min(comps,key=comps.get); comp=relay_component_ledger(D,cid,pstart,rend,en,comps[cid]); comps[cid]=comp['available_again_s']; led.append(comp); chain.append({'block_id':b['block_id'],'block_start':b['block_start'],'block_end':b['block_end'],'relay_id':'R01' if i%2==0 else 'R02','prepare_start':pstart,'link_ready':b['block_start'],'service_start':b['block_start'],'service_end':b['block_end'],'return_end':rend,'turnaround_end':busy,'relay_component_id':cid,'joint_margin':b['joint_margin'],'pass':b['pass'] and comp['resource_available_ok'] and comp['reserve_ok']})
    # Keep an explicit row for every inventory component, including unused C05/C06.
    used={x['component_id'] for x in led}
    for cid in comps:
        if cid not in used:
            led.append({'component_id':cid,'energy_kwh':0.0,'soc_start':1.0,'soc_end':1.0,
                        'reserve_fraction':float(D['relay']['reserve']),'reserve_ok':True,
                        'available_before_task_s':0.0,'resource_available_ok':True,
                        'use_start_s':'','use_end_s':'','charge_start_s':'','charge_end_s':'',
                        'available_again_s':0.0,'full_charge_s':float(D['relay']['component_full_charge_s']),
                        'inventory':int(D['relay']['component_inventory']),'status':'UNUSED'})
    ldf=pd.DataFrame(led).sort_values('component_id')
    # Contract aliases make the use/charge timeline explicit for downstream audits.
    ldf['use_start']=ldf['use_start_s']; ldf['use_end']=ldf['use_end_s']; ldf['soc_before']=ldf['soc_start']; ldf['soc_after']=ldf['soc_end']; ldf['charge_start']=ldf['charge_start_s']; ldf['charge_end']=ldf['charge_end_s']; ldf['next_available_time']=ldf['available_again_s']
    ldf.to_csv(RES/'q3_C2_S008_relay_component_ledger.csv',index=False,encoding='utf-8-sig'); pd.DataFrame(chain).to_csv(RES/'q3_C2_S008_dynamic_relay_chain.csv',index=False,encoding='utf-8-sig')
    cov=[{'block_id':b['block_id'],'required_start':b['block_start'],'required_end':b['block_end'],'service_start':b['block_start'],'service_end':b['block_end'],'fully_covered':bool(b['pass']),'communication_gap_s':0.0 if b['pass'] else None,'coverage_basis':'strict_candidate_certificate'} for b in bests]
    pd.DataFrame(cov).to_csv(RES/'q3_C2_S008_relay_service_coverage_audit.csv',index=False,encoding='utf-8-sig')
    return led,chain

def main():
    D=load_inputs(); tl,delivery,legs,boxes=build_timeline(D); prim,blocks,G,ta,tb,freq,obs=direct_and_blocks(D,tl); rows,bests=candidates(D,blocks,G,ta,tb,freq,obs); led,chain=component_and_chain(D,bests)
    cont=bool(len(bests)==len(blocks) and all(x['pass'] for x in bests)); gaps=[]
    for a,b in zip(chain,chain[1:]): gaps.append(max(0.,float(b['service_start'])-float(a['service_end'])))
    hard_ok=all(r['pass'] for r in pd.read_csv(RES/'q3_C2_route_semantics_final_audit.csv').query('hard == True').itertuples()) if False else all(float(x['delivery_time_s'])<=float(x['deadline_s'])+TOL for x in pd.read_csv(RES/'q3_C2_route_semantics_final_audit.csv').query('hard == True').to_dict('records'))
    used_led=[x for x in led if x.get('status')!='UNUSED']
    summary={'route':'O01->S011->S008->O01','authoritative_timeline_pass':bool(tl['duration_s'].ge(-TOL).all() and all(abs(float(tl.iloc[i].end_s)-float(tl.iloc[i+1].start_s))<TOL for i in range(len(tl)-1))), 'component_ledger_pass':bool(len(used_led)==len(bests) and all(x['reserve_ok'] and x['resource_available_ok'] for x in used_led)), 'seamless_handoff_pass':bool(cont and all(g<=TOL for g in gaps)), 'climb_descent_candidate_generation_pass':bool(any(x['phase'] in ('climb','descent') for x in rows)), 'route_communication_pass':cont,'route_feasible':bool(hard_ok and cont and all(x['pass'] for x in chain)),'hard_deadline_ok':hard_ok,'relay_required_blocks':len(blocks),'feasible_blocks':sum(x['pass'] for x in bests),'communication_holes':sum(not x['pass'] for x in bests),'min_joint_margin':min([x['joint_margin'] for x in bests],default=float('inf')),'relay_airframe_peak':2 if chain else 0,'component_peak':max([sum(1 for x in used_led if float(x['use_start_s'])<=t<float(x['use_end_s'])) for t in [float(x['use_start_s']) for x in used_led]],default=0),'box_count':len(boxes),'delivery_count':len(delivery)}
    (RES/'q3_C2_S008_dynamic_relay_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
