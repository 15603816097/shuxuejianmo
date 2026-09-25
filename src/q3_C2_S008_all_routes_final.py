from __future__ import annotations
import json, math, sys
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
from minimal_pipeline import load_inputs,node_map,line_geometry,terrain_profile,xy,energy_leg
from q3_official_semantics import relay_leg_time_energy,relay_component_ledger
from q3_five_service_hover_refinement import thresholds
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-6

def xyz(D,p,h):
    q=xy(float(p['经度（°）']),float(p['纬度（°）']),float(D['center']['纬度（°）'])); return np.array([q[0],q[1],float(h)])

def timeline(D,order,typ,ids):
    nm=node_map(D); d=D['drones'][typ]; bm={x['货箱编号']:x for x in D['boxes']}; boxes=[bm[i] for i in ids]; mass=sum(float(x['单箱质量（kg）']) for x in boxes); t=float(d.prep+d.load_box*len(boxes)); rows=[]; delivery={}; payload=mass; prev='O01'
    for nxt in (*order,'O01'):
        a,b=nm[prev],nm[nxt]; h0=float(a['海拔（m）'])+(30 if prev!='O01' else 0); h1=float(b['海拔（m）'])+(30 if nxt!='O01' else 0); dist,peak,*_=line_geometry(a,b,D['dem']); e,ft=energy_leg(d,dist,peak,h0,h1,payload); cruise=peak+50.; up=max(0.,cruise-h0)/d.climb_speed; down=max(0.,cruise-h1)/d.descend_speed
        if up>TOL: rows.append({'phase':'climb','start':t,'end':t+up,'a':a,'b':a,'ha':h0,'hb':cruise}); t+=up
        rows.append({'phase':'cruise','start':t,'end':t+dist/d.speed,'a':a,'b':b,'ha':cruise,'hb':cruise}); t+=dist/d.speed
        if down>TOL: rows.append({'phase':'descent','start':t,'end':t+down,'a':b,'b':b,'ha':cruise,'hb':h1}); t+=down
        if nxt!='O01':
            for bx in [x for x in boxes if x['服务区编号']==nxt]:
                hs=float(d.handoff_box); rows.append({'phase':'handoff','start':t,'end':t+hs,'a':b,'b':b,'ha':h1,'hb':h1}); t+=hs; delivery[bx['货箱编号']]=t; payload-=float(bx['单箱质量（kg）'])
        prev=nxt
    return rows,delivery,mass,t

def blocks(D,traj):
    td,ta,tb=thresholds(D); c=D['center']; freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); G=xyz(D,c,float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]))
    req=[]
    for p in traj:
        if p['phase']=='handoff': continue
        da=max(np.linalg.norm(xyz(D,p['a'],p['ha'])-G),np.linalg.norm(xyz(D,p['b'],p['hb'])-G)); m=td-(32.45+20*math.log10(freq)+20*math.log10(max(da/1000,1e-12))+obs); p=dict(p); p['direct_margin']=m
        if m<0: req.append(p)
    out=[]
    for p in req:
        if out and p['phase']==out[-1]['phase'] and abs(p['start']-out[-1]['end'])<TOL: out[-1]['items'].append(p); out[-1]['end']=p['end']
        else: out.append({'start':p['start'],'end':p['end'],'phase':p['phase'],'items':[p]})
    return out,G,ta,tb,freq,obs

def block_candidates(D,b,G,ta,tb,freq,obs):
    c=D['center']; points=[]
    for p in b['items']:
        points.extend([(p['a'],p['ha']),(p['b'],p['hb'])])
    lon=np.mean([float(q['经度（°）']) for q,h in points]); lat=np.mean([float(q['纬度（°）']) for q,h in points]); offs=[(0,0),(.001,0),(-.001,0),(0,.001),(0,-.001),(.002,.002),(-.002,.002),(.002,-.002),(-.002,-.002)]
    best=None
    for dx,dy in offs:
        rp={'经度（°）':lon+dx,'纬度（°）':lat+dy}; vals=[z for _,z in terrain_profile(rp,rp,D['dem']) if np.isfinite(z)]
        if not vals: continue
        for off in (50.,100.,150.,200.,250.,300.):
            rh=max(vals)+off; R=xyz(D,rp,rh); ac=[]
            for p in b['items']:
                for q,h in ((p['a'],p['ha']),(p['b'],p['hb'])): ac.append(ta-(32.45+20*math.log10(freq)+20*math.log10(max(np.linalg.norm(xyz(D,q,h)-R)/1000,1e-12))+obs))
            back=tb-(32.45+20*math.log10(freq)+20*math.log10(max(np.linalg.norm(R-G)/1000,1e-12))+obs); jm=min(min(ac),back); z={'joint':jm,'access':min(ac),'backhaul':back,'x':rp['经度（°）'],'y':rp['纬度（°）'],'h':rh,'source':'trajectory_3d_endpoints_plus_local_xy_offsets'}
            if best is None or z['joint']>best['joint']: best=z
    return best

def main():
    D=load_inputs(); base=pd.read_csv(RES/'q3_C2_legal_group_candidates_frozen.csv'); rows=[]; counts={'total':0,'transport':0,'geometry':0,'chain':0,'final':0}
    for _,r in base.iterrows():
        counts['total']+=1; ids=json.loads(r['box_set']); order=tuple(str(r['visit_order']).split('->')); typ=str(r['drone_type']); traj,delivery,mass,makespan=timeline(D,order,typ,ids); d=D['drones'][typ]; payload=mass<=d.max_mass+TOL and sum(float(next(x for x in D['boxes'] if x['货箱编号']==i)['单箱体积（m³）']) for i in ids)<=d.volume+TOL; energy=0.0
        # exact two-leg energy with payload updates
        nm=node_map(D); rem=mass; energy=0.; prev='O01'
        for nxt in (*order,'O01'):
            a,b=nm[prev],nm[nxt]; dist,peak,*_=line_geometry(a,b,D['dem']); h0=float(a['海拔（m）'])+(30 if prev!='O01' else 0); h1=float(b['海拔（m）'])+(30 if nxt!='O01' else 0); e,_=energy_leg(d,dist,peak,h0,h1,rem); energy+=e
            if nxt!='O01': rem-=sum(float(x['单箱质量（kg）']) for x in D['boxes'] if x['货箱编号'] in ids and x['服务区编号']==nxt)
            prev=nxt
        reserve=energy<=(1-d.reserve)*d.energy+TOL; hard=True; bm={x['货箱编号']:x for x in D['boxes']}
        for i in ids:
            b= bm[i]; arr=delivery[i]
            if b['物资类型']=='医疗物资': hard &= arr<=float(b['期望送达时间（s）'])+TOL
            if b['是否首批保障']=='是': hard &= arr<=float(b['首批截止时间（s）'])+TOL
        base_ok=payload and reserve and hard; counts['transport']+=int(base_ok)
        bs,G,ta,tb,freq,obs=blocks(D,traj); bests=[block_candidates(D,b,G,ta,tb,freq,obs) for b in bs]; feas=[x for x in bests if x and x['joint']>0]; geo=bool(len(feas)==len(bs)); counts['geometry']+=int(geo)
        chain_ok=False; comp_ok=False; relay_energy=0.; relay_peak=0
        if geo:
            free={'R01':0.,'R02':0.}; compfree={f'C{i:02d}':0. for i in range(1,7)}; chain_ok=True
            for b in bests:
                dist,peak,*_=line_geometry(D['center'],{'经度（°）':b['x'],'纬度（°）':b['y']},D['dem']); tout,eout=relay_leg_time_energy(D,dist,peak,float(D['center']['海拔（m）']),b['h']); tback,eback=relay_leg_time_energy(D,dist,peak,b['h'],float(D['center']['海拔（m）'])); rid=min(free,key=free.get); ps=b['start']-float(D['relay']['prep_s'])-tout-float(D['relay']['link_s']); rend=b['end']+tback; busy=rend+float(D['relay']['turn_s']); cid=min(compfree,key=compfree.get); en=eout+eback+(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(b['end']-b['start'])/3600.; led=relay_component_ledger(D,cid,ps,rend,en,compfree[cid]); chain_ok &= ps>=free[rid]-TOL and led['resource_available_ok'] and led['reserve_ok']; free[rid]=busy; compfree[cid]=led['available_again_s']; relay_energy+=en
            relay_peak=2; comp_ok=chain_ok
        counts['chain'] += int(geo and chain_ok); final=base_ok and geo and chain_ok and comp_ok; counts['final']+=int(final)
        rows.append({'route':f"O01->{'->'.join(order)}->O01",'visit_order':'->'.join(order),'box_set':json.dumps(ids,ensure_ascii=False),'drone_type':typ,'hard_deadline_ok':hard,'payload_ok':payload,'volume_ok':sum(float(bm[i]['单箱体积（m³）']) for i in ids)<=d.volume+TOL,'transport_reserve_ok':reserve,'relay_block_count':len(bs),'feasible_block_count':len(feas),'worst_block_joint_margin':min([x['joint'] for x in bests],default=float('inf')),'communication_geometry_pass':geo,'relay_chain_ok':chain_ok,'component_ok':comp_ok,'route_feasible':final,'transport_energy':energy,'relay_energy':relay_energy,'total_energy':energy+relay_energy,'makespan':makespan})
    out=pd.DataFrame(rows).sort_values(['route_feasible','worst_block_joint_margin','total_energy','makespan'],ascending=[False,False,True,True]); out.to_csv(RES/'q3_C2_S008_all_routes_screening_final.csv',index=False,encoding='utf-8-sig'); print(json.dumps(counts,ensure_ascii=False)); print(out.head(3).to_string(index=False))
if __name__=='__main__': main()
