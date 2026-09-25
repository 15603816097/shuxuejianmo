from __future__ import annotations
import json, math, itertools, sys
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
from minimal_pipeline import load_inputs,node_map,line_geometry,terrain_profile,xy,energy_leg
from q3_official_semantics import relay_leg_time_energy, relay_component_ledger
from q3_five_service_hover_refinement import thresholds
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-6

def xyz(D,p,h):
 q=xy(float(p['经度（°）']),float(p['纬度（°）']),float(D['center']['纬度（°）'])); return np.array([q[0],q[1],float(h)])

def make_timeline(D,order,typ,ids):
 nm=node_map(D); d=D['drones'][typ]; bm={x['货箱编号']:x for x in D['boxes']}; boxes=[bm[i] for i in ids]; payload=sum(float(x['单箱质量（kg）']) for x in boxes); t=float(d.prep+d.load_box*len(boxes)); delivery={}; traj=[]; prev='O01'
 for nxt in (*order,'O01'):
  a,b=nm[prev],nm[nxt]; h0=float(a['海拔（m）'])+(30 if prev!='O01' else 0); h1=float(b['海拔（m）'])+(30 if nxt!='O01' else 0); dist,peak,*_=line_geometry(a,b,D['dem']); _,ft=energy_leg(d,dist,peak,h0,h1,payload); cruise=peak+50.; up=max(0.,cruise-h0)/d.climb_speed; down=max(0.,cruise-h1)/d.descend_speed
  if up>TOL: traj.append({'phase':'climb','start':t,'end':t+up,'a':a,'b':a,'ha':h0,'hb':cruise}); t+=up
  traj.append({'phase':'cruise','start':t,'end':t+dist/d.speed,'a':a,'b':b,'ha':cruise,'hb':cruise}); t+=dist/d.speed
  if down>TOL: traj.append({'phase':'descent','start':t,'end':t+down,'a':b,'b':b,'ha':cruise,'hb':h1}); t+=down
  if nxt!='O01':
   for bx in [x for x in boxes if x['服务区编号']==nxt]:
    hs=float(d.handoff_box); traj.append({'phase':'handoff','start':t,'end':t+hs,'a':b,'b':b,'ha':h1,'hb':h1}); t+=hs; delivery[bx['货箱编号']]=t; payload-=float(bx['单箱质量（kg）'])
  prev=nxt
 return traj,delivery,t

def get_blocks(D,traj):
 td,ta,tb=thresholds(D); c=D['center']; freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); G=xyz(D,c,float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]))
 req=[]
 for p in traj:
  if p['phase']=='handoff': continue
  dm=td-(32.45+20*math.log10(freq)+20*math.log10(max(max(np.linalg.norm(xyz(D,p['a'],p['ha'])-G),np.linalg.norm(xyz(D,p['b'],p['hb'])-G))/1000,1e-12))+obs)
  if dm<0: req.append(p)
 blocks=[]
 for p in req:
  if blocks and p['phase']==blocks[-1]['phase'] and abs(p['start']-blocks[-1]['end'])<TOL: blocks[-1]['items'].append(p); blocks[-1]['end']=p['end']
  else: blocks.append({'start':p['start'],'end':p['end'],'phase':p['phase'],'items':[p]})
 return blocks,G,ta,tb,freq,obs

def best_hover(D,b,G,ta,tb,freq,obs):
 pts=[(p['a'],p['ha']) for p in b['items']]+[(p['b'],p['hb']) for p in b['items']]; lon=np.mean([float(x['经度（°）']) for x,h in pts]); lat=np.mean([float(x['纬度（°）']) for x,h in pts]); best=None
 for dx,dy in [(0,0),(.001,0),(-.001,0),(0,.001),(0,-.001),(.002,.002),(-.002,.002),(.002,-.002),(-.002,-.002)]:
  rp={'经度（°）':lon+dx,'纬度（°）':lat+dy}; vals=[z for _,z in terrain_profile(rp,rp,D['dem']) if np.isfinite(z)]
  if not vals: continue
  for off in (50.,100.,150.,200.,250.,300.):
   rh=max(vals)+off; R=xyz(D,rp,rh); am=[]
   for p in b['items']:
    for q,h in ((p['a'],p['ha']),(p['b'],p['hb'])): am.append(ta-(32.45+20*math.log10(freq)+20*math.log10(max(np.linalg.norm(xyz(D,q,h)-R)/1000,1e-12))+float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')])))
   bm=tb-(32.45+20*math.log10(freq)+20*math.log10(max(np.linalg.norm(R-G)/1000,1e-12))+float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]))
   z={'joint':min(min(am),bm),'access':min(am),'backhaul':bm,'x':rp['经度（°）'],'y':rp['纬度（°）'],'h':rh}
   if best is None or z['joint']>best['joint']: best=z
 return best

def main():
 D=load_inputs(); boxes=D['boxes']; by={s:[x for x in boxes if x['服务区编号']==s] for s in sorted({x['服务区编号'] for x in boxes})}; partners=['S005','S007','S009','S010','S013','S014','S015']; s8=by['S008']; rows=[]
 # urgent-first sets: S008 medical, then one/two earliest boxes at each partner; no Cartesian explosion
 for a,b in itertools.combinations(partners,2):
  for order in [(a,b,'S008'),(a,'S008',b),('S008',a,b)]:
   sets=[ [next(x for x in s8 if x['物资类型']=='医疗物资')], [next(x for x in s8 if x['物资类型']=='医疗物资'), next(x for x in s8 if x['货箱编号']=='S008-WAT-01')] ]
   for typ in ('B','C'):
    for extra in sets:
     chosen=list(extra)+[by[svc][0] for svc in order if svc!='S008']; ids=[x['货箱编号'] for x in chosen]; d=D['drones'][typ]; mass=sum(float(x['单箱质量（kg）']) for x in chosen); vol=sum(float(x['单箱体积（m³）']) for x in chosen); payload=mass<=d.max_mass+TOL and vol<=d.volume+TOL
     traj,delivery,makespan=make_timeline(D,order,typ,ids); rem=mass; energy=0.; nm=node_map(D); prev='O01'
     for nxt in (*order,'O01'):
      aa,bb=nm[prev],nm[nxt]; dist,peak,*_=line_geometry(aa,bb,D['dem']); h0=float(aa['海拔（m）'])+(30 if prev!='O01' else 0); h1=float(bb['海拔（m）'])+(30 if nxt!='O01' else 0); e,_=energy_leg(d,dist,peak,h0,h1,rem); energy+=e
      if nxt!='O01': rem-=sum(float(x['单箱质量（kg）']) for x in chosen if x['服务区编号']==nxt)
      prev=nxt
     reserve=energy<=(1-d.reserve)*d.energy+TOL; hard=True
     for x in chosen:
      if x['物资类型']=='医疗物资': hard &= delivery[x['货箱编号']]<=float(x['期望送达时间（s）'])+TOL
      if x['是否首批保障']=='是': hard &= delivery[x['货箱编号']]<=float(x['首批截止时间（s）'])+TOL
     base=payload and reserve and hard; bs,G,ta,tb,freq,obs=get_blocks(D,traj); best=[best_hover(D,z,G,ta,tb,freq,obs) for z in bs]; feas=[z for z in best if z and z['joint']>0]; geo=bool(bs and len(feas)==len(bs)); relay_energy=0.0
     for z,bk in zip(best,bs):
      if z is None: continue
      dist,peak,*_=line_geometry(D['center'],{'经度（°）':z['x'],'纬度（°）':z['y']},D['dem']); tout,eout=relay_leg_time_energy(D,dist,peak,float(D['center']['海拔（m）']),z['h']); tback,eback=relay_leg_time_energy(D,dist,peak,z['h'],float(D['center']['海拔（m）'])); relay_energy += eout+eback+(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(bk['end']-bk['start'])/3600.0
     rows.append({'route':'O01->'+'->'.join(order)+'->O01','visit_order':'->'.join(order),'box_set':json.dumps(ids,ensure_ascii=False),'drone_type':typ,'hard_deadline_ok':hard,'payload_ok':mass<=d.max_mass+TOL,'reserve_ok':reserve,'relay_block_count':len(bs),'feasible_block_count':len(feas),'worst_block_joint_margin':min([z['joint'] for z in best],default=float('inf')),'communication_geometry_pass':geo,'relay_chain_ok':False,'component_ok':False,'route_feasible':False,'transport_energy':energy,'relay_energy':relay_energy,'total_energy':energy+relay_energy,'makespan':makespan})
 out=pd.DataFrame(rows).sort_values(['route_feasible','worst_block_joint_margin'],ascending=[False,False]); out.to_csv(RES/'q3_C3_S008_three_point_screening.csv',index=False,encoding='utf-8-sig'); print('total',len(out),'transport',int((out.payload_ok&out.reserve_ok&out.hard_deadline_ok).sum()),'geometry',int(out.communication_geometry_pass.sum()),'best',float(out.worst_block_joint_margin.max()))
if __name__=='__main__': main()
