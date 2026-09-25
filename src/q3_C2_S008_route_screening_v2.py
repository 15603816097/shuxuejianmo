from __future__ import annotations
import math,json,itertools
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,q1,q2,node_map,terrain_profile,xy,energy_leg,line_geometry
from q3_five_service_hover_refinement import thresholds
from q3_official_semantics import relay_leg_time_energy
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-9

def xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)
def ground(D,p):
 v=[z for _,z in terrain_profile(p,p,D['dem']) if np.isfinite(z)]; return max(v) if v else np.nan

def multi_traj(D,order,typ):
 nm=node_map(D); d=D['drones'][typ]; nodes=['O01']+list(order)+['O01']; out=[]; cur_t=float(d.prep); prev='O01'; prev_h=float(nm['O01']['海拔（m）'])
 for nxt in nodes[1:]:
  a=nm[prev]; b=nm[nxt]; end_h=float(b['海拔（m）'])+(30 if nxt!='O01' else 0); dist,peak,*_=line_geometry(a,b,D['dem']); cruise=peak+50; up=max(0,cruise-prev_h); down=max(0,cruise-end_h); tu=up/d.climb_speed; tc=dist/d.speed; td=down/d.descend_speed
  # climb
  if tu>0: out.append({'t0':cur_t,'t1':cur_t+tu,'phase':'climb','a':a,'b':a,'ha':prev_h,'hb':cruise})
  cur_t+=tu
  prof=terrain_profile(a,b,D['dem']); cuts=sorted({0.,1.} | {float(t) for t,_ in prof if 0<float(t)<1})
  for u,v in zip(cuts[:-1],cuts[1:]):
   aa={'经度（°）':float(a['经度（°）'])+u*(float(b['经度（°）'])-float(a['经度（°）'])),'纬度（°）':float(a['纬度（°）'])+u*(float(b['纬度（°）'])-float(a['纬度（°）']))}; bb={'经度（°）':float(a['经度（°）'])+v*(float(b['经度（°）'])-float(a['经度（°）'])),'纬度（°）':float(a['纬度（°）'])+v*(float(b['纬度（°）'])-float(a['纬度（°）']))}; out.append({'t0':cur_t+tc*u,'t1':cur_t+tc*v,'phase':'cruise','a':aa,'b':bb,'ha':cruise,'hb':cruise})
  cur_t+=tc
  if td>0: out.append({'t0':cur_t,'t1':cur_t+td,'phase':'descent','a':b,'b':b,'ha':cruise,'hb':end_h})
  cur_t+=td
  if nxt!='O01':
   out.append({'t0':cur_t,'t1':cur_t+float(d.handoff),'phase':'handoff','a':b,'b':b,'ha':end_h,'hb':end_h}); cur_t+=float(d.handoff)
  prev=nxt; prev_h=end_h
 return out

def communication(D,traj):
 td,ta,tb=thresholds(D); c=D['center']; gh=float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); G=xyz(D,c,gh); req=[]
 for i,e in enumerate(traj):
  A=xyz(D,e['a'],e['ha']); B=xyz(D,e['b'],e['hb']); d=max(np.linalg.norm(A-G),np.linalg.norm(B-G)); m=td-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+obs); e['direct_margin']=float(m); e['relay_required']=m<0
  if e['relay_required']: req.append(e)
 if not req:return {'ok':True,'req':[],'access':float('inf'),'backhaul':float('inf'),'joint':float('inf'),'candidate':None}
 ep=[]
 for e in req:ep.extend([xyz(D,e['a'],e['ha']),xyz(D,e['b'],e['hb'])])
 best=None; start=req[0]['a']; end=req[-1]['b'];
 for f in np.linspace(.05,.95,19):
  rp={'经度（°）':float(start['经度（°）'])+f*(float(end['经度（°）'])-float(start['经度（°）'])),'纬度（°）':float(start['纬度（°）'])+f*(float(end['纬度（°）'])-float(start['纬度（°）']))}; rg=ground(D,rp)
  if not np.isfinite(rg):continue
  for off in [50.,100.,150.,200.,250.,300.]:
   rh=rg+off; R=xyz(D,rp,rh); freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB')]) if False else float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); da=max(np.linalg.norm(ep-R,axis=1)); G=xyz(D,D['center'],gh); db=np.linalg.norm(R-G); ma=ta-(32.45+20*math.log10(freq)+20*math.log10(max(da/1000,1e-12))+obs); mb=tb-(32.45+20*math.log10(freq)+20*math.log10(max(db/1000,1e-12))+obs); z={'access':float(ma),'backhaul':float(mb),'joint':float(min(ma,mb)),'candidate':(rp['经度（°）'],rp['纬度（°）'],rh)}
   if best is None or z['joint']>best['joint']:best=z
 return {'ok':best['joint']>0,'req':req,'access':best['access'],'backhaul':best['backhaul'],'joint':best['joint'],'candidate':best['candidate']}

def main():
 D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); boxes=D['boxes']; bm={x['货箱编号']:x for x in boxes}; svcs=sorted({x['服务区编号'] for x in boxes if x['服务区编号']!='S008'}); nm=node_map(D); rows=[]; altrows=[]; comm_cache={}
 for partner in svcs:
  b8=[x for x in boxes if x['服务区编号']=='S008']; bp=[x for x in boxes if x['服务区编号']==partner]; subsets=[b8[:1],b8[:2],b8[:3],b8,b8[:2]+bp[:1],b8[:2]+bp[:2],b8+bp[:1],b8+bp[:2],b8[:1]+bp,b8[:2]+bp,b8+bp]
  # unique subsets
  seen=set()
  for order in [(partner,'S008'),('S008',partner)]:
   for subset in subsets:
    ids=tuple(sorted(x['货箱编号'] for x in subset)); key=(order,ids)
    if key in seen:continue
    seen.add(key)
    for typ in ['B','C']:
     d=D['drones'][typ]; mass=sum(float(x['单箱质量（kg）']) for x in subset); vol=sum(float(x['单箱体积（m³）']) for x in subset); payload=mass<=d.max_mass+TOL and vol<=d.volume+TOL
     ck=(order,typ)
     if ck not in comm_cache: comm_cache[ck]=communication(D,multi_traj(D,order,typ))
     tcur=0.; energy=0.; delivery={}; prev='O01'; nm2=nm
     # compute legs and delivery at handoff
     tcur=float(d.prep+d.load_box*len(subset)); rem_mass=mass
     for j,nxt in enumerate(list(order)+['O01']):
      a=nm2[prev]; bb=nm2[nxt]; dist,peak,*_=line_geometry(a,bb,D['dem']); e,ft=energy_leg(d,dist,peak,float(a['海拔（m）']),float(bb['海拔（m）'])+(30 if nxt!='O01' else 0),rem_mass); energy+=e; tcur+=ft
      if nxt!='O01':
       hand=d.handoff+d.handoff_box*sum(1 for x in subset if x['服务区编号']==nxt); tcur+=hand; delivery[nxt]=tcur; rem_mass-=sum(float(x['单箱质量（kg）']) for x in subset if x['服务区编号']==nxt)
      prev=nxt
     reserve=energy<=(1-d.reserve)*d.energy+TOL; hard=True; soft=0.
     for x in subset:
      dl=float(x['期望送达时间（s）']); ifhard=(x['物资类型']=='医疗物资' or x['是否首批保障']=='是')
      if ifhard: hard &= delivery.get(x['服务区编号'],1e99)<=dl+TOL
      soft+=max(0,delivery.get(x['服务区编号'],1e99)-dl)
     cm=comm_cache[(order,typ)]; route_feasible=payload and reserve and hard and cm['ok']
     rows.append({'route':f"O01->{order[0]}->{order[1]}->O01",'visit_order':'->'.join(order),'box_set':json.dumps(ids,ensure_ascii=False),'drone_type':typ,'payload_ok':payload,'volume_ok':vol<=d.volume+TOL,'reserve_ok':reserve,'hard_deadline_ok':hard,'soft_lateness':soft,'relay_required_interval_count':len(cm['req']),'best_access_margin':cm['access'],'best_backhaul_margin':cm['backhaul'],'best_joint_margin':cm['joint'],'communication_pass':cm['ok'],'relay_energy_ok':reserve,'route_feasible':route_feasible,'mass_kg':mass,'volume_m3':vol,'energy_kwh':energy,'delivery_times':json.dumps(delivery,ensure_ascii=False)})
 # semantic audit artifacts
 drows=[]
 for x in D['boxes']:
  cat='MEDICAL_HARD' if x['物资类型']=='医疗物资' else ('FIRST_BATCH_HARD' if x['是否首批保障']=='是' else 'EXPECTED_SOFT')
  drows.append({'box_id':x['货箱编号'],'service':x['服务区编号'],'category':cat,'hard_deadline_s':min(float(x['首批截止时间（s）']),float(x['期望送达时间（s）'])) if cat!='EXPECTED_SOFT' and x['首批截止时间（s）'] is not None else (float(x['期望送达时间（s）']) if cat=='MEDICAL_HARD' else ''),'expected_time_s':float(x['期望送达时间（s）']),'used_as_hard_constraint':cat!='EXPECTED_SOFT'})
 pd.DataFrame(drows).to_csv(RES/'q3_C2_deadline_semantics_audit.csv',index=False,encoding='utf-8-sig')
 arows=[]
 for order in [(s,'S008') for s in svcs]+[('S008',s) for s in svcs]:
  tr=multi_traj(D,order,'C')
  for i in range(1,len(tr)):
   if abs(float(tr[i-1]['t1'])-float(tr[i]['t0']))<1e-6:
    arows.append({'route':'O01->'+order[0]+'->'+order[1]+'->O01','phase_prev':tr[i-1]['phase'],'phase_next':tr[i]['phase'],'previous_segment_end_altitude':tr[i-1]['hb'],'next_segment_start_altitude':tr[i]['ha'],'continuous':abs(float(tr[i-1]['hb'])-float(tr[i]['ha']))<1e-6})
 pd.DataFrame(arows).to_csv(RES/'q3_C2_altitude_continuity_audit.csv',index=False,encoding='utf-8-sig')
 out=pd.DataFrame(rows); out.to_csv(RES/'q3_C2_S008_route_screening_v2.csv',index=False,encoding='utf-8-sig'); print('candidates',len(out),'hard',int(out.hard_deadline_ok.sum()),'payload_reserve',int((out.payload_ok&out.reserve_ok).sum()),'comm',int(out.communication_pass.sum()),'feasible',int(out.route_feasible.sum())); print(out[out.route_feasible].head().to_string(index=False))
if __name__=='__main__':main()
