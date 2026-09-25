from __future__ import annotations
import math, json
from pathlib import Path
import numpy as np, pandas as pd
from minimal_pipeline import load_inputs, q1, q2, node_map, terrain_profile, line_geometry, xy
from q3_official_semantics import build_transport_trajectory
from build_q3_strict_certificate import distance_bound

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; RES.mkdir(exist_ok=True)
TOL=1e-9

def thresholds(D):
 c=D['comm']; sens=float(c[('接收参数','接收灵敏度（dBm）')]); fade=float(c[('接收参数','衰落裕量（dB）')]); loss=float(c[('传播参数','系统损耗（dB）')])
 def tw(a,b): return float(a)+float(b)-sens-fade-loss
 direct=min(tw(c[('运输无人机','发射功率（dBm）')],c[('运输无人机','天线增益（dBi）')])+float(c[('固定网关 G01','天线增益（dBi）')]),tw(c[('固定网关 G01','发射功率（dBm）')],c[('固定网关 G01','天线增益（dBi）')])+float(c[('运输无人机','天线增益（dBi）')]))
 access=tw(c[('运输无人机','发射功率（dBm）')],c[('运输无人机','天线增益（dBi）')])+float(c[('中继接入端','天线增益（dBi）')])
 back=min(tw(c[('中继回传端','发射功率（dBm）')],c[('中继回传端','天线增益（dBi）')])+float(c[('固定网关 G01','天线增益（dBi）')]),tw(c[('固定网关 G01','发射功率（dBm）')],c[('固定网关 G01','天线增益（dBi）')])+float(c[('中继回传端','天线增益（dBi）')]))
 return direct,access,back

def pkey(p): return (round(float(p['经度（°）']),10),round(float(p['纬度（°）']),10))
def ground(D,p):
 vals=[v for _,v in terrain_profile(p,p,D['dem']) if np.isfinite(v)]
 return max(vals) if vals else float('nan')
def xyz_m(D,p,h):
 lat0=float(D['center']['纬度（°）']); return np.array(xy(float(p['经度（°）']),float(p['纬度（°）']),lat0)+(float(h),))
PROFILE_CACHE={}
def cached_profile(D,a,b):
    key=(round(float(a['经度（°）']),8),round(float(a['纬度（°）']),8),round(float(b['经度（°）']),8),round(float(b['纬度（°）']),8))
    if key not in PROFILE_CACHE: PROFILE_CACHE[key]=terrain_profile(a,b,D['dem'])
    return PROFILE_CACHE[key]

def link_bound(D,a,b,ha,hb,thr,receiver,receiver_h):
    # Conservative continuous bound: analytic maximum distance plus worst-case
    # terrain obstruction loss. No endpoint sampling is used as a pass gate.
    dmax,_=distance_bound(D,a,b,ha,hb,receiver,receiver_h)
    f=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')])
    fspl=32.45+20*math.log10(f)+20*math.log10(max(dmax/1000,1e-12))
    return float(thr-(fspl+obs)),float('nan'),float(dmax),True

def gen_xy(D,service):
 o=D['center']; s=node_map(D)[service]; lo0,la0=float(o['经度（°）']),float(o['纬度（°）']); lo1,la1=float(s['经度（°）']),float(s['纬度（°）'])
 # bounding box grid plus route and perpendicular offsets; clipped only by DEM validity later
 xs=np.linspace(min(lo0,lo1),max(lo0,lo1),4); ys=np.linspace(min(la0,la1),max(la0,la1),4)
 pts=[{'经度（°）':float(x),'纬度（°）':float(y)} for x in xs for y in ys]
 dx,dy=lo1-lo0,la1-la0; norm=math.hypot(dx,dy) or 1.0
 for f in np.linspace(.15,.85,5):
  x=lo0+f*dx; y=la0+f*dy
  for off in (-.25,-.125,0,.125,.25):
   # degree-scale perpendicular offset, local diagnostic envelope
   pts.append({'经度（°）':x-off*dy/norm*(abs(dx)+abs(dy)+1e-9),'纬度（°）':y+off*dx/norm*(abs(dx)+abs(dy)+1e-9)})
 out=[]; seen=set()
 for p in pts:
  if pkey(p) in seen: continue
  if np.isfinite(ground(D,p)):
   seen.add(pkey(p)); out.append(p)
 return out

def main():
 D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); nm=node_map(D); direct_thr,access_thr,back_thr=thresholds(D); gateway=D['center']; gh=float(gateway['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); levels=[100.,200.,300.]
 services=sorted(tasks['service'].astype(str).unique()); margin_rows=[]; service_rows=[]; rejection=[]; candidate_counts=[]
 for svc in services:
  t=tasks[tasks.service.astype(str)==svc].iloc[0].to_dict(); t['start_s']=0.0; traj=build_transport_trajectory(D,t,0.0); s=nm[svc]; target_h=max(v for _,v in terrain_profile(s,s,D['dem']) if np.isfinite(v))+30.0
  # direct-required intervals independently from direct link
  req=[]
  for i,e in enumerate(traj):
   md,cl,dist,los=link_bound(D,e['a'],e['b'],e['ha'],e['hb'],direct_thr,gateway,gh)
   if md < -TOL: req.append((float(e['t0']),float(e['t1']),i))
  req_all=list(req)
  # merge contiguous required primitives for reporting only;
  # rows retain the complete interval bounds used for the conservative check.
  merged=[]
  for q in req:
   if merged and q[0] <= merged[-1][1] + 1e-9: merged[-1]=(merged[-1][0],q[1],merged[-1][2],q[2])
   else: merged.append((q[0],q[1],q[2],q[2]))
  req=[(a,b,ia) for a,b,ia,ib in merged]
  xy_pts=gen_xy(D,svc); candidates=[]; feasible=[]
  for j,rp0 in enumerate(xy_pts):
   rg=ground(D,rp0)
   for lev in levels:
    rh=rg+lev
    if lev>float(D['relay']['max_height_m'])+TOL: continue
    cid=f'{svc}-H{j:03d}-Z{int(lev):03d}'
    cand={'candidate_id':cid,'service':svc,'x':rp0['经度（°）'],'y':rp0['纬度（°）'],'altitude':rh,'ground':rg,'req_count':len(req_all),'access_ok':True,'backhaul_ok':True,'min_a':float('inf'),'min_b':float('inf'),'min_clear':float('inf')}
    # static backhaul bound is independently computed once but recorded for each required interval
    mb,cb,db,lb=link_bound(D,rp0,rp0,rh,rh,back_thr,gateway,gh)
    cand['min_b']=mb; cand['backhaul_ok']=bool(mb>=-TOL)
    for t0,t1,ii in req_all:
      e=traj[ii]
      ma,ca,da,la=link_bound(D,e['a'],e['b'],e['ha'],e['hb'],access_thr,rp0,rh)
      cand['min_a']=min(cand['min_a'],ma); cand['min_clear']=min(cand['min_clear'],ca)
      cand['access_ok'] &= bool(ma>=-TOL); cand['backhaul_ok'] &= bool(mb>=-TOL)
      margin_rows.append({'service':svc,'sortie':str(t['sortie']),'candidate_id':cid,'interval_start':t0,'interval_end':t1,'access_margin_lower_bound':ma,'backhaul_margin_lower_bound':mb,'access_pass':bool(ma>=-TOL and la),'backhaul_pass':bool(mb>=-TOL and lb),'access_clearance_lower_bound':ca,'backhaul_clearance_lower_bound':cb,'candidate_hover_x':rp0['经度（°）'],'candidate_hover_y':rp0['纬度（°）'],'candidate_altitude':rh})
    if not req:
      cand['min_a']=float('inf'); cand['access_ok']=True
    cand['joint_ok']=bool(cand['access_ok'] and cand['backhaul_ok']); candidates.append(cand)
    if cand['joint_ok']: feasible.append(cand)
    else:
      if not cand['access_ok'] and cand['backhaul_ok']: reason='ACCESS_LINK_FAIL'
      elif cand['access_ok'] and not cand['backhaul_ok']: reason='BACKHAUL_LINK_FAIL'
      elif not cand['access_ok'] and not cand['backhaul_ok']: reason='ACCESS_AND_BACKHAUL_FAIL'
      else: reason='OTHER_TRUE_UNKNOWN'
      rejection.append({'service':svc,'sortie':str(t['sortie']),'candidate_id':cid,'candidate_hover_x':rp0['经度（°）'],'candidate_hover_y':rp0['纬度（°）'],'candidate_altitude':rh,'required_interval_count':len(req),'min_access_margin_db':cand['min_a'],'min_backhaul_margin_db':cand['min_b'],'access_pass':cand['access_ok'],'backhaul_pass':cand['backhaul_ok'],'rejection_reason':reason})
  candidate_counts.append({'service':svc,'xy_candidates':len(xy_pts),'candidate_total':len(candidates),'required_intervals':len(req_all)})
  if feasible:
   best=max(feasible,key=lambda c:min(c['min_a'],c['min_b']))
   service_rows.append({'service_area':svc,'direct_full_path_pass':len(req)==0,'relay_candidate_total':len(candidates),'relay_candidate_feasible':len(feasible),'best_candidate_x':best['x'],'best_candidate_y':best['y'],'best_candidate_altitude':best['altitude'],'best_access_margin':best['min_a'],'best_backhaul_margin':best['min_b'],'best_joint_margin':min(best['min_a'],best['min_b']),'relay_feasible':True,'failure_reason':'NONE'})
  else:
   service_rows.append({'service_area':svc,'direct_full_path_pass':len(req)==0,'relay_candidate_total':len(candidates),'relay_candidate_feasible':0,'best_candidate_x':np.nan,'best_candidate_y':np.nan,'best_candidate_altitude':np.nan,'best_access_margin':max([c['min_a'] for c in candidates],default=np.nan),'best_backhaul_margin':max([c['min_b'] for c in candidates],default=np.nan),'best_joint_margin':max([min(c['min_a'],c['min_b']) for c in candidates],default=np.nan),'relay_feasible':False,'failure_reason':'NO_STRICT_FEASIBLE_CANDIDATE'})
 pd.DataFrame(margin_rows).to_csv(RES/'q3_access_backhaul_margin_audit.csv',index=False,encoding='utf-8-sig')
 pd.DataFrame(rejection).to_csv(RES/'q3_communication_rejection_detailed_v2.csv',index=False,encoding='utf-8-sig')
 pd.DataFrame(service_rows).to_csv(RES/'q3_service_relay_feasibility_map_v2.csv',index=False,encoding='utf-8-sig')
 pd.DataFrame(candidate_counts).to_csv(RES/'q3_hover_candidate_generator_audit_v2.csv',index=False,encoding='utf-8-sig')
 # summarized counts and compare old scheduler failures conservatively
 old=pd.read_csv(RES/'q3_communication_rejection_detailed.csv') if (RES/'q3_communication_rejection_detailed.csv').exists() else pd.DataFrame()
 counts=pd.Series([x['rejection_reason'] for x in rejection]).value_counts().to_dict()
 meta={'services':len(services),'candidate_total':int(sum(x['candidate_total'] for x in candidate_counts)),'candidate_rejections':len(rejection),'new_reason_counts':counts,'old_unknown_rows':int(len(old)),'old_reason_counts':old.rejection_reason.value_counts().to_dict() if len(old) else {},'method':'independent grid+route-offset candidates; official dynamic trajectory; separate access/backhaul bounds; no strict_q3 call'}
 (RES/'q3_candidate_diagnostic_v2_meta.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(meta,ensure_ascii=False,indent=2)); print(pd.DataFrame(service_rows).to_string(index=False))
if __name__=='__main__': main()
