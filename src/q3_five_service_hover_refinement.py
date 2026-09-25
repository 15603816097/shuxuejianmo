from __future__ import annotations
import math,json
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,q1,q2,node_map,terrain_profile,xy,line_geometry
from q3_official_semantics import build_transport_trajectory
from build_q3_strict_certificate import distance_bound
from q3_relay_candidate_diagnostic_v2 import thresholds,ground
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-9
TARGET=['S002','S003','S004','S008','S012']

def candidate_xy(D,svc):
 o=D['center']; s=node_map(D)[svc]; lo0,la0=float(o['经度（°）']),float(o['纬度（°）']); lo1,la1=float(s['经度（°）']),float(s['纬度（°）'])
 dlo,dla=abs(lo1-lo0),abs(la1-la0); exlo=max(dlo*.20,0.003); exla=max(dla*.20,0.003)
 xs=np.linspace(min(lo0,lo1)-exlo,max(lo0,lo1)+exlo,10); ys=np.linspace(min(la0,la1)-exla,max(la0,la1)+exla,10)
 pts=[(float(x),float(y),'bbox_grid') for x in xs for y in ys]
 dx,dy=lo1-lo0,la1-la0; norm=math.hypot(dx,dy) or 1
 for f in np.linspace(.05,.95,15):
  x=lo0+f*dx; y=la0+f*dy
  for off in np.linspace(-.35,.35,5):
   scale=max(dlo+dla,.005); pts.append((x-off*dy/norm*scale,y+off*dx/norm*scale,'route_offset'))
 # local service mesh
 for x in np.linspace(lo1-exlo*.5,lo1+exlo*.5,7):
  for y in np.linspace(la1-exla*.5,la1+exla*.5,7): pts.append((float(x),float(y),'service_mesh'))
 out=[]; seen=set()
 for x,y,src in pts:
  p={'经度（°）':x,'纬度（°）':y}; g=ground(D,p)
  if np.isfinite(g) and (round(x,9),round(y,9)) not in seen:
   seen.add((round(x,9),round(y,9))); out.append((x,y,g,src))
 return out

def endpoint_xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)

def evaluate(D,svc,req,all_phases,rp,level,thr_a,thr_b,gh):
 x,y,g,_=rp; rh=g+level; relay={'经度（°）':x,'纬度（°）':y}
 if level>float(D['relay']['max_height_m'])+TOL:return None
 # Piecewise linear trajectory: max distance to fixed relay is attained at endpoints.
 pts=[]
 for e in req: pts.extend([endpoint_xyz(D,e['a'],e['ha']),endpoint_xyz(D,e['b'],e['hb'])])
 if not pts:return None
 R=endpoint_xyz(D,relay,rh); arr=np.asarray(pts); dists=np.linalg.norm(arr-R,axis=1); dmax=float(dists.max())
 freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); fspl=32.45+20*math.log10(freq)+20*math.log10(max(dmax/1000,1e-12)); access=float(thr_a-fspl-obs)
 # Backhaul distance and worst-case loss
 G=endpoint_xyz(D,D['center'],gh); db=float(np.linalg.norm(R-G)); fsplb=32.45+20*math.log10(freq)+20*math.log10(max(db/1000,1e-12)); back=float(thr_b-fsplb-obs)
 # Official relay energy/reserve check, using two legs + hover/communication over demand duration.
 r=D['relay']; dist_out,peak_out,*_=line_geometry(D['center'],relay,D['dem']); t_out,e_out=__import__('q3_official_semantics').relay_leg_time_energy(D,dist_out,peak_out,float(D['center']['海拔（m）']),rh); t_back,e_back=__import__('q3_official_semantics').relay_leg_time_energy(D,dist_out,peak_out,rh,float(D['center']['海拔（m）'])); dur=(max(float(e['t1']) for e in req)-min(float(e['t0']) for e in req)) if req else 0.0; energy=e_out+e_back+(float(r['hover_power_kw'])+float(r['comm_power_kw']))*dur/3600.0; avail=float(r['energy_kwh'])*(1-float(r['reserve'])); energy_ok=bool(energy<=avail+1e-9)
 return {'x':x,'y':y,'altitude':rh,'ground':g,'access_margin':access,'backhaul_margin':back,'joint_margin':min(access,back),'relay_energy_kwh':energy,'relay_energy_available_kwh':avail,'relay_energy_ok':energy_ok,'valid':bool(access>0 and back>0 and energy_ok),'dmax_m':dmax,'backhaul_distance_m':db}

def main():
 D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); nm=node_map(D); td,ta,tb=thresholds(D); gh=float(D['center']['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); allrows=[]; summary=[]
 for svc in TARGET:
  t=tasks[tasks.service.astype(str)==svc].iloc[0].to_dict(); t['start_s']=0.; traj=build_transport_trajectory(D,t,0.); req=[]
  for i,e in enumerate(traj):
   # direct link lower bound; worst-case obstacle is applied consistently
   pts=[endpoint_xyz(D,e['a'],e['ha']),endpoint_xyz(D,e['b'],e['hb']),endpoint_xyz(D,D['center'],gh)]
   d=max(np.linalg.norm(pts[0]-pts[2]),np.linalg.norm(pts[1]-pts[2])); f=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); md=td-(32.45+20*math.log10(f)+20*math.log10(max(d/1000,1e-12))+obs)
   if md<0:req.append(e)
  base=candidate_xy(D,svc); levels=[50.,100.,150.,200.,250.,300.]; coarse=[]
  for rp in base:
   for lev in levels:
    z=evaluate(D,svc,req,traj,rp,lev,ta,tb,gh)
    if z: z.update({'service_area':svc,'source':'coarse','candidate_id':f'{svc}-C{len(coarse):05d}'}); coarse.append(z)
  top=sorted(coarse,key=lambda z:z['joint_margin'],reverse=True)[:30]
  refined=[]; seen=set()
  for z in top:
   for dx in np.linspace(-0.002,0.002,5):
    for dy in np.linspace(-0.002,0.002,5):
     for da in (-25.,0.,25.):
      p=(z['x']+dx,z['y']+dy,'local_refine'); g=ground(D,{'经度（°）':p[0],'纬度（°）':p[1]})
      if not np.isfinite(g):continue
      key=(round(p[0],8),round(p[1],8),round(g+z['altitude']-z['ground']+da,3))
      if key in seen:continue
      seen.add(key); lev=max(1.,z['altitude']-z['ground']+da); zz=evaluate(D,svc,req,traj,(p[0],p[1],g,p[2]),lev,ta,tb,gh)
      if zz: zz.update({'service_area':svc,'source':'local_refine','candidate_id':f'{svc}-R{len(refined):05d}'}); refined.append(zz)
  pool=coarse+refined; valid=[z for z in pool if z['valid']]; best=max(pool,key=lambda z:z['joint_margin']) if pool else None
  for rank,z in enumerate(sorted(pool,key=lambda z:z['joint_margin'],reverse=True)[:50],1):
   allrows.append({'service_area':svc,'candidate_rank':rank,'x':z['x'],'y':z['y'],'altitude':z['altitude'],'access_margin':z['access_margin'],'backhaul_margin':z['backhaul_margin'],'joint_margin':z['joint_margin'],'relay_energy_ok':z['relay_energy_ok'],'valid':z['valid'],'source':z['source'],'candidate_count_total':len(pool)})
  summary.append({'service_area':svc,'coarse_candidate_count':len(coarse),'refined_candidate_count':len(refined),'candidate_count':len(pool),'relay_candidate_found':bool(valid),'best_access_margin':best['access_margin'],'best_backhaul_margin':best['backhaul_margin'],'best_joint_margin':best['joint_margin'],'best_altitude':best['altitude'],'best_relay_energy_ok':best['relay_energy_ok'],'search_area':'expanded bbox + route offsets + service mesh + top30 local 5x5x3','altitude_range_m':f"{min(levels)}..{max(levels)} above DEM"})
 pd.DataFrame(allrows).to_csv(RES/'q3_five_service_hover_refinement.csv',index=False,encoding='utf-8-sig'); pd.DataFrame(summary).to_csv(RES/'q3_five_service_hover_refinement_summary.csv',index=False,encoding='utf-8-sig'); (RES/'q3_five_service_hover_refinement.json').write_text(json.dumps({'targets':TARGET,'summary':summary,'method':'independent access/backhaul analytic bound; no strict_q3 call'},ensure_ascii=False,indent=2),encoding='utf-8'); print(pd.DataFrame(summary).to_string(index=False))
if __name__=='__main__':main()
