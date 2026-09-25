from __future__ import annotations
import math,json
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,q1,q2,node_map,terrain_profile,xy
from q3_official_semantics import build_transport_trajectory
from q3_five_service_hover_refinement import thresholds,evaluate
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-9

def ground(D,p):
 vals=[v for _,v in terrain_profile(p,p,D['dem']) if np.isfinite(v)]
 return max(vals) if vals else float('nan')
def xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)

def main():
 D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); svc='S008'; t=tasks[tasks.service.astype(str)==svc].iloc[0].to_dict(); t['start_s']=0.; traj=build_transport_trajectory(D,t,0.)
 td,ta,tb=thresholds(D); gh=float(D['center']['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')])
 req=[]
 for e in traj:
  a,b0=e['a'],e['b']; A=xyz(D,a,e['ha']); B=xyz(D,b0,e['hb']); G=xyz(D,D['center'],gh); d=max(float(np.linalg.norm(A-G)),float(np.linalg.norm(B-G))); md=td-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+obs)
  if md<0:req.append(e)
 base=pd.read_csv(RES/'q3_five_service_hover_refinement.csv'); seeds=base[base.service_area==svc].sort_values('joint_margin',ascending=False).head(20)
 rows=[]; seen=set(); best=None
 def test(x,y,off,level):
  nonlocal best
  p={'经度（°）':float(x),'纬度（°）':float(y)}; g=ground(D,p)
  if not np.isfinite(g): return False
  if level<50-TOL or level>float(D['relay']['max_height_m'])+TOL:return False
  key=(round(float(x),9),round(float(y),9),round(float(level),4))
  if key in seen:return False
  seen.add(key); z=evaluate(D,svc,req,traj,(float(x),float(y),g,'local'),float(level),ta,tb,gh)
  if z is None:return False
  z.update({'candidate_id':f'S008-L{len(rows):06d}','source':'super_local','height_offset':float(level),'dem_ok':True,'height_ok':True,'energy_ok':bool(z['relay_energy_ok']),'feasible':bool(z['valid'])}); rows.append(z)
  if best is None or z['joint_margin']>best['joint_margin']: best=z
  return bool(z['valid'])
 # multi-level windows around top seeds; continue through all to establish best
 for _,s in seeds.iterrows():
  g=ground(D,{'经度（°）':s.x,'纬度（°）':s.y}); off=float(s.altitude)-g
  for dx in np.linspace(-0.002,0.002,9):
   for dy in np.linspace(-0.002,0.002,9):
    for da in np.linspace(-50,50,11): test(s.x+dx,s.y+dy,off+da,off+da)
 # refine around top 20 new candidates
 for _,s in pd.DataFrame(rows).sort_values('joint_margin',ascending=False).head(20).iterrows():
  for dx in np.linspace(-0.0005,0.0005,7):
   for dy in np.linspace(-0.0005,0.0005,7):
    for da in np.linspace(-20,20,9): test(s.x+dx,s.y+dy,s.height_offset+da,s.height_offset+da)
 # final very local pass
 for _,s in pd.DataFrame(rows).sort_values('joint_margin',ascending=False).head(10).iterrows():
  for dx in np.linspace(-0.0001,0.0001,5):
   for dy in np.linspace(-0.0001,0.0001,5):
    for da in np.linspace(-5,5,5): test(s.x+dx,s.y+dy,s.height_offset+da,s.height_offset+da)
 out=pd.DataFrame(rows); out.to_csv(RES/'q3_S008_final_refinement.csv',index=False,encoding='utf-8-sig')
 z=out.sort_values('joint_margin',ascending=False).iloc[0]; summary={'candidate_count':len(out),'best_x':float(z.x),'best_y':float(z.y),'best_altitude':float(z.altitude),'height_offset':float(z.height_offset),'best_access_margin':float(z.access_margin),'best_backhaul_margin':float(z.backhaul_margin),'best_joint_margin':float(z.joint_margin),'energy_ok':bool(z.energy_ok),'dem_ok':bool(z.dem_ok),'height_ok':bool(z.height_ok),'feasible':bool(z.feasible),'valid_feasible_count':int(out.feasible.sum()),'seed_count':len(seeds),'search':'top20 seeds; levels ±50m/10m, ±20m/5m, ±5m/2.5m; joint XY+altitude'}
 (RES/'q3_S008_final_refinement_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
