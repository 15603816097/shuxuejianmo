from __future__ import annotations
import math,json
from pathlib import Path
import numpy as np,pandas as pd
from scipy.optimize import differential_evolution
from minimal_pipeline import load_inputs,q1,q2,node_map,terrain_profile,xy
from q3_official_semantics import build_transport_trajectory,relay_leg_time_energy
from build_q3_strict_certificate import distance_bound
from q3_five_service_hover_refinement import thresholds
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-9

def xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)

def ground(D,lon,lat):
    # Fast legal DEM lookup for global search; final link certificate still uses
    # all relay-required trajectory primitives and analytic distance bounds.
    lons=np.asarray(D['dem']['longitude']).ravel(); lats=np.asarray(D['dem']['latitude']).ravel(); z=np.asarray(D['dem']['dem']);
    j=int(np.argmin(np.abs(lons-float(lon)))); i=int(np.argmin(np.abs(lats-float(lat)))); val=float(z[i,j]); nod=float(np.asarray(D['dem'].get('nodata',[-32767])).ravel()[0]);
    return val if np.isfinite(val) and val!=nod else float('nan')

def main():
 D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); t=tasks[tasks.service.astype(str)=='S008'].iloc[0].to_dict(); t['start_s']=0.; traj=build_transport_trajectory(D,t,0.)
 td,ta,tb=thresholds(D); c=D['center']; gh=float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); G=xyz(D,c,gh)
 # Direct certificate and actual relay-required primitives.
 req=[]; audit=[]
 for i,e in enumerate(traj):
  A=xyz(D,e['a'],e['ha']); B=xyz(D,e['b'],e['hb']); d=max(float(np.linalg.norm(A-G)),float(np.linalg.norm(B-G))); fspl=32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12)); margin=td-fspl-obs; direct=margin>=-TOL
  audit.append({'primitive_interval':i,'start':e['t0'],'end':e['t1'],'phase':e['phase'],'direct_available':direct,'relay_required':not direct,'direct_margin_lower_bound_db':margin,'transport_start_xyz':json.dumps(A.tolist()),'transport_end_xyz':json.dumps(B.tolist())})
  if not direct:req.append(e)
 pd.DataFrame(audit).to_csv(RES/'q3_S008_relay_required_interval_audit.csv',index=False,encoding='utf-8-sig')
 # Precompute endpoint vectors for strict interval distance bounds.
 ep=[]
 for e in req: ep.extend([xyz(D,e['a'],e['ha']),xyz(D,e['b'],e['hb'])])
 ep=np.asarray(ep); R0=xyz(D,c,float(c['海拔（m）']))
 # DEM bounds are the actual data domain.
 lons=np.asarray(D['dem']['longitude']).ravel(); lats=np.asarray(D['dem']['latitude']).ravel(); bounds=[(float(lons.min()),float(lons.max())),(float(lats.min()),float(lats.max())),(50.,float(D['relay']['max_height_m']))]
 cache={}
 def eval_v(v,detail=False):
  lon,lat,off=map(float,v); key=(round(lon,8),round(lat,8),round(off,5));
  if key in cache and not detail:return cache[key]
  g=ground(D,lon,lat)
  if not np.isfinite(g): return 1e3
  rh=g+off; R=xyz(D,{'经度（°）':lon,'纬度（°）':lat},rh)
  if len(ep): d=np.linalg.norm(ep-R,axis=1); dmax=float(d.max())
  else:dmax=0.
  fspl=32.45+20*math.log10(freq)+20*math.log10(max(dmax/1000,1e-12)); access=ta-(fspl+obs)
  db=float(np.linalg.norm(R-R0)); fsplb=32.45+20*math.log10(freq)+20*math.log10(max(db/1000,1e-12)); back=tb-(fsplb+obs)
  P0=xyz(D,c,float(c['海拔（m）'])); dist=float(np.linalg.norm(R-P0)); peak=max(float(g),float(c['海拔（m）'])); _,eo=relay_leg_time_energy(D,dist,peak,float(c['海拔（m）']),rh); _,eb=relay_leg_time_energy(D,dist,peak,rh,float(c['海拔（m）'])); dur=(max(e['t1'] for e in req)-min(e['t0'] for e in req)) if req else 0.; energy=eo+eb+(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*dur/3600.; avail=float(D['relay']['energy_kwh'])*(1-float(D['relay']['reserve'])); energy_ok=energy<=avail+TOL; joint=min(access,back); val=-joint if energy_ok else 100+(-joint)
  out={'x':lon,'y':lat,'altitude':rh,'height_offset':off,'access_margin':access,'backhaul_margin':back,'joint_margin':joint,'energy_ok':energy_ok,'dem_ok':True,'height_ok':50-TOL<=off<=float(D['relay']['max_height_m'])+TOL,'relay_energy_kwh':energy,'relay_energy_available_kwh':avail,'objective':val,'dmax_m':dmax,'backhaul_distance_m':db}
  cache[key]=val
  return out if detail else val
 # Multiple independent DE seeds.
 results=[]
 for seed in [11,23,37,53,71,89]:
  de=differential_evolution(lambda v:eval_v(v),bounds,seed=seed,popsize=8,maxiter=35,tol=1e-5,polish=True,workers=1,updating='immediate')
  results.append(eval_v(de.x,True)|{'seed':seed,'de_fun':float(de.fun),'nfev':int(de.nfev)})
 # Stage 2 strict evaluation of top regions (same full primitive bound, no relaxed check).
 top=sorted(results,key=lambda z:z['joint_margin'],reverse=True)
 rows=[]
 for rank,z in enumerate(top,1): rows.append({'rank':rank,'seed':z['seed'],'x':z['x'],'y':z['y'],'altitude':z['altitude'],'height_offset':z['height_offset'],'access_margin':z['access_margin'],'backhaul_margin':z['backhaul_margin'],'joint_margin':z['joint_margin'],'energy_ok':z['energy_ok'],'dem_ok':z['dem_ok'],'height_ok':z['height_ok'],'strict_certificate_pass':bool(z['joint_margin']>0 and z['energy_ok'] and z['dem_ok'] and z['height_ok']),'candidate_evaluations':z['nfev']})
 pd.DataFrame(rows).to_csv(RES/'q3_S008_global_hover_optimization.csv',index=False,encoding='utf-8-sig')
 best=top[0]; # binding primitive at final best
 R=xyz(D,{'经度（°）':best['x'],'纬度（°）':best['y']},best['altitude']); bind=[]
 for i,e in enumerate(req):
  A=xyz(D,e['a'],e['ha']); B=xyz(D,e['b'],e['hb']); d=max(float(np.linalg.norm(A-R)),float(np.linalg.norm(B-R))); fspl=32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12)); m=ta-(fspl+obs); bind.append({'primitive_interval':i,'time_start':e['t0'],'time_end':e['t1'],'phase':e['phase'],'transport_start_xyz':json.dumps(A.tolist()),'transport_end_xyz':json.dumps(B.tolist()),'relay_xyz':json.dumps(R.tolist()),'dem_obstruction_state':'worst_case_obstacle_loss_applied','distance_m':d,'fspl_db':fspl,'obstacle_loss_db':obs,'access_threshold_db':ta,'access_margin_db':m})
 bd=min(bind,key=lambda x:x['access_margin_db']) if bind else None; pd.DataFrame([bd] if bd else []).to_csv(RES/'q3_S008_binding_access_interval.csv',index=False,encoding='utf-8-sig')
 summary={'relay_required_interval_count':len(req),'primitive_interval_count':len(traj),'direct_available_interval_count':sum(not x['relay_required'] for x in audit),'best_x':best['x'],'best_y':best['y'],'best_altitude':best['altitude'],'height_above_ground':best['height_offset'],'best_access_margin':best['access_margin'],'best_backhaul_margin':best['backhaul_margin'],'best_joint_margin':best['joint_margin'],'binding_interval':None if bd is None else bd['primitive_interval'],'candidate_evaluations':sum(z['nfev'] for z in results),'strict_certificate_pass':bool(best['joint_margin']>0 and best['energy_ok'] and best['dem_ok'] and best['height_ok']),'seeds':[11,23,37,53,71,89],'search_domain':bounds,'note':'Global DE over full DEM longitude/latitude domain; strict bound uses all relay-required primitive endpoints and worst-case terrain loss.'}
 (RES/'q3_S008_global_hover_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2)); print('top',pd.DataFrame(rows).head().to_string(index=False))
if __name__=='__main__':main()
