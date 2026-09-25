from __future__ import annotations
import math,json
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,node_map,xy,terrain_profile
from q3_C2_S008_route_screening_v2 import multi_traj
from q3_five_service_hover_refinement import thresholds
from q3_official_semantics import relay_leg_time_energy
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-9

def xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)
def ground(D,p):
 v=[z for _,z in terrain_profile(p,p,D['dem']) if np.isfinite(z)]; return max(v) if v else np.nan

def main():
 D=load_inputs(); order=('S011','S008'); traj=multi_traj(D,order,'C'); td,ta,tb=thresholds(D); c=D['center']; gh=float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); G=xyz(D,c,gh)
 req=[]
 for i,e in enumerate(traj):
  A=xyz(D,e['a'],e['ha']); B=xyz(D,e['b'],e['hb']); d=max(np.linalg.norm(A-G),np.linalg.norm(B-G)); m=td-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+obs); e=dict(e); e['primitive_index']=i; e['direct_margin']=m; e['relay_required']=m<0
  if e['relay_required']: req.append(e)
 blocks=[]
 for e in req:
  if blocks and abs(float(e['t0'])-float(blocks[-1]['end']))<1e-6: blocks[-1]['items'].append(e); blocks[-1]['end']=e['t1']
  else: blocks.append({'start':e['t0'],'end':e['t1'],'items':[e]})
 candidates=[]; chosen=[]
 for bi,b in enumerate(blocks,1):
  first,last=b['items'][0],b['items'][-1]; a0=first['a']; b0=last['b']; ep=[]
  for e in b['items']:ep.extend([xyz(D,e['a'],e['ha']),xyz(D,e['b'],e['hb'])])
  ep=np.asarray(ep); best=[]
  for f in np.linspace(.05,.95,19):
   rp={'经度（°）':float(a0['经度（°）'])+f*(float(b0['经度（°）'])-float(a0['经度（°）'])),'纬度（°）':float(a0['纬度（°）'])+f*(float(b0['纬度（°）'])-float(a0['纬度（°）']))}; rg=ground(D,rp)
   if not np.isfinite(rg):continue
   for off in [50.,100.,150.,200.,250.,300.]:
    rh=rg+off; R=xyz(D,rp,rh); da=float(np.linalg.norm(ep-R,axis=1).max()); db=float(np.linalg.norm(R-G)); ma=ta-(32.45+20*math.log10(freq)+20*math.log10(max(da/1000,1e-12))+obs); mb=tb-(32.45+20*math.log10(freq)+20*math.log10(max(db/1000,1e-12))+obs); dist=np.linalg.norm(R-xyz(D,c,float(c['海拔（m）']))); energy=2*dist/1000*float(D['relay']['power_kw'])/float(D['relay']['speed'])/3.6 + (float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(b['end']-b['start'])/3600; ok=ma>0 and mb>0 and energy<=float(D['relay']['energy_kwh'])*(1-float(D['relay']['reserve']))
    candidates.append({'block_id':bi,'block_start':b['start'],'block_end':b['end'],'hover_x':rp['经度（°）'],'hover_y':rp['纬度（°）'],'altitude':rh,'access_margin':ma,'backhaul_margin':mb,'joint_margin':min(ma,mb),'energy':energy,'energy_ok':energy<=float(D['relay']['energy_kwh'])*(1-float(D['relay']['reserve'])),'pass':ok})
  cc=[x for x in candidates if x['block_id']==bi]; bestc=max(cc,key=lambda x:x['joint_margin']); chosen.append(bestc)
 # assign relay IDs with chain timing from O01 and no overlap
 chain=[]; relay_free={'R01':0.,'R02':0.}; gaps=False
 for b in chosen:
  rid=min(relay_free,key=relay_free.get); out_dist=math.hypot((b['hover_x']-float(c['经度（°）']))*111000*math.cos(math.radians(float(c['纬度（°）']))),(b['hover_y']-float(c['纬度（°）']))*111000); t_out=out_dist/float(D['relay']['speed']); prep=float(D['relay']['prep_s']); setup=float(D['relay']['link_s']); service_start=float(b['block_start']); prep_start=service_start-prep-t_out-setup; return_end=service_start+float(b['block_end']-b['block_start'])+t_out; busy_end=return_end+float(D['relay']['turn_s']); if_ready=prep_start>=0 and prep_start>=relay_free[rid]-TOL; passc=bool(b['pass'] and if_ready); gaps |= not if_ready; relay_free[rid]=busy_end if passc else relay_free[rid]; chain.append({'block_id':b['block_id'],'block_start':b['block_start'],'block_end':b['block_end'],'relay_id':rid,'hover_x':b['hover_x'],'hover_y':b['hover_y'],'altitude':b['altitude'],'access_margin':b['access_margin'],'backhaul_margin':b['backhaul_margin'],'prepare_start':prep_start,'link_ready':service_start,'service_start':service_start,'service_end':b['block_end'],'relocation_or_return':f'return_end={return_end:.3f};busy_end={busy_end:.3f}','energy':b['energy'],'pass':passc})
 pd.DataFrame(candidates).to_csv(RES/'q3_C2_S008_block_candidates.csv',index=False,encoding='utf-8-sig'); pd.DataFrame(chain).to_csv(RES/'q3_C2_S008_relay_chain.csv',index=False,encoding='utf-8-sig'); summary={'route':'O01->S011->S008->O01','relay_required_service_blocks':len(blocks),'block_feasible_hover_count':sum(any(x['pass'] for x in candidates if x['block_id']==i) for i in range(1,len(blocks)+1)),'relay_chain_blocks':len(chain),'relay_ids_used':sorted({x['relay_id'] for x in chain}),'max_relays':2,'min_block_joint_margin':min(x['joint_margin'] for x in chosen) if chosen else None,'communication_holes':int(sum(not x['pass'] for x in chain)),'component_inventory':6,'route_feasible':bool(chain and all(x['pass'] for x in chain) and not gaps),'note':'Each block independently certified; relay timing is backsolved from block start, with return/turnaround and no relocation teleportation.'}; (RES/'q3_C2_S008_relay_chain_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2)); print(pd.DataFrame(chain).to_string(index=False))
if __name__=='__main__':main()
