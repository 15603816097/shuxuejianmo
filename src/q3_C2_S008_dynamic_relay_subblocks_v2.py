from __future__ import annotations
import math,json
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,node_map,terrain_profile,xy,line_geometry
from q3_C2_S008_route_screening_v2 import multi_traj
from q3_five_service_hover_refinement import thresholds
from q3_official_semantics import relay_leg_time_energy
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-9

def xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)
def ground(D,p):
 v=[z for _,z in terrain_profile(p,p,D['dem']) if np.isfinite(z)]; return max(v) if v else np.nan

def main():
 D=load_inputs(); order=('S011','S008'); typ='C'; traj=multi_traj(D,order,typ); td,ta,tb=thresholds(D); c=D['center']; gh=float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); G=xyz(D,c,gh); freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); req=[]; direct=[]
 for i,e in enumerate(traj):
  A=xyz(D,e['a'],e['ha']);B=xyz(D,e['b'],e['hb']);d=max(np.linalg.norm(A-G),np.linalg.norm(B-G));m=td-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+obs); e=dict(e);e['primitive_index']=i;e['direct_margin']=m;e['relay_required']=m<0;direct.append({'primitive_index':i,'start':e['t0'],'end':e['t1'],'phase':e['phase'],'direct_available':m>=0,'relay_required':m<0,'direct_margin':m});
  if e['relay_required']:req.append(e)
 blocks=[]
 for e in req:
  if blocks and e['phase']==blocks[-1]['phase'] and abs(float(e['t0'])-float(blocks[-1]['end']))<1e-6: blocks[-1]['items'].append(e);blocks[-1]['end']=e['t1']
  else: blocks.append({'start':e['t0'],'end':e['t1'],'phase':e['phase'],'items':[e]})
 cand_rows=[]; bests=[]
 for bi,b in enumerate(blocks,1):
  ep=np.asarray([v for e in b['items'] for v in (xyz(D,e['a'],e['ha']),xyz(D,e['b'],e['hb']))]); a0=b['items'][0]['a']; b0=b['items'][-1]['b']; local=[]
  for f in np.linspace(.05,.95,19):
   rp={'经度（°）':float(a0['经度（°）'])+f*(float(b0['经度（°）'])-float(a0['经度（°）'])),'纬度（°）':float(a0['纬度（°）'])+f*(float(b0['纬度（°）'])-float(a0['纬度（°）']))}; g=ground(D,rp)
   if not np.isfinite(g):continue
   for off in [50.,100.,150.,200.,250.,300.]:
    rh=g+off;R=xyz(D,rp,rh);da=float(np.linalg.norm(ep-R,axis=1).max());db=float(np.linalg.norm(R-G));ma=ta-(32.45+20*math.log10(freq)+20*math.log10(max(da/1000,1e-12))+obs);mb=tb-(32.45+20*math.log10(freq)+20*math.log10(max(db/1000,1e-12))+obs);dist,peak,*_=line_geometry(c,rp,D['dem']);_,eo=relay_leg_time_energy(D,dist,peak,float(c['海拔（m）']),rh);_,eb=relay_leg_time_energy(D,dist,peak,rh,float(c['海拔（m）']));svc_e=(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(b['end']-b['start'])/3600;en=eo+eb+svc_e;ok=ma>0 and mb>0 and en<=float(D['relay']['energy_kwh'])*(1-float(D['relay']['reserve'])); z={'block_id':bi,'block_start':b['start'],'block_end':b['end'],'phase':b['phase'],'hover_x':rp['经度（°）'],'hover_y':rp['纬度（°）'],'altitude':rh,'access_margin':ma,'backhaul_margin':mb,'joint_margin':min(ma,mb),'energy':en,'energy_ok':en<=float(D['relay']['energy_kwh'])*(1-float(D['relay']['reserve'])),'pass':ok};local.append(z);cand_rows.append(z)
  bests.append(max(local,key=lambda x:x['joint_margin']))
 # allocate R01/R02 using formal prepare/outbound/link/service/return chain
 free={'R01':0.,'R02':0.}; chain=[]
 for b in bests:
  rid=min(free,key=free.get); rp={'经度（°）':b['hover_x'],'纬度（°）':b['hover_y']}; rg=ground(D,rp); rh=b['altitude'];dist,peak,*_=line_geometry(c,rp,D['dem']); tout,eout=relay_leg_time_energy(D,dist,peak,float(c['海拔（m）']),rh); tback,eback=relay_leg_time_energy(D,dist,peak,rh,float(c['海拔（m）'])); prep=float(D['relay']['prep_s']); setup=float(D['relay']['link_s']); pstart=b['block_start']-prep-tout-setup; link=b['block_start']; send=b['block_end']; rend=send+tback; busy=rend+float(D['relay']['turn_s']); ready=pstart>=free[rid]-TOL and pstart>=-TOL; passc=bool(b['pass'] and ready); free[rid]=busy if passc else free[rid]; chain.append({'subblock':b['block_id'],'start':b['block_start'],'end':b['block_end'],'phase':b['phase'],'relay_id':rid,'hover_x':b['hover_x'],'hover_y':b['hover_y'],'altitude':b['altitude'],'min_access_margin':b['access_margin'],'min_backhaul_margin':b['backhaul_margin'],'link_ready':link,'service_start':b['block_start'],'service_end':b['block_end'],'prepare_start':pstart,'relocation_start':None,'relocation_end':None,'return_start':send,'return_end':rend,'turnaround_end':busy,'energy':b['energy'],'pass':passc})
 pd.DataFrame(direct).to_csv(RES/'q3_C2_S008_dynamic_direct_audit.csv',index=False,encoding='utf-8-sig'); pd.DataFrame(cand_rows).to_csv(RES/'q3_C2_S008_dynamic_relay_subblocks.csv',index=False,encoding='utf-8-sig'); pd.DataFrame(chain).to_csv(RES/'q3_C2_S008_dynamic_relay_chain.csv',index=False,encoding='utf-8-sig'); summary={'route':'O01->S011->S008->O01','relay_required_service_blocks':len(blocks),'primitive_interval_count':len(traj),'block_feasible_hover_count':sum(any(x['pass'] for x in cand_rows if x['block_id']==i) for i in range(1,len(blocks)+1)),'relay_count_used':len(set(x['relay_id'] for x in chain)),'handoff_count':max(0,len(chain)-1),'communication_holes':sum(not x['pass'] for x in chain),'min_joint_margin':min(x['joint_margin'] for x in bests),'route_feasible':bool(chain and all(x['pass'] for x in chain)),'note':'Blocks are phase-contiguous; each block has independent hover candidates and formal relay timing. Ordinary expected times are soft only; medical/first-batch are hard.'}; (RES/'q3_C2_S008_dynamic_relay_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2)); print(pd.DataFrame(chain).to_string(index=False))
if __name__=='__main__':main()
