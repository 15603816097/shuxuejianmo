from __future__ import annotations
import json, math, sys
from pathlib import Path
import pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
from minimal_pipeline import load_inputs
from q3_C3_S008_three_point_screening import make_timeline,get_blocks,best_hover
from q3_official_semantics import relay_leg_time_energy,relay_component_ledger
from minimal_pipeline import line_geometry,node_map
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-6

def main():
 D=load_inputs(); df=pd.read_csv(RES/'q3_C3_S008_three_point_screening.csv'); bm={x['货箱编号']:x for x in D['boxes']}
 usable=df[df.reserve_ok==True]; fail=df[df.reserve_ok==False]; picks=pd.concat([df.nlargest(1,'worst_block_joint_margin'),fail.head(1),usable.head(1)]).drop_duplicates(subset=['route','box_set','drone_type'])
 out=[]
 for _,r in picks.iterrows():
  ids=json.loads(r.box_set); order=tuple(r.visit_order.split('->')); typ=r.drone_type; traj,delivery,makespan=make_timeline(D,order,typ,ids); bs,G,ta,tb,freq,obs=get_blocks(D,traj); best=[best_hover(D,b,G,ta,tb,freq,obs) for b in bs]; compfree={f'C{i:02d}':0.0 for i in range(1,7)}; comp_rows=[]; relay_e=0.; component_ok=True
  for i,(z,b) in enumerate(zip(best,bs)):
   if z is None: component_ok=False; continue
   rp={'经度（°）':z['x'],'纬度（°）':z['y']}; dist,peak,*_=line_geometry(D['center'],rp,D['dem']); tout,eout=relay_leg_time_energy(D,dist,peak,float(D['center']['海拔（m）']),z['h']); tback,eback=relay_leg_time_energy(D,dist,peak,z['h'],float(D['center']['海拔（m）'])); en=eout+eback+(float(D['relay']['hover_power_kw'])+float(D['relay']['comm_power_kw']))*(b['end']-b['start'])/3600.; relay_e+=en; cid=min(compfree,key=compfree.get); ps=b['start']-float(D['relay']['prep_s'])-tout-float(D['relay']['link_s']); rend=b['end']+tback; c=relay_component_ledger(D,cid,ps,rend,en,compfree[cid]); compfree[cid]=c['available_again_s']; comp_rows.append(c); component_ok &= c['reserve_ok'] and c['resource_available_ok']
  out.append({'route':r.route,'drone_type':typ,'box_set':r.box_set,'blocks':len(bs),'transport_energy_kwh':float(r.transport_energy),'relay_energy_kwh_formal':relay_e,'total_energy_kwh_formal':float(r.transport_energy)+relay_e,'transport_reserve_ok':bool(r.reserve_ok),'component_ledger_ok':component_ok,'communication_geometry_pass':bool(r.communication_geometry_pass),'route_feasible':bool(r.route_feasible)})
 pd.DataFrame(out).to_csv(RES/'q3_C3_representative_energy_regression.csv',index=False,encoding='utf-8-sig')
 rows=[]
 for _,r in df.iterrows():
  dr=D['drones'][r.drone_type]; usable_energy=(1-dr.reserve)*dr.energy; ids=json.loads(r.box_set); payload=sum(float(bm[i]['单箱质量（kg）']) for i in ids); slack=usable_energy-float(r.transport_energy); reason='PASS' if slack>=-TOL else ('route too long / climb energy' if float(r.transport_energy)>usable_energy*1.15 else ('payload-dependent range/energy' if payload>0.6*dr.max_mass else 'return reserve constraint'))
  rows.append({'route':r.route,'drone_type':r.drone_type,'payload_kg':payload,'transport_energy':r.transport_energy,'usable_energy':usable_energy,'reserve_limit':usable_energy,'energy_slack':slack,'dominant_reason':reason})
 pd.DataFrame(rows).to_csv(RES/'q3_C3_reserve_failure_audit.csv',index=False,encoding='utf-8-sig')
 print(pd.DataFrame(out).to_string(index=False)); print('reserve fail',int((~df.reserve_ok).sum()))
if __name__=='__main__':main()
