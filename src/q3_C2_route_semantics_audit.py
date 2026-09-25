from __future__ import annotations
import json,math
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs,node_map,line_geometry,energy_leg
from q3_C2_S008_route_screening_v2 import multi_traj
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'

def main():
 D=load_inputs(); nm=node_map(D); order=('S011','S008'); typ='C'; boxes=[x for x in D['boxes'] if x['服务区编号'] in order]; d=D['drones'][typ]; traj=multi_traj(D,order,typ); rows=[]; cur='O01'; rem=sum(float(x['单箱质量（kg）']) for x in boxes); delivery={}; segno=0; prev_end=None
 # derive segment spans from trajectory by phase and node transitions via route nodes
 nodes=['O01']+list(order)+['O01']; tcursor=float(d.prep+d.load_box*len(boxes));
 for j,nxt in enumerate(nodes[1:]):
  a=nm[cur]; b=nm[nxt]; dist,peak,*_=line_geometry(a,b,D['dem']); end_h=float(b['海拔（m）'])+(30 if nxt!='O01' else 0); start_h=float(a['海拔（m）'])+(30 if cur!='O01' else 0); e,ft=energy_leg(d,dist,peak,start_h,end_h,rem); start=tcursor; tcursor+=ft; hand=0.
  if nxt!='O01': hand=float(d.handoff)+float(d.handoff_box)*sum(1 for x in boxes if x['服务区编号']==nxt); tcursor+=hand; delivery[nxt]=tcursor; delivered=sum(float(x['单箱质量（kg）']) for x in boxes if x['服务区编号']==nxt); rows.append({'route':'O01->S011->S008->O01','segment_id':j+1,'segment':f'{cur}->{nxt}','start_time_s':start,'flight_end_s':start+ft,'handoff_start_s':start+ft,'handoff_end_s':tcursor,'start_altitude_m':start_h,'end_altitude_m':end_h,'payload_before_kg':rem,'payload_delivered_kg':delivered,'payload_after_kg':rem-delivered,'flight_time_s':ft,'transport_energy_kwh':e,'handoff_time_s':hand,'timeline_source':'multi_traj/energy_leg unified'}); rem-=delivered
  else: rows.append({'route':'O01->S011->S008->O01','segment_id':j+1,'segment':f'{cur}->{nxt}','start_time_s':start,'flight_end_s':start+ft,'handoff_start_s':None,'handoff_end_s':None,'start_altitude_m':start_h,'end_altitude_m':end_h,'payload_before_kg':rem,'payload_delivered_kg':0,'payload_after_kg':rem,'flight_time_s':ft,'transport_energy_kwh':e,'handoff_time_s':0,'timeline_source':'multi_traj/energy_leg unified'})
  cur=nxt
 # per box deadline semantics on deliveries
 for x in boxes:
  dl_source='MEDICAL_EXPECTED' if x['物资类型']=='医疗物资' else ('FIRST_BATCH_DEADLINE' if x['是否首批保障']=='是' else 'EXPECTED_SOFT'); hard=dl_source!='EXPECTED_SOFT'; deadline=float(x['期望送达时间（s）']) if dl_source=='MEDICAL_EXPECTED' else (float(x['首批截止时间（s）']) if hard else float(x['期望送达时间（s）'])); arr=delivery[x['服务区编号']]; rows.append({'route':'O01->S011->S008->O01','segment_id':f'BOX:{x["货箱编号"]}','segment':x['服务区编号'],'delivery_time_s':arr,'box_id':x['货箱编号'],'deadline_source':dl_source,'deadline_s':deadline,'hard_constraint':hard,'hard_ok':(arr<=deadline if hard else True),'soft_lateness_s':max(0,arr-deadline) if not hard else 0,'payload_before_kg':None,'payload_delivered_kg':None,'payload_after_kg':None,'flight_time_s':None,'transport_energy_kwh':None,'timeline_source':'delivery at service handoff'})
 pd.DataFrame(rows).to_csv(RES/'q3_C2_route_semantics_final_audit.csv',index=False,encoding='utf-8-sig'); print(pd.DataFrame(rows).to_string(index=False)); print('delivery',delivery,'total transport energy',sum(float(r.get('transport_energy_kwh') or 0) for r in rows if str(r.get('segment_id','')).isdigit()))
if __name__=='__main__':main()
