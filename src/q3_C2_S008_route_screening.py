from __future__ import annotations
import math,json
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,q1,q2,node_map,terrain_profile,xy,line_geometry,energy_leg
from q3_five_service_hover_refinement import thresholds
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'

def xyz(D,p,h):
 lat0=float(D['center']['纬度（°）']); q=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([q[0],q[1],float(h)],float)
def ground(D,p):
 v=[z for _,z in terrain_profile(p,p,D['dem']) if np.isfinite(z)]; return max(v) if v else np.nan
def comm_segment(D,a,b,ha,hb):
 td,ta,tb=thresholds(D); c=D['center']; gh=float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); f0=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); A=xyz(D,a,ha); B=xyz(D,b,hb); G=xyz(D,c,gh); d=max(np.linalg.norm(A-G),np.linalg.norm(B-G)); dm=td-(32.45+20*math.log10(f0)+20*math.log10(max(d/1000,1e-12))+obs)
 if dm>=0:return {'required':False,'access':float('inf'),'backhaul':float('inf'),'joint':float('inf'),'ok':True}
 best=None
 for frac in np.linspace(.05,.95,19):
  rp={'经度（°）':float(a['经度（°）'])+frac*(float(b['经度（°）'])-float(a['经度（°）'])),'纬度（°）':float(a['纬度（°）'])+frac*(float(b['纬度（°）'])-float(a['纬度（°）']))}; rg=ground(D,rp)
  if not np.isfinite(rg):continue
  for off in [50.,100.,150.,200.,250.,300.]:
   rh=rg+off; R=xyz(D,rp,rh); da=max(np.linalg.norm(A-R),np.linalg.norm(B-R)); db=np.linalg.norm(R-G); ma=ta-(32.45+20*math.log10(f0)+20*math.log10(max(da/1000,1e-12))+obs); mb=tb-(32.45+20*math.log10(f0)+20*math.log10(max(db/1000,1e-12))+obs); z={'access':float(ma),'backhaul':float(mb),'joint':float(min(ma,mb)),'candidate':(rp['经度（°）'],rp['纬度（°）'],rh)}
   if best is None or z['joint']>best['joint']:best=z
 return {'required':True,'access':best['access'],'backhaul':best['backhaul'],'joint':best['joint'],'ok':best['joint']>0}

def main():
 D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); nm=node_map(D); services=sorted({x['服务区编号'] for x in D['boxes'] if x['服务区编号']!='S008'}); d=D['drones']['C']; rows=[]
 for s in services:
  for order in [(s,'S008'),('S008',s)]:
   box=[x for x in D['boxes'] if x['服务区编号'] in order]; mass=sum(float(x['单箱质量（kg）']) for x in box); vol=sum(float(x['单箱体积（m³）']) for x in box); payload_ok=mass<=d.max_mass and vol<=d.volume; nodes=['O01']+list(order)+['O01']; tcur=float(d.prep+d.load_box*len(box)); energy=0.; comm=[]; delivery={}; cur='O01'
   for j,nxt in enumerate(nodes[1:]):
    a=nm[cur]; bnode=nm[nxt]; dist,peak,*_=line_geometry(a,bnode,D['dem']); remaining_mass=sum(float(x['单箱质量（kg）']) for x in D['boxes'] if x['服务区编号'] in order[j:]) if j<2 else 0.; h0=float(a['海拔（m）']); h1=float(bnode['海拔（m）'])+(30 if nxt!='O01' else 0); e,ft=energy_leg(d,dist,peak,h0,h1,remaining_mass); energy+=e; tcur+=ft
    
    # delivery times and communication on outbound segments; rebuild elapsed
   tcur=float(d.prep+d.load_box*len(box)); energy=0.; comm=[]; delivery={}; cur='O01'
   for j,nxt in enumerate(nodes[1:]):
    a=nm[cur]; bnode=nm[nxt]; dist,peak,*_=line_geometry(a,bnode,D['dem']); remaining_mass=sum(float(x['单箱质量（kg）']) for x in D['boxes'] if x['服务区编号'] in order[j:]) if j<2 else 0.; h0=float(a['海拔（m）']); h1=float(bnode['海拔（m）'])+(30 if nxt!='O01' else 0); e,ft=energy_leg(d,dist,peak,h0,h1,remaining_mass); energy+=e; tcur+=ft
    
    # service handoff at each visited point
    # recompute with handoff and delivery bookkeeping
   tcur=float(d.prep+d.load_box*len(box)); energy=0.; comm=[]; delivery={}; cur='O01'
   for j,nxt in enumerate(nodes[1:]):
    a=nm[cur]; bnode=nm[nxt]; dist,peak,*_=line_geometry(a,bnode,D['dem']); remaining_mass=sum(float(x['单箱质量（kg）']) for x in D['boxes'] if x['服务区编号'] in order[j:]) if j<2 else 0.; h0=float(a['海拔（m）']); h1=float(bnode['海拔（m）'])+(30 if nxt!='O01' else 0); e,ft=energy_leg(d,dist,peak,h0,h1,remaining_mass); energy+=e; tcur+=ft
    if nxt!='O01':
     hand=d.handoff+d.handoff_box*sum(1 for x in box if x['服务区编号']==nxt); tcur+=hand; delivery[nxt]=tcur; comm.append(comm_segment(D,a,bnode,h0,h1))
    cur=nxt
   reserve_ok=energy <= (1-d.reserve)*d.energy+1e-9
   hard_ok=True
   for x in box:
    if x['服务区编号'] in delivery:
     deadline=min(float(x['首批截止时间（s）']),float(x['期望送达时间（s）'])) if x['是否首批保障']=='是' else (float(x['期望送达时间（s）']))
     hard_ok &= delivery[x['服务区编号']]<=deadline+1e-9
   comm_ok=all(z['ok'] for z in comm); rows.append({'route':'O01->'+order[0]+'->'+order[1]+'->O01','visit_order':'->'.join(order),'S008_delivery_time_s':delivery.get('S008',np.nan),'hard_deadline_ok':bool(hard_ok),'route_payload_hard_ok':bool(hard_ok and payload_ok),'payload_ok':payload_ok,'mass_kg':mass,'volume_m3':vol,'relay_required_interval_count':sum(z['required'] for z in comm),'best_access_margin':min([z['access'] for z in comm],default=float('inf')),'best_backhaul_margin':min([z['backhaul'] for z in comm],default=float('inf')),'best_joint_margin':min([z['joint'] for z in comm],default=float('inf')),'communication_feasible':comm_ok,'transport_reserve_ok':reserve_ok,'energy_kwh':energy,'route_time_s':tcur})
 out=pd.DataFrame(rows); out.to_csv(RES/'q3_C2_S008_route_screening.csv',index=False,encoding='utf-8-sig'); print(out.to_string(index=False)); print('tests',len(out),'hard',int(out.hard_deadline_ok.sum()),'comm',int(out.communication_feasible.sum()))
if __name__=='__main__':main()
