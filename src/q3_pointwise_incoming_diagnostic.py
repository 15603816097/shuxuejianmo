import sys,math,json,hashlib
from pathlib import Path
import numpy as np,pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
import q3_final_pipeline as p
from q3_S008_hover_refinement_v2 import thresholds,ground
from q3_semantics_core import xyz,link_obstructed
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'; DELTA=.1
D=p.load_inputs(); ta,tb=thresholds(D); tb+=DELTA
freq=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')]); c=D['center']; gh=float(c['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]);
lons=np.asarray(D['dem']['longitude']).ravel(); lats=np.asarray(D['dem']['latitude']).ravel()
def margin(a,b,ha,hb,thr):
 d=float(np.linalg.norm(xyz(D,a,ha)-xyz(D,b,hb))); ol=obs if link_obstructed(D,a,b,ha,hb) else 0.; return thr-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+ol)
def interp(q,t):
 a,b=q['a'],q['b']; u=0 if q['end_s']==q['start_s'] else max(0,min(1,(t-q['start_s'])/(q['end_s']-q['start_s']))); ep={'经度（°）':float(a['经度（°）'])+u*(float(b['经度（°）'])-float(a['经度（°）'])),'纬度（°）':float(a['纬度（°）'])+u*(float(b['纬度（°）'])-float(a['纬度（°）']))}; h=float(q['ha'])+u*(float(q['hb'])-float(q['ha'])); return ep,h
def candidates():
 out=[]
 for lo in np.linspace(float(lons.min()),float(lons.max()),30):
  for la in np.linspace(float(lats.min()),float(lats.max()),30):
   q={'经度（°）':float(lo),'纬度（°）':float(la)}; g=ground(D,q)
   if not np.isfinite(g): continue
   for agl in np.linspace(50,300,11):
    h=g+agl; out.append({'p':q,'h':h,'backhaul':margin(q,c,h,gh,tb)})
 return sorted(out,key=lambda x:x['backhaul'],reverse=True)[:10]
H=candidates(); nodes=['O01']+[f'S{i:03d}' for i in range(1,16)]; rows=[]
for target in ['S002','S003','S012']:
 for src in nodes:
  if src==target: continue
  order=tuple(x for x in (src,target) if x!='O01'); tl=p.build_authoritative_timeline(D,order,'C',[]); blocks=p.direct_and_relay_blocks(D,tl); req=[b for b in blocks if any(str(x.get('to_node'))==target and str(x.get('from_node'))==src for x in b.get('items',[]))]; samples=[]
  for b in req:
   for q in b['items']:
    for t in np.linspace(float(q['start_s']),float(q['end_s']),max(2,int((q['end_s']-q['start_s'])/5)+1)):
     ep,h=interp(q,t); best=-1e9; ba=bb=-1e9; bp=None
     for z in H:
      a=margin(ep,z['p'],h,z['h'],ta); j=min(a,z['backhaul'])
      if j>best: best=j;ba=a;bb=z['backhaul'];bp=z
     samples.append((t,best,ba,bb,bp))
  if samples:
   z=min(samples,key=lambda x:x[1]); rows.append({'from_node':src,'to_node':target,'drone_type':'C','pointwise_sample_count':len(samples),'nonpositive_sample_count':sum(x[1]<=0 for x in samples),'worst_best_joint_margin':z[1],'best_access_margin_at_binding':z[2],'best_backhaul_margin_at_binding':z[3],'binding_time':z[0],'best_hover_x':z[4]['p']['经度（°）'],'best_hover_y':z[4]['p']['纬度（°）'],'best_hover_altitude':z[4]['h'],'pointwise_edge_possible':bool(all(x[1]>0 for x in samples))})
  else: rows.append({'from_node':src,'to_node':target,'drone_type':'C','pointwise_sample_count':0,'nonpositive_sample_count':0,'worst_best_joint_margin':float('inf'),'pointwise_edge_possible':True})
out=pd.DataFrame(rows); out.to_csv(RES/'q3_S002_S003_S012_pointwise_incoming.csv',index=False,encoding='utf-8-sig'); summary={}
for s in ['S002','S003','S012']:
 z=out[out.to_node==s]; best=float(z.worst_best_joint_margin.max()); d=0.0
 while d<20 and float((z.worst_best_joint_margin+d).max())<=0: d+=.001
 summary[s]={'incoming_total':len(z),'pointwise_feasible_incoming':int(z.pointwise_edge_possible.sum()),'best_incoming':z.loc[z.worst_best_joint_margin.idxmax(),'from_node'],'worst_best_joint_margin':best,'min_delta_db':round(d,3)}
(RES/'q3_S002_S003_S012_pointwise_summary.json').write_text(json.dumps({'summary':summary,'global_restoration_delta_db':max(x['min_delta_db'] for x in summary.values()),'pipeline_sha256':SHA,'delta_backhaul_db':DELTA},ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2))
