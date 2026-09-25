import json,subprocess,sys,time,hashlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'; outp=RES/'q3_K23_selected_edges_recomputed_partial.csv'
u=pd.read_csv(RES/'q3_K23_selected_unique_edges.csv'); edges=[(str(r.from_node),str(r.to_node),'C') for _,r in u.iterrows()]
if outp.exists():
 old=pd.read_csv(outp); done=set(zip(old.from_node,old.to_node,old.drone_type)); edges=[e for e in edges if e not in done]
else: old=pd.DataFrame()
def run(e):
 t=time.time()
 try:
  p=subprocess.run([sys.executable,str(ROOT/'src/q3_edge_batch_worker_1p1db.py'),json.dumps([list(e)])],cwd=ROOT,text=True,capture_output=True,timeout=180)
  if p.returncode==0:
   z=json.loads(p.stdout)[0]; return {'from_node':e[0],'to_node':e[1],'drone_type':e[2],'status':z.get('evaluation_status','COMPLETED'),'runtime_seconds':time.time()-t,'worst_joint_margin':z.get('worst_margin'),'communication_possible':z.get('communication_possible'),'pipeline_sha256':SHA}
  return {'from_node':e[0],'to_node':e[1],'drone_type':e[2],'status':'ERROR','runtime_seconds':time.time()-t,'worst_joint_margin':float('nan'),'communication_possible':False,'pipeline_sha256':SHA}
 except subprocess.TimeoutExpired:
  return {'from_node':e[0],'to_node':e[1],'drone_type':e[2],'status':'TIMEOUT','runtime_seconds':time.time()-t,'worst_joint_margin':float('nan'),'communication_possible':False,'pipeline_sha256':SHA}
rows=[]
with ThreadPoolExecutor(max_workers=6) as ex:
 fs=[ex.submit(run,e) for e in edges]
 for f in as_completed(fs):
  z=f.result(); rows.append(z); pd.DataFrame(rows).to_csv('/tmp/q3_partial_new.csv',index=False)
  print(z,flush=True)
if len(rows):
 allr=pd.concat([old,pd.DataFrame(rows)],ignore_index=True) if len(old) else pd.DataFrame(rows); allr.to_csv(outp,index=False,encoding='utf-8-sig')
print('done',len(rows),'remaining',len(edges))
