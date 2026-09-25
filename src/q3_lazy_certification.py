import ast,json,subprocess,sys,time,hashlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import pandas as pd,numpy as np
from scipy.optimize import milp,LinearConstraint,Bounds
from scipy.sparse import lil_matrix
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'
POOL=RES/'q3_candidate_pool_1p1db.csv'; CACHE=RES/'q3_true_edge_cache_1p1db.csv'; BL=RES/'q3_true_failed_edge_blacklist_1p1db.csv'
init=[('S002','O01','C'),('S012','S002','C')]
def route_edges(v):
 n=['O01']+str(v).split('>')+['O01']; return list(zip(n,n[1:]))
def run_edge(e):
 t=time.time()
 try:
  p=subprocess.run([sys.executable,str(ROOT/'src/q3_edge_batch_worker_1p1db.py'),json.dumps([list(e)])],cwd=ROOT,text=True,capture_output=True,timeout=180)
  if p.returncode==0:
   z=json.loads(p.stdout)[0]; return {'from_node':e[0],'to_node':e[1],'drone_type':e[2],'status':z.get('evaluation_status','COMPLETED'),'worst_joint_margin':z.get('worst_margin'),'communication_possible':z.get('communication_possible'),'runtime_seconds':time.time()-t,'pipeline_sha256':SHA}
 except subprocess.TimeoutExpired: pass
 return {'from_node':e[0],'to_node':e[1],'drone_type':e[2],'status':'TIMEOUT','worst_joint_margin':np.nan,'communication_possible':False,'runtime_seconds':time.time()-t,'pipeline_sha256':SHA}
def solve(c):
 boxes=sorted({b for z in c.box_ids.map(ast.literal_eval) for b in z}); A=lil_matrix((len(boxes),len(c))); ix={b:i for i,b in enumerate(boxes)}
 for j,z in enumerate(c.box_ids.map(ast.literal_eval)):
  for b in z:A[ix[b],j]=1
 res=milp(np.array([1+1e-6*float(a)+1e-9*float(b) for a,b in zip(c.soft_lateness,c.transport_energy)]),integrality=np.ones(len(c)),bounds=Bounds(0,1),constraints=LinearConstraint(A.tocsr(),1,1),options={'time_limit':120})
 if res.x is None:return None
 return c.iloc[np.flatnonzero(res.x>.5)].copy()
def main():
 if hashlib.sha256((ROOT/'src/q3_final_pipeline.py').read_bytes()).hexdigest()!=SHA:raise RuntimeError('SHA')
 c=pd.read_csv(POOL); cache=pd.read_csv(CACHE) if CACHE.exists() else pd.DataFrame(columns=['from_node','to_node','drone_type','status','worst_joint_margin','communication_possible','runtime_seconds','pipeline_sha256']); blacklist=set(init)
 for _,r in cache.iterrows():
  if str(r.status)=='COMPLETED' and not bool(r.communication_possible):blacklist.add((str(r.from_node),str(r.to_node),str(r.drone_type)))
 BL.write_text(pd.DataFrame([{'from_node':a,'to_node':b,'drone_type':t} for a,b,t in sorted(blacklist)]).to_csv(index=False),encoding='utf-8')
 itrows=[]
 for it in range(1,11):
  mask=[]
  for v in c.visit_order:
   mask.append(all((a,b,'C') not in blacklist for a,b in route_edges(v)))
  f=c.loc[mask].copy(); f.to_csv(RES/'q3_lazy_filtered_candidates.csv',index=False); sel=solve(f)
  if sel is None:
   itrows.append({'iteration':it,'candidate_count_after_filter':len(f),'selected_route_count':0,'selected_unique_edges':0,'cached_edges':len(cache),'new_edges_recomputed':0,'new_failed_edges':0,'all_selected_edges_certified':False,'status':'NO_EXACT_COVER'});break
  edges=sorted({(a,b,'C') for v in sel.visit_order for a,b in route_edges(v)}); unc=[e for e in edges if not ((cache.from_node==e[0])&(cache.to_node==e[1])&(cache.drone_type==e[2])).any()]; new=[]
  with ThreadPoolExecutor(max_workers=6) as ex:
   for fut in as_completed([ex.submit(run_edge,e) for e in unc]):
    z=fut.result(); new.append(z); cache=pd.concat([cache,pd.DataFrame([z])],ignore_index=True); cache.to_csv(CACHE,index=False,encoding='utf-8-sig')
  fails=[z for z in new if z['status']=='COMPLETED' and not z['communication_possible']]; blacklist.update((z['from_node'],z['to_node'],z['drone_type']) for z in fails); BL.write_text(pd.DataFrame([{'from_node':a,'to_node':b,'drone_type':t} for a,b,t in sorted(blacklist)]).to_csv(index=False),encoding='utf-8')
  done=all(((cache.from_node==a)&(cache.to_node==b)&(cache.drone_type=='C')&(cache.status=='COMPLETED')&(cache.communication_possible==True)).any() for a,b,_ in edges)
  itrows.append({'iteration':it,'candidate_count_after_filter':len(f),'selected_route_count':len(sel),'selected_unique_edges':len(edges),'cached_edges':len(cache),'new_edges_recomputed':len(new),'new_failed_edges':len(fails),'all_selected_edges_certified':done,'status':'CERTIFIED' if done else 'REPEAT'}); sel.to_csv(RES/f'q3_lazy_selected_routes_iter{it}.csv',index=False)
  if done:
   sel.to_csv(RES/'q3_lazy_final_selected_routes.csv',index=False); break
 pd.DataFrame(itrows).to_csv(RES/'q3_lazy_certification_iterations.csv',index=False,encoding='utf-8-sig'); print(pd.DataFrame(itrows).to_string(index=False)); print('blacklist',len(blacklist),'cache',len(cache))
if __name__=='__main__':main()
