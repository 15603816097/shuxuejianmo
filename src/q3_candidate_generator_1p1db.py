"""Q3 candidate generation: communication skeletons first, box packing second."""
from __future__ import annotations
import itertools, hashlib, json, sys
from pathlib import Path
import pandas as pd
sys.path.insert(0,str(Path(__file__).resolve().parent))
import q3_final_pipeline as pipe
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'; DELTA=1.1

def main():
 if hashlib.sha256((ROOT/'src/q3_final_pipeline.py').read_bytes()).hexdigest()!=SHA: raise RuntimeError('frozen SHA mismatch')
 data=pipe.load_inputs(); graph=pd.read_csv(RES/'q3_full_edge_graph_1p1db.csv'); typ='C'; feasible={(str(r.from_node),str(r.to_node)):bool(r.communication_possible) for _,r in graph[graph.drone_type==typ].iterrows()}; services=[f'S{i:03d}' for i in range(1,16)]
 def edge(u,v): return feasible.get((u,v),False)
 def ok(seq):
  nodes=('O01',)+tuple(seq)+('O01',); return all(edge(a,b) for a,b in zip(nodes,nodes[1:]))
 skeletons=[seq for n in (1,2,3) for seq in itertools.permutations(services,n) if ok(seq)]; sk=[]
 for i,seq in enumerate(skeletons,1):
  mm=[]
  for a,b in zip(('O01',)+seq,seq+('O01',)):
   z=graph[(graph.from_node==a)&(graph.to_node==b)&(graph.drone_type==typ)]
   if len(z): mm.append(float(z.iloc[0].worst_margin))
  sk.append({'skeleton_id':f'SK-{i:05d}','service_sequence':'>'.join(seq),'edge_count':len(seq)+1,'communication_margin':min(mm) if mm else float('nan'),'drone_type':typ})
 pd.DataFrame(sk).to_csv(RES/'q3_route_skeletons_1p1db.csv',index=False,encoding='utf-8-sig')
 bm={x['货箱编号']:x for x in data['boxes']}; records=[]; seen=set()
 for si,seq in enumerate(skeletons,1):
  ids=[x['货箱编号'] for x in data['boxes'] if x['服务区编号'] in seq]; assignments=[(b,) for b in ids]
  for s in seq: assignments += list(itertools.combinations([b for b in ids if bm[b]['服务区编号']==s],2))
  if len(ids)>1: assignments.append(tuple(ids))
  for boxset in assignments:
   key=(seq,tuple(sorted(boxset)),typ)
   if key in seen: continue
   seen.add(key); tl=pipe.build_authoritative_timeline(data,seq,typ,list(boxset)); dl=pipe.deadline_ledger(data,tl); en=pipe.transport_energy_ledger(data,tl); d=data['drones'][typ]; mass=sum(float(bm[b]['单箱质量（kg）']) for b in boxset); vol=sum(float(bm[b]['单箱体积（m³）']) for b in boxset); energy=sum(float(x['energy_kwh']) for x in en); transport=mass<=float(d.max_mass)+pipe.TOL and vol<=float(d.volume)+pipe.TOL and energy<=(1-float(d.reserve))*float(d.energy)+pipe.TOL; hard=all(x['pass'] for x in dl if x['hard'])
   if transport and hard:
    soft=sum(max(0,float(x['delivery_time_s'])-float(x['deadline_s'])) for x in dl if not x['hard']); records.append({'candidate_id':f'Q3V2-{len(records)+1:06d}','route':'O01->'+'->'.join(seq)+'->O01','visit_order':'>'.join(seq),'box_ids':json.dumps(list(boxset),ensure_ascii=False),'drone_type':typ,'transport_feasible':True,'hard_deadline_ok':True,'communication_edge_feasible':True,'soft_lateness':soft,'transport_energy':energy,'route_feasible_level':'ROUTE_LEVEL_CANDIDATE','skeleton_id':f'SK-{si:05d}'})
 out=pd.DataFrame(records).drop_duplicates(subset=['route','box_ids','drone_type']); out.to_csv(RES/'q3_candidate_pool_1p1db.csv',index=False,encoding='utf-8-sig'); all_boxes=sorted(bm); counts={b:0 for b in all_boxes}
 for z in out.box_ids:
  for b in json.loads(z): counts[b]+=1
 pd.DataFrame([{'box_id':b,'candidate_count':n,'covered':n>0,'reason':'HAS_CANDIDATE' if n else 'NO_PHYSICALLY_VALID_GENERATED_CANDIDATE'} for b,n in counts.items()]).to_csv(RES/'q3_candidate_coverage_1p1db.csv',index=False,encoding='utf-8-sig')
 summary={'candidate_count':len(out),'single':sum('>' not in x for x in out.visit_order),'double':sum(x.count('>')==1 for x in out.visit_order),'triple':sum(x.count('>')==2 for x in out.visit_order),'route_skeleton_count':len(skeletons),'covered_boxes':int(sum(n>0 for n in counts.values())),'uncovered_boxes':int(sum(n==0 for n in counts.values())),'min_candidates_per_box':int(min(counts.values())),'median_candidates_per_box':float(pd.Series(list(counts.values())).median()),'max_candidates_per_box':int(max(counts.values())),'frozen_sha256':SHA,'delta_backhaul_db':DELTA}; (RES/'q3_candidate_pool_1p1db_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
 diag=[]
 singleton_counts={}
 for z in out[out.box_ids.map(lambda q: len(json.loads(q))==1)].box_ids:
  singleton_counts[json.loads(z)[0]]=singleton_counts.get(json.loads(z)[0],0)+1
 for b in all_boxes:
  x=bm[b]; s=str(x['服务区编号']); ins=[u for u in ['O01']+services if u!=s and edge(u,s)]; outs=[v for v in ['O01']+services if v!=s and edge(s,v)]; cycles=ok((s,)) or any(ok((a,s,b2)) for a,b2 in itertools.permutations(services,2) if s not in (a,b2)); reason='COVERED' if counts[b] else ('NO_COMM_IN_EDGE' if not ins else 'NO_COMM_OUT_EDGE' if not outs else 'NO_COMMUNICATION_CYCLE' if not cycles else 'CANDIDATE_GENERATOR_MISSING'); hard_type='MEDICAL_HARD' if x['物资类型']=='医疗物资' else ('FIRST_BATCH_HARD' if x['是否首批保障']=='是' else 'ORDINARY_SOFT'); deadline=x['期望送达时间（s）'] if hard_type=='MEDICAL_HARD' else (x['首批截止时间（s）'] if hard_type=='FIRST_BATCH_HARD' else x['期望送达时间（s）']); diag.append({'box_id':b,'service_id':s,'mass':x['单箱质量（kg）'],'volume':x['单箱体积（m³）'],'category':x['物资类型'],'hard_deadline_type':hard_type,'deadline':deadline,'communication_feasible_in_count':len(ins),'communication_feasible_out_count':len(outs),'communication_cycle':cycles,'singleton_candidate_count':singleton_counts.get(b,0),'singleton_present_in_pool':singleton_counts.get(b,0)>0,'candidate_count':counts[b],'failure_reason':reason})
 pd.DataFrame(diag).to_csv(RES/'q3_uncovered_box_diagnostic_1p1db.csv',index=False,encoding='utf-8-sig'); print(json.dumps(summary,ensure_ascii=False))
if __name__=='__main__': main()
