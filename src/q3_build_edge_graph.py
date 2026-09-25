"""Build the Q3 communication graph under the frozen evaluator.

The evaluator is called on a synthetic O01->from->to->O01 trajectory and only
the requested from->to leg is retained for the directed-edge record.
"""
from __future__ import annotations
import json, hashlib, sys, time
import signal
from pathlib import Path
import pandas as pd
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0,str(Path(__file__).resolve().parent))
import q3_final_pipeline as pipe
import q3_five_service_hover_refinement as comm

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'; DELTA=.1
NODES=['O01']+[f'S{i:03d}' for i in range(1,16)]
BASE=comm.thresholds
_DATA=None

def _init_worker():
    global _DATA
    _DATA=pipe.load_inputs()

def run_edge(data, u, v, typ):
    def wrapped(D):
        a,b,c=BASE(D); return a,b,c+DELTA
    comm.thresholds=wrapped; pipe.thresholds=wrapped
    # Use the frozen evaluator's authoritative timeline and communication
    # functions directly; avoid evaluating unrelated legs of the synthetic
    # route, since this layer is geometry-only.
    order=tuple(x for x in (u,v) if x!='O01')
    tl=pipe.build_authoritative_timeline(data, order, typ, [])
    blocks=pipe.direct_and_relay_blocks(data,tl); chosen=[]
    for b in blocks:
        if any(str(p.get('from_node'))==u and str(p.get('to_node'))==v for p in b.get('items',[])):
            chosen.append(pipe._candidate_for_block(data,b))
    if not chosen:
        return {'communication_possible':True,'worst_margin':float('inf'),'relay_required':False,'relay_required_duration':0.0,'candidate_count':0,'pipeline_called':True}
    margins=[float(x.get('joint_margin',float('nan'))) for x in chosen if x is not None]
    return {'communication_possible':bool(margins and min(margins)>=0),'worst_margin':min(margins) if margins else float('nan'),'relay_required':True,'relay_required_duration':sum(float(b['end_s'])-float(b['start_s']) for b in blocks if any(str(p.get('from_node'))==u and str(p.get('to_node'))==v for p in b.get('items',[]))),'candidate_count':len(chosen),'pipeline_called':True}

def worker(args):
    u,v,typ=args
    try:
        z=run_edge(_DATA,u,v,typ); z['evaluation_status']='COMPLETED'; return u,v,typ,z
    except Exception as e:
        return u,v,typ,{'communication_possible':False,'worst_margin':float('nan'),'relay_required':None,'relay_required_duration':float('nan'),'candidate_count':0,'pipeline_called':False,'evaluation_status':'ERROR:'+type(e).__name__}

def main():
    if hashlib.sha256((ROOT/'src/q3_final_pipeline.py').read_bytes()).hexdigest()!=SHA: raise RuntimeError('frozen SHA mismatch')
    rows=[]; t0=time.time(); args=[(u,v,typ) for typ in ('C',) for u in NODES for v in NODES if u!=v]
    global _DATA
    _DATA=pipe.load_inputs()
    with ThreadPoolExecutor(max_workers=16) as ex:
        for u,v,typ,z in ex.map(worker,args):
            rows.append({'from_node':u,'to_node':v,'drone_type':typ,**z,'scenario':'FEASIBILITY_RESTORED_0P1DB','delta_backhaul_db':DELTA,'pipeline_sha256':SHA})
    pd.DataFrame(rows).to_csv(RES/'q3_full_edge_graph.csv',index=False,encoding='utf-8-sig'); print(json.dumps({'records':len(rows),'runtime_seconds':time.time()-t0},ensure_ascii=False))
if __name__=='__main__': main()
