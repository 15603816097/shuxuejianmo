"""Strict Q3 directed communication graph for all transport drone types.

Uses the frozen Q3 evaluator with delta=0 dB (official strict scenario).
This is a geometry pre-screen only; selected routes are certified again at
route level and global relay/component scheduling is a later gate.
"""
from __future__ import annotations
import hashlib, json, time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import pandas as pd

import q3_build_edge_graph as base

ROOT=Path(__file__).resolve().parents[1]
RES=ROOT/"results"/"q3_official"
RES.mkdir(parents=True,exist_ok=True)
SHA="5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb"
NODES=["O01"]+[f"S{i:03d}" for i in range(1,16)]

def main():
    got=hashlib.sha256((ROOT/"src/q3_final_pipeline.py").read_bytes()).hexdigest()
    if got!=SHA:
        raise RuntimeError(f"frozen Q3 SHA mismatch: {got}")
    base.DELTA=0.0
    data=base.pipe.load_inputs()
    base._DATA=data
    args=[(u,v,t) for t in ("A","B","C") for u in NODES for v in NODES if u!=v]
    rows=[]
    t0=time.time()
    def one(a):
        u,v,t=a
        try:
            z=base.run_edge(data,u,v,t)
            status="COMPLETED"
        except Exception as e:
            z={"communication_possible":False,"worst_margin":float("nan"),
               "relay_required":None,"relay_required_duration":float("nan"),
               "candidate_count":0,"pipeline_called":False}
            status="ERROR:"+type(e).__name__
        return {"from_node":u,"to_node":v,"drone_type":t,**z,
                "evaluation_status":status,"scenario":"OFFICIAL_STRICT",
                "delta_backhaul_db":0.0,"pipeline_sha256":SHA}
    with ThreadPoolExecutor(max_workers=12) as ex:
        for i,z in enumerate(ex.map(one,args),1):
            rows.append(z)
            if i%60==0:
                print(f"strict edges {i}/{len(args)}",flush=True)
    df=pd.DataFrame(rows).sort_values(["drone_type","from_node","to_node"])
    df.to_csv(RES/"strict_edge_graph_all_types.csv",index=False,encoding="utf-8-sig")
    summary={
        "records":int(len(df)),
        "completed":int((df.evaluation_status=="COMPLETED").sum()),
        "communication_possible":int(df.communication_possible.fillna(False).astype(bool).sum()),
        "by_type":{
            t:{
                "edges":int((df.drone_type==t).sum()),
                "possible":int(df.loc[df.drone_type==t,"communication_possible"].fillna(False).astype(bool).sum())
            } for t in ("A","B","C")
        },
        "scenario":"OFFICIAL_STRICT",
        "delta_backhaul_db":0.0,
        "pipeline_sha256":SHA,
        "runtime_s":time.time()-t0,
        "note":"Edge graph is a pre-screen. Route-level and global resource certification remain required."
    }
    (RES/"strict_edge_graph_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
