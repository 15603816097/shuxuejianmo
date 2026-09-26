"""One shard of the official-strict Q3 directed communication graph."""
from __future__ import annotations
import argparse, hashlib, json, time
from pathlib import Path
import pandas as pd
import q3_build_edge_graph as base

ROOT=Path(__file__).resolve().parents[1]
RES=ROOT/"results"/"q3_official_shards"
RES.mkdir(parents=True,exist_ok=True)
SHA="5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb"
NODES=["O01"]+[f"S{i:03d}" for i in range(1,16)]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--type",required=True,choices=["A","B","C"])
    ap.add_argument("--shard",required=True,type=int)
    ap.add_argument("--shards",type=int,default=4)
    a=ap.parse_args()
    if not 0<=a.shard<a.shards: raise ValueError("bad shard")
    got=hashlib.sha256((ROOT/"src/q3_final_pipeline.py").read_bytes()).hexdigest()
    if got!=SHA: raise RuntimeError(f"frozen Q3 SHA mismatch: {got}")
    base.DELTA=0.0
    data=base.pipe.load_inputs(); base._DATA=data
    pairs=[(u,v,a.type) for u in NODES for v in NODES if u!=v]
    work=[z for i,z in enumerate(pairs) if i%a.shards==a.shard]
    rows=[]; t0=time.time()
    for i,(u,v,t) in enumerate(work,1):
        try:
            z=base.run_edge(data,u,v,t); status="COMPLETED"
        except Exception as e:
            z={"communication_possible":False,"worst_margin":float("nan"),
               "relay_required":None,"relay_required_duration":float("nan"),
               "candidate_count":0,"pipeline_called":False}
            status="ERROR:"+type(e).__name__
        rows.append({"from_node":u,"to_node":v,"drone_type":t,**z,
                     "evaluation_status":status,"scenario":"OFFICIAL_STRICT",
                     "delta_backhaul_db":0.0,"pipeline_sha256":SHA})
        if i%10==0: print(f"{a.type} shard {a.shard}: {i}/{len(work)}",flush=True)
    out=RES/f"strict_edges_{a.type}_shard{a.shard}.csv"
    pd.DataFrame(rows).to_csv(out,index=False,encoding="utf-8-sig")
    print(json.dumps({"type":a.type,"shard":a.shard,"records":len(rows),
                      "errors":sum(str(x["evaluation_status"]).startswith("ERROR") for x in rows),
                      "runtime_s":time.time()-t0},ensure_ascii=False))

if __name__=="__main__":
    main()
