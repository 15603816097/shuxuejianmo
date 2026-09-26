"""Merge and validate strict Q3 communication-graph shards."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import pandas as pd

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",default="results/q3_official_shards")
    ap.add_argument("--out",default="results/q3_official")
    a=ap.parse_args()
    src=Path(a.input); out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    fs=sorted(src.glob("strict_edges_*_shard*.csv"))
    if not fs: raise RuntimeError("no strict graph shards found")
    df=pd.concat([pd.read_csv(p) for p in fs],ignore_index=True)
    key=["from_node","to_node","drone_type"]
    dup=int(df.duplicated(key).sum())
    df=df.drop_duplicates(key,keep="last").sort_values(["drone_type","from_node","to_node"])
    errors=int(df.evaluation_status.astype(str).str.startswith("ERROR").sum())
    expected=3*16*15
    if len(df)!=expected or dup or errors:
        raise RuntimeError(f"strict graph incomplete: rows={len(df)} expected={expected} dup={dup} errors={errors}")
    df.to_csv(out/"strict_edge_graph_all_types.csv",index=False,encoding="utf-8-sig")
    summary={
      "records":int(len(df)),"expected_records":expected,"duplicate_records":dup,"errors":errors,
      "communication_possible":int(df.communication_possible.fillna(False).astype(bool).sum()),
      "by_type":{t:{
        "edges":int((df.drone_type==t).sum()),
        "possible":int(df.loc[df.drone_type==t,"communication_possible"].fillna(False).astype(bool).sum())
      } for t in ("A","B","C")},
      "scenario":"OFFICIAL_STRICT","delta_backhaul_db":0.0
    }
    (out/"strict_edge_graph_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
