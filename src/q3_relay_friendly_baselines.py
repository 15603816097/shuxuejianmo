"""Relay-friendly Q3 transport baselines.

Start from the exact Q1 single-service partition, then strategically split
blind-area batches so hard-deadline boxes and ordinary boxes can be clustered
in time. The intent is to make the four geographic relay clusters schedulable
with four relay sorties, rather than merely increasing K by arbitrary balanced
splits.
"""
from __future__ import annotations
import argparse, ast, json, math
from pathlib import Path
import pandas as pd

import q3_final_pipeline as pipe
from minimal_pipeline import load_inputs, q1, route_stats

CLUSTER = {
    "S003":"G1","S005":"G1","S007":"G1","S015":"G1",
    "S004":"G2","S008":"G2","S009":"G2",
    "S002":"G3","S012":"G3","S013":"G3",
    "S010":"G4","S014":"G4",
}
DIRECT={"S001","S006","S011"}

def ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try:return [str(x) for x in json.loads(str(v))]
    except Exception:return [str(x) for x in ast.literal_eval(str(v))]

def bm(data): return {str(x["货箱编号"]):x for x in data["boxes"]}

def is_hard(b):
    return str(b.get("物资类型",""))=="医疗物资" or str(b.get("是否首批保障",""))=="是"

def deadline_key(b):
    ds=[]
    if str(b.get("物资类型",""))=="医疗物资": ds.append(float(b["期望送达时间（s）"]))
    if str(b.get("是否首批保障",""))=="是": ds.append(float(b["首批截止时间（s）"]))
    return min(ds) if ds else float(b.get("期望送达时间（s）",14000.0))

def safe_split(data,row,left,right):
    box= bm(data); typ=str(row["type"]); service=str(row["service"])
    out=[]
    for chunk in (left,right):
        if not chunk: return None
        st=route_stats(data,typ,service,[box[x] for x in chunk])
        if not st["safe"]: return None
        nr=dict(row); nr.update(st); nr["box_ids"]=list(chunk); out.append(nr)
    return out

def propose_split(data,row):
    box=bm(data); z=ids(row["box_ids"])
    hard=[x for x in z if is_hard(box[x])]
    soft=[x for x in z if not is_hard(box[x])]
    # Primary relay-friendly split: hard wave vs ordinary wave.
    if hard and soft:
        p=safe_split(data,row,sorted(hard,key=lambda x:deadline_key(box[x])),
                     sorted(soft,key=lambda x:deadline_key(box[x])))
        if p is not None: return p,"hard_vs_soft"
    # Secondary: deadline-ordered balanced split.
    z=sorted(z,key=lambda x:(deadline_key(box[x]),x))
    cut=max(1,len(z)//2)
    p=safe_split(data,row,z[:cut],z[cut:])
    if p is not None: return p,"deadline_balanced"
    # Try every cut as fallback.
    for cut in range(1,len(z)):
        p=safe_split(data,row,z[:cut],z[cut:])
        if p is not None: return p,"deadline_cut"
    return None,None

def split_score(data,row,mode):
    box=bm(data); service=str(row["service"]); z=ids(row["box_ids"])
    hard=sum(is_hard(box[x]) for x in z); soft=len(z)-hard
    blind=1 if service in CLUSTER else 0
    mixed=1 if hard and soft else 0
    # Prefer blind, mixed-urgency, larger batches; direct areas last.
    return (blind,mixed,len(z),hard, -1 if service in DIRECT else 0)

def build(target_k):
    data=load_inputs(); _,base=q1(data)
    rows=[]
    for _,r in base.iterrows():
        d=r.to_dict(); d["box_ids"]=ids(d["box_ids"]); rows.append(d)
    audit=[]
    while len(rows)<target_k:
        cand=[]
        for i,r in enumerate(rows):
            if len(ids(r["box_ids"]))<2: continue
            p,mode=propose_split(data,r)
            if p is not None:
                cand.append((split_score(data,r,mode),i,p,mode))
        if not cand: raise RuntimeError(f"cannot reach K={target_k}")
        _,i,p,mode=max(cand,key=lambda x:x[0])
        old=rows.pop(i); rows.extend(p)
        audit.append({"service":old["service"],"mode":mode,"before":ids(old["box_ids"]),
                      "after":[x["box_ids"] for x in p]})
    return data,pd.DataFrame(rows),audit

def hard_latest(data,service,typ,box_ids):
    tl=pipe.build_authoritative_timeline(data,(service,),typ,box_ids)
    offsets={}
    for e in tl["events"]:
        if e.get("event")=="handoff": offsets[str(e["box_id"])]=float(e["end_s"])
    box=bm(data); latest=14000.0; anyhard=False
    for bid in box_ids:
        b=box[bid]; ds=[]
        if str(b.get("物资类型",""))=="医疗物资": ds.append(float(b["期望送达时间（s）"]))
        if str(b.get("是否首批保障",""))=="是": ds.append(float(b["首批截止时间（s）"]))
        for d in ds:
            anyhard=True; latest=min(latest,d-offsets.get(bid,0.0))
    return max(0.0,latest) if anyhard else 14000.0,tl,offsets

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--k",type=int,required=True)
    ap.add_argument("--out",default="results/q3_relay_friendly")
    a=ap.parse_args(); data,batches,audit=build(a.k)
    rows=[]
    for i,r in batches.reset_index(drop=True).iterrows():
        service=str(r["service"]); typ=str(r["type"]); box_ids=ids(r["box_ids"])
        latest,tl,offsets=hard_latest(data,service,typ,box_ids)
        d=data["drones"][typ]; energy=float(r["energy_kwh"]); soc=1-energy/float(d.energy)
        full={"A":1800.0,"B":2400.0,"C":3000.0}[typ]
        charge=full*(0.65*(0.9-soc)/0.9+0.35) if soc<0.9 else full*0.35*(1-soc)/0.1
        rows.append({
          "candidate_id":f"RF{a.k}-{i+1:03d}","visit_order":service,"service":service,
          "relay_cluster":CLUSTER.get(service,"DIRECT"),"drone_type":typ,"type":typ,
          "box_ids":json.dumps(box_ids,ensure_ascii=False),"duration_s":float(tl["makespan_s"]),
          "latest_start_s":float(latest),"delivery_offsets":json.dumps(offsets,ensure_ascii=False),
          "charge_s":float(max(0.0,charge)),"transport_energy_kwh":energy,
          "hard_box_count":sum(is_hard(bm(data)[x]) for x in box_ids)
        })
    df=pd.DataFrame(rows); allids=[b for x in df.box_ids for b in json.loads(x)]
    if len(allids)!=80 or len(set(allids))!=80: raise RuntimeError("box cover invalid")
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    label=f"RF{a.k}"
    df.to_csv(out/f"{label}.csv",index=False,encoding="utf-8-sig")
    (out/f"{label}_split_audit.json").write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding="utf-8")
    summary={"label":label,"routes":len(df),"boxes":80,"unique_boxes":80,
             "cluster_route_counts":df.relay_cluster.value_counts().to_dict(),
             "hard_route_count":int((df.hard_box_count>0).sum()),
             "strategy":"blind-cluster hard-vs-soft split first, then deadline-balanced"}
    (out/f"{label}.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__": main()
