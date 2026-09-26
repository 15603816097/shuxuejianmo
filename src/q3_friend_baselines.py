"""Friend-paper-style Q3 single-service transport baseline generator.

Creates six Q3 baseline families analogous to the paper's outer comparison:
31, 18, 19a, 19b, 20 and 23 sorties.  These are search baselines, not claimed
to reproduce the paper's hidden route table exactly.

All baselines are generated independently from Q1 single-service feasible
partitions.  K>18 baselines are obtained only by splitting a feasible
single-service batch; 19b deliberately splits a communication-blind service
first to create a structurally different 19-sortie baseline.
"""
from __future__ import annotations
import argparse, ast, json, math
from pathlib import Path
import pandas as pd
import q3_final_pipeline as pipe
from minimal_pipeline import load_inputs, q1, route_stats
from q2_sortie_count_sensitivity import split_to_k

ROOT=Path(__file__).resolve().parents[1]
BLIND={"S002","S003","S004","S005","S007","S008","S009","S010","S012","S013","S014","S015"}

def ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try:return [str(x) for x in json.loads(str(v))]
    except Exception:return [str(x) for x in ast.literal_eval(str(v))]

def boxmap(data): return {str(x["货箱编号"]):x for x in data["boxes"]}

def split_once_variant(data,base,blind_first=False):
    rows=[]
    for _,r in base.iterrows():
        d=r.to_dict(); d["box_ids"]=ids(d["box_ids"]); rows.append(d)
    cand=[]
    for i,r in enumerate(rows):
        n=len(r["box_ids"])
        if n<2: continue
        score=(1 if str(r["service"]) in BLIND else 0,n) if blind_first else (n,1 if str(r["service"]) in BLIND else 0)
        cand.append((score,i))
    if not cand: raise RuntimeError("no splittable batch")
    _,i=max(cand)
    r=rows.pop(i); z=r["box_ids"]; cut=len(z)//2
    bm=boxmap(data)
    for chunk in (z[:cut],z[cut:]):
        st=route_stats(data,str(r["type"]),str(r["service"]),[bm[b] for b in chunk])
        if not st["safe"]: raise RuntimeError("variant split became unsafe")
        nr=dict(r); nr.update(st); nr["box_ids"]=chunk; rows.append(nr)
    return pd.DataFrame(rows)

def hard_latest(data,service,typ,box_ids):
    tl=pipe.build_authoritative_timeline(data,(service,),typ,box_ids)
    offsets={}
    for e in tl["events"]:
        if e.get("event")=="handoff":
            offsets[str(e["box_id"])]=float(e["end_s"])
    bm=boxmap(data); latest=14000.0
    anyhard=False
    for bid in box_ids:
        b=bm[bid]
        ds=[]
        if str(b.get("物资类型",""))=="医疗物资": ds.append(float(b["期望送达时间（s）"]))
        if str(b.get("是否首批保障",""))=="是": ds.append(float(b["首批截止时间（s）"]))
        for d in ds:
            anyhard=True; latest=min(latest,d-offsets.get(bid,0.0))
    return max(0.0,latest) if anyhard else 14000.0, tl, offsets

def make(label,out):
    data=load_inputs(); _,base=q1(data)
    if label=="K18":
        batches=base.copy()
    elif label=="K19A":
        batches=split_to_k(data,base,19)
    elif label=="K19B":
        batches=split_once_variant(data,base,blind_first=True)
    elif label=="K20":
        batches=split_to_k(data,base,20)
    elif label=="K23":
        batches=split_to_k(data,base,23)
    elif label=="K31":
        batches=split_to_k(data,base,31)
    else:
        raise ValueError(label)
    rows=[]
    for i,r in batches.reset_index(drop=True).iterrows():
        service=str(r["service"]); typ=str(r["type"]); box_ids=ids(r["box_ids"])
        latest,tl,offsets=hard_latest(data,service,typ,box_ids)
        d=data["drones"][typ]
        energy=float(r["energy_kwh"])
        soc=1.0-energy/float(d.energy)
        full={"A":1800.0,"B":2400.0,"C":3000.0}[typ]
        charge=full*(0.65*(0.9-soc)/0.9+0.35) if soc<0.9 else full*0.35*(1-soc)/0.1
        rows.append({
          "candidate_id":f"{label}-{i+1:03d}",
          "visit_order":service,
          "service":service,
          "drone_type":typ,
          "type":typ,
          "box_ids":json.dumps(box_ids,ensure_ascii=False),
          "duration_s":float(tl["makespan_s"]),
          "latest_start_s":float(latest),
          "delivery_offsets":json.dumps(offsets,ensure_ascii=False),
          "charge_s":float(max(0.0,charge)),
          "transport_energy_kwh":energy
        })
    df=pd.DataFrame(rows)
    allids=[b for x in df.box_ids for b in json.loads(x)]
    if len(allids)!=80 or len(set(allids))!=80:
        raise RuntimeError(f"{label}: box cover invalid {len(allids)=} {len(set(allids))=}")
    p=Path(out); p.mkdir(parents=True,exist_ok=True)
    df.to_csv(p/f"{label}.csv",index=False,encoding="utf-8-sig")
    summary={"label":label,"routes":len(df),"boxes":len(allids),"unique_boxes":len(set(allids)),
             "hard_latest_min_s":float(df.latest_start_s.min()),"single_service_only":True,
             "scope_note":"Friend-paper-style outer transport baseline; not an exact reproduction of the paper's hidden route table."}
    (p/f"{label}.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--label",required=True); ap.add_argument("--out",default="results/q3_friend_baselines")
    a=ap.parse_args(); make(a.label,a.out)
