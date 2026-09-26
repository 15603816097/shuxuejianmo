"""Dense strict hover search for the frozen multi-stop Q3 transport seed.

Purpose: test whether the apparent four-position geographic lower bound from the
1596-point grid is a discretization artifact.  Rebuild the exact relay-required
blocks, evaluate a substantially denser full-DEM stationary-hover grid, merge
the previous useful candidates, and solve minimum set cover again.

This is still a candidate-space certificate, not a proof over the continuous
longitude/latitude/altitude domain.
"""
from __future__ import annotations
import argparse, ast, hashlib, json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix

import q3_final_pipeline as pipe
import q3_hover_hypergraph as hg

ROOT=Path(__file__).resolve().parents[1]
SHA="5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb"

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try:return [str(x) for x in json.loads(str(v))]
    except Exception:return [str(x) for x in ast.literal_eval(str(v))]

def build_blocks(data,sch,label):
    blocks=[]
    for ri,r in sch.iterrows():
        order=tuple(x for x in str(r.visit_order).split(">") if x)
        tl=pipe.build_authoritative_timeline(data,order,str(r.drone_type),parse_ids(r.box_ids))
        for bi,b in enumerate(pipe.direct_and_relay_blocks(data,tl)):
            blocks.append({"block_id":f"{label}-R{ri+1:02d}-B{bi+1:02d}",
                           "route_index":int(ri),"candidate_id":str(r.get("candidate_id","")),
                           "visit_order":str(r.visit_order),"drone_type":str(r.drone_type),**b})
    return blocks

def dense_candidates(data,old_cov):
    lons=np.asarray(data["dem"]["longitude"]).ravel()
    lats=np.asarray(data["dem"]["latitude"]).ravel()
    xs=np.linspace(float(lons.min()),float(lons.max()),31)
    ys=np.linspace(float(lats.min()),float(lats.max()),23)
    agls=[25.,50.,75.,100.,125.,150.,175.,200.,225.,250.,275.,300.]
    out=[]; seen=set()
    def add(lon,lat,agl,src):
        k=(round(float(lon),8),round(float(lat),8),round(float(agl),2))
        if k in seen:return
        z=hg.candidate_record(data,float(lon),float(lat),float(agl),src)
        if z is not None:
            seen.add(k); out.append(z)
    for x in xs:
        for y in ys:
            for a in agls:add(x,y,a,"dense_full_dem")
    if old_cov is not None and old_cov.exists():
        df=pd.read_csv(old_cov)
        for _,r in df.iterrows():
            add(r.lon,r.lat,r.agl_m,"previous_useful")
        # Local densification around the most broadly useful old candidates.
        top=df.sort_values("covered_block_count",ascending=False).head(80)
        for _,r in top.iterrows():
            for dx in np.linspace(-0.0020,0.0020,5):
                for dy in np.linspace(-0.0020,0.0020,5):
                    for da in (-25.,0.,25.):
                        a=min(300.,max(10.,float(r.agl_m)+da))
                        add(float(r.lon)+dx,float(r.lat)+dy,a,"top80_local_bridge")
    return out

def main(schedule,old_hg,outdir):
    got=hashlib.sha256((ROOT/"src/q3_final_pipeline.py").read_bytes()).hexdigest()
    if got!=SHA: raise RuntimeError(f"frozen Q3 SHA mismatch: {got}")
    data=pipe.load_inputs(); sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=build_blocks(data,sch,"DENSE")
    out=Path(outdir); out.mkdir(parents=True,exist_ok=True)
    old_cov=Path(old_hg)/"candidate_coverage.csv" if old_hg else None
    cands=dense_candidates(data,old_cov)
    _,ta,tb=pipe.thresholds(data)
    statics=[hg.candidate_static(data,c,tb) for c in cands]
    cover=[set() for _ in cands]
    print(json.dumps({"blocks":len(blocks),"candidates":len(cands)},ensure_ascii=False),flush=True)

    for ci,(c,st) in enumerate(zip(cands,statics)):
        for bi,b in enumerate(blocks):
            score,am,bm,eok,e=hg.block_score(data,b,c,ta,st)
            if score>=-1e-9 and eok:cover[ci].add(bi)
        if (ci+1)%500==0:
            print(f"dense evaluated {ci+1}/{len(cands)}",flush=True)

    covered_by=[[] for _ in blocks]
    for ci,s in enumerate(cover):
        for bi in s:covered_by[bi].append(ci)
    uncovered=[blocks[i]["block_id"] for i,z in enumerate(covered_by) if not z]
    chosen=[]
    if not uncovered:
        useful=[i for i,s in enumerate(cover) if s]
        A=lil_matrix((len(blocks),len(useful)),dtype=float)
        for j,ci in enumerate(useful):
            for bi in cover[ci]:A[bi,j]=1.
        res=milp(np.ones(len(useful)),integrality=np.ones(len(useful)),
                 bounds=Bounds(0,1),
                 constraints=LinearConstraint(A.tocsr(),1,np.inf),
                 options={"time_limit":300})
        if res.x is not None:
            chosen=[useful[j] for j in np.flatnonzero(res.x>.5)]

    rows=[]
    for i,c in enumerate(cands):
        if cover[i]:
            rows.append({"candidate_index":i,"lon":c["lon"],"lat":c["lat"],
                         "agl_m":c["agl_m"],"altitude_m":c["altitude_m"],
                         "source":c["source"],"covered_block_count":len(cover[i]),
                         "chosen_min_cover":i in chosen,
                         "covered_blocks":"|".join(blocks[j]["block_id"] for j in sorted(cover[i]))})
    pd.DataFrame(rows).to_csv(out/"candidate_coverage.csv",index=False,encoding="utf-8-sig")
    # target_search only needs block ids, route indices and relative block times here.
    pd.DataFrame([{"block_id":b["block_id"],"route_index":b["route_index"],
                   "visit_order":b["visit_order"],"start_s":b["start_s"],"end_s":b["end_s"]}
                  for b in blocks]).to_csv(out/"block_diagnostics.csv",index=False,encoding="utf-8-sig")
    if chosen:
        pd.DataFrame([r for r in rows if r["candidate_index"] in set(chosen)]).to_csv(
            out/"minimum_cover_positions.csv",index=False,encoding="utf-8-sig")
    summary={"label":"MULTI_DENSE","routes":len(sch),"relay_blocks":len(blocks),
             "candidate_count":len(cands),"useful_candidate_count":len(rows),
             "covered_blocks":len(blocks)-len(uncovered),"uncovered_blocks":uncovered,
             "all_blocks_covered":not uncovered,
             "minimum_hover_positions":None if uncovered or not chosen else len(chosen),
             "three_position_geographic_candidate_found":bool(chosen and len(chosen)<=3),
             "scope_note":"Dense full-DEM + local bridge candidate space; not a proof over the continuous hover domain."}
    (out/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print("DENSE3_SUMMARY="+json.dumps(summary,ensure_ascii=False),flush=True)

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True)
    ap.add_argument("--old-hg",required=True)
    ap.add_argument("--out",required=True)
    a=ap.parse_args(); main(a.schedule,a.old_hg,a.out)
