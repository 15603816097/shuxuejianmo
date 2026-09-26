"""Q3 temporal-reachability diagnostic and lead-aware hover set cover.

Filters the strict geographic hypergraph by whether a relay mission can be
prepared, flown and linked before each communication block while the associated
transport route still starts no later than its hard-deadline latest start.

This diagnoses whether the previous four-position geographic cover failed
because it ignored relay deployment lead time.
"""
from __future__ import annotations
import argparse, ast, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix

from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy

TOL=1e-9

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return json.loads(str(v))
    except Exception: return ast.literal_eval(str(v))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True)
    ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--out",default="results/q3_temporal_cover")
    a=ap.parse_args()
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    data=load_inputs()
    sch=pd.read_csv(a.schedule).reset_index(drop=True)
    hg=Path(a.hypergraph)
    blocks=pd.read_csv(hg/"block_diagnostics.csv").reset_index(drop=True)
    cand=pd.read_csv(hg/"candidate_coverage.csv").reset_index(drop=True)

    # latest feasible transport start from hard deadlines; NaN means unrestricted
    latest=[]
    for _,r in sch.iterrows():
        latest.append(float(r.latest_start_s) if pd.notna(r.latest_start_s) else 14000.0)
    c=data["center"]; rd=data["relay"]
    rows=[]; cover_sets=[]
    for ci,r in cand.iterrows():
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,peak,*_=line_geometry(c,q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,peak,float(c["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,peak,float(r.altitude_m),float(c["海拔（m）"]))
        lead=float(rd["prep_s"])+float(tout)+float(rd.get("link_s",0.0))
        tail=float(tback)+float(rd["turn_s"])
        geo=set(str(r.covered_blocks).split("|")) if pd.notna(r.covered_blocks) else set()
        temporal=set()
        for bi,b in blocks.iterrows():
            bid=str(b.block_id)
            if bid not in geo: continue
            ri=int(b.route_index)
            # route start must satisfy route_start + block_offset >= lead
            earliest_route=max(0.0,lead-float(b.start_s))
            if earliest_route <= latest[ri]+TOL:
                temporal.add(bid)
        cover_sets.append(temporal)
        rows.append({
            "candidate_index":int(r.candidate_index),"lon":float(r.lon),"lat":float(r.lat),
            "agl_m":float(r.agl_m),"altitude_m":float(r.altitude_m),"source":str(r.source),
            "relay_outbound_s":float(tout),"relay_lead_s":lead,"relay_tail_s":tail,
            "base_energy_kwh":float(eout+eback),
            "geographic_block_count":len(geo),"temporal_block_count":len(temporal),
            "temporal_blocks":"|".join(sorted(temporal))
        })

    all_blocks=[str(x) for x in blocks.block_id]
    byb={b:[] for b in all_blocks}
    for ci,s in enumerate(cover_sets):
        for b in s: byb[b].append(ci)
    diag=[]
    for bi,b in blocks.iterrows():
        bid=str(b.block_id); ri=int(b.route_index)
        diag.append({
            "block_id":bid,"route_index":ri,"visit_order":str(b.visit_order),
            "block_offset_start_s":float(b.start_s),"block_offset_end_s":float(b.end_s),
            "route_latest_start_s":latest[ri],
            "temporal_candidate_count":len(byb[bid]),
            "reachable_before_hard_latest":len(byb[bid])>0
        })
    ddf=pd.DataFrame(diag); ddf.to_csv(out/"block_temporal_reachability.csv",index=False,encoding="utf-8-sig")
    cdf=pd.DataFrame(rows); cdf.to_csv(out/"candidate_temporal_coverage.csv",index=False,encoding="utf-8-sig")

    uncovered=[b for b,z in byb.items() if not z]
    chosen=[]
    status="UNREACHABLE_BLOCKS"
    if not uncovered:
        useful=[i for i,s in enumerate(cover_sets) if s]
        A=lil_matrix((len(all_blocks),len(useful)),dtype=float)
        bix={b:i for i,b in enumerate(all_blocks)}
        for j,ci in enumerate(useful):
            for b in cover_sets[ci]: A[bix[b],j]=1.0
        res=milp(np.ones(len(useful)),integrality=np.ones(len(useful)),bounds=Bounds(0,1),
                 constraints=LinearConstraint(A.tocsr(),1,np.inf),options={"time_limit":120})
        if res.x is not None:
            chosen=[useful[j] for j in np.flatnonzero(res.x>.5)]
            status="TEMPORAL_SET_COVER_FOUND"
            cdf.iloc[chosen].to_csv(out/"minimum_temporal_cover_positions.csv",index=False,encoding="utf-8-sig")
    summary={
        "status":status,"routes":len(sch),"blocks":len(blocks),
        "candidates":len(cand),"unreachable_blocks":uncovered,
        "reachable_blocks":len(all_blocks)-len(uncovered),
        "minimum_temporal_cover_positions":None if not chosen else len(chosen),
        "geographic_minimum_positions_previous":4,
        "four_positions_still_possible_after_lead_filter":bool(chosen and len(chosen)<=4),
        "note":"This gate includes hard-deadline latest starts and relay prep/outbound/link lead time, but not two-relay concurrency or cross-wave component reuse."
    }
    (out/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
