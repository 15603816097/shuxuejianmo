"""Q3 strict route-set seed search from the frozen Q2-v2 candidate universe.

Workflow:
1) generate the same 15,218-ish Q2 transport candidate universe;
2) filter candidates by the strict all-type directed communication graph;
3) solve exact 80-box set partition for K=22..26;
4) lazily certify only selected routes with the frozen strict Q3 evaluator;
5) blacklist route-level geometry failures and re-solve.

The output is only a route-set geometry certificate. Global start times,
transport/battery resources, relay drones and relay energy components are
explicitly deferred to the next Q3 gate.
"""
from __future__ import annotations
import ast, hashlib, json, math, time
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix, vstack

import q2_v2_solver as q2
import q3_final_pipeline as pipe

ROOT=Path(__file__).resolve().parents[1]
RES=ROOT/"results"/"q3_official"
RES.mkdir(parents=True,exist_ok=True)
SHA="5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb"

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return [str(x) for x in json.loads(str(v))]
    except Exception: return [str(x) for x in ast.literal_eval(str(v))]

def edge_keys(order):
    s=[x for x in str(order).split(">") if x]
    n=["O01"]+s+["O01"]
    return list(zip(n,n[1:]))

def solve_exact_k(c:pd.DataFrame, boxes:list[str], K:int, time_limit=120):
    if c.empty: return None,None
    ix={b:i for i,b in enumerate(boxes)}
    A=lil_matrix((len(boxes),len(c)),dtype=float)
    for j,z in enumerate(c.box_ids):
        for b in parse_ids(z):
            if b in ix: A[ix[b],j]=1.0
    cnt=lil_matrix((1,len(c)),dtype=float); cnt[0,:]=1.0
    AA=vstack([A.tocsr(),cnt.tocsr()])
    lo=np.r_[np.ones(len(boxes)),K]
    hi=np.r_[np.ones(len(boxes)),K]
    # Route count is fixed. Prefer low transport energy, then shorter route
    # duration. Scheduling/timeliness is optimized in the next gate.
    cost=np.array([float(e)+1e-5*float(d) for e,d in zip(c.energy_kwh,c.duration_s)],dtype=float)
    res=milp(cost,integrality=np.ones(len(c)),bounds=Bounds(0,1),
             constraints=LinearConstraint(AA,lo,hi),
             options={"time_limit":float(time_limit)})
    if res.x is None: return res,None
    sel=np.flatnonzero(res.x>.5)
    return res,c.iloc[sel].copy()

def certify_routes(data,sel):
    rows=[]; failed=[]
    for _,r in sel.iterrows():
        ids=parse_ids(r.box_ids)
        order=tuple(str(r.visit_order).split(">"))
        out=pipe.evaluate(data,order,ids,str(r.drone_type))
        candidates=[x for x in out["hover_candidates"] if x is not None]
        geometry=bool(out["status"]["COMMUNICATION_GEOMETRY_PASS"])
        transport=bool(out["status"]["TRANSPORT_FEASIBLE"])
        ok=bool(geometry and transport)
        rows.append({
            "candidate_id":str(r.candidate_id),"visit_order":str(r.visit_order),
            "drone_type":str(r.drone_type),"box_count":len(ids),
            "transport_feasible":transport,"communication_geometry_pass":geometry,
            "route_geometry_certified":ok,
            "relay_blocks":len(out["relay_required_blocks"]),
            "min_joint_margin_db":min([float(x["joint_margin"]) for x in candidates],default=float("inf")),
            "transport_energy_kwh":float(out["transport_energy_kwh"])
        })
        if not ok: failed.append(str(r.candidate_id))
    return pd.DataFrame(rows),failed

def main():
    t0=time.time()
    got=hashlib.sha256((ROOT/"src/q3_final_pipeline.py").read_bytes()).hexdigest()
    if got!=SHA: raise RuntimeError(f"frozen SHA mismatch: {got}")
    data=q2.load_inputs()
    official=sorted(str(x["货箱编号"]) for x in data["boxes"])
    if len(official)!=80: raise RuntimeError(f"expected 80 boxes, got {len(official)}")

    graph=pd.read_csv(RES/"strict_edge_graph_all_types.csv")
    possible={}
    for _,r in graph.iterrows():
        possible[(str(r.from_node),str(r.to_node),str(r.drone_type))] = (
            str(r.evaluation_status)=="COMPLETED" and bool(r.communication_possible)
        )

    pool=q2.generate_pool(data,900)
    keep=[]
    for _,r in pool.iterrows():
        t=str(r.drone_type)
        keep.append(all(possible.get((a,b,t),False) for a,b in edge_keys(r.visit_order)))
    screened=pool.loc[keep].copy().reset_index(drop=True)
    screened.to_csv(RES/"strict_edge_screened_q2_pool.csv",index=False,encoding="utf-8-sig")

    coverage={b:0 for b in official}
    for z in screened.box_ids:
        for b in parse_ids(z):
            if b in coverage: coverage[b]+=1
    pd.DataFrame([{"box_id":b,"candidate_count":coverage[b],"covered":coverage[b]>0} for b in official]).to_csv(
        RES/"strict_pool_box_coverage.csv",index=False,encoding="utf-8-sig")

    attempts=[]; final_sel=None; final_cert=None; final_k=None
    blacklist=set()
    for K in (22,23,24,25,26):
        for it in range(1,9):
            cur=screened.loc[~screened.candidate_id.astype(str).isin(blacklist)].copy()
            res,sel=solve_exact_k(cur,official,K,120)
            status=str(getattr(res,"message","NO_RESULT")) if res is not None else "NO_RESULT"
            if sel is None:
                attempts.append({"K":K,"iteration":it,"candidate_count":len(cur),
                                 "selected":0,"failed_selected":None,"status":status})
                break
            cert,failed=certify_routes(data,sel)
            attempts.append({"K":K,"iteration":it,"candidate_count":len(cur),
                             "selected":len(sel),"failed_selected":len(failed),
                             "status":"ROUTE_GEOMETRY_CERTIFIED" if not failed else "BLACKLIST_AND_REPEAT"})
            cert.to_csv(RES/f"route_cert_K{K}_iter{it}.csv",index=False,encoding="utf-8-sig")
            sel.to_csv(RES/f"route_set_K{K}_iter{it}.csv",index=False,encoding="utf-8-sig")
            if not failed:
                final_sel=sel; final_cert=cert; final_k=K
                break
            blacklist.update(failed)
        if final_sel is not None: break

    pd.DataFrame(attempts).to_csv(RES/"strict_route_set_search_iterations.csv",index=False,encoding="utf-8-sig")
    if final_sel is not None:
        final_sel.to_csv(RES/"strict_route_set_seed.csv",index=False,encoding="utf-8-sig")
        final_cert.to_csv(RES/"strict_route_set_certificate.csv",index=False,encoding="utf-8-sig")
        union=[]
        for z in final_sel.box_ids: union += parse_ids(z)
        status="ROUTE_SET_GEOMETRY_CERTIFIED_NEEDS_GLOBAL_SCHEDULE"
    else:
        union=[]; status="NO_ROUTE_SET_GEOMETRY_CERTIFICATE_IN_K22_K26"

    summary={
        "status":status,
        "frozen_pipeline_sha256":got,
        "official_box_count":80,
        "edge_screened_candidate_count":int(len(screened)),
        "covered_boxes":int(sum(v>0 for v in coverage.values())),
        "uncovered_boxes":[b for b,v in coverage.items() if v==0],
        "selected_K":None if final_k is None else int(final_k),
        "selected_route_count":0 if final_sel is None else int(len(final_sel)),
        "selected_box_count":int(len(union)),
        "selected_unique_boxes":int(len(set(union))),
        "duplicate_box_count":int(len(union)-len(set(union))),
        "route_geometry_all_pass":bool(final_cert is not None and final_cert.route_geometry_certified.all()),
        "global_transport_resource_certified":False,
        "global_relay_resource_certified":False,
        "continuous_global_communication_certified":False,
        "runtime_s":time.time()-t0,
        "scope_note":"This gate certifies strict route geometry for an exact 80-box route set. It does not yet certify global clocks or relay/component resource calendars."
    }
    (RES/"q3_stage1_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
