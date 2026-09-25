"""Strict Q3 route-level audit of the frozen Q2 zero-lateness schedule.

This is the first Q3 gate only: each selected transport route is reevaluated
with the frozen strict Q3 evaluator. It deliberately does not claim global
relay/resource schedulability; that is a later gate.
"""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import pandas as pd
import q3_final_pipeline as pipe

ROOT=Path(__file__).resolve().parents[1]
RES=ROOT/"results"/"q3_strict"
RES.mkdir(parents=True,exist_ok=True)
SHA="5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True)
    args=ap.parse_args()
    got=hashlib.sha256((ROOT/"src/q3_final_pipeline.py").read_bytes()).hexdigest()
    if got!=SHA:
        raise RuntimeError(f"frozen q3 evaluator sha mismatch: {got}")

    data=pipe.load_inputs()
    sch=pd.read_csv(args.schedule)
    rows=[]
    box_union=[]
    for _,r in sch.iterrows():
        ids=json.loads(r.box_ids) if isinstance(r.box_ids,str) else list(r.box_ids)
        order=tuple(str(r.visit_order).split(">"))
        out=pipe.evaluate(
            data,order,ids,str(r.drone_type),
            drone_id=str(r.get("aircraft","AUTO")),
            start_time=float(r.get("start_s",0.0)),
            battery_id=str(r.get("battery","AUTO")),
        )
        box_union.extend(str(x) for x in ids)
        rows.append({
            "sortie":str(r.get("sortie","")),
            "candidate_id":str(r.get("candidate_id","")),
            "visit_order":str(r.visit_order),
            "drone_type":str(r.drone_type),
            "box_count":len(ids),
            "transport_feasible":bool(out["status"]["TRANSPORT_FEASIBLE"]),
            "communication_geometry_pass":bool(out["status"]["COMMUNICATION_GEOMETRY_PASS"]),
            "relay_chain_pass":bool(out["status"]["RELAY_CHAIN_PASS"]),
            "component_pass":bool(out["status"]["COMPONENT_PASS"]),
            "route_feasible":bool(out["status"]["ROUTE_FEASIBLE"]),
            "transport_energy_kwh":float(out["transport_energy_kwh"]),
            "relay_energy_kwh":None if pd.isna(out["relay_energy_kwh"]) else float(out["relay_energy_kwh"]),
            "relay_blocks":len(out["relay_required_blocks"]),
            "min_joint_margin_db":min([float(x["joint_margin"]) for x in out["hover_candidates"] if x is not None],default=float("inf")),
        })

    df=pd.DataFrame(rows)
    df.to_csv(RES/"q2_final_route_level_strict_audit.csv",index=False,encoding="utf-8-sig")
    official={str(x["货箱编号"]) for x in data["boxes"]}
    observed=set(box_union)
    summary={
        "frozen_pipeline_sha256":got,
        "input_sorties":int(len(sch)),
        "input_boxes":int(len(box_union)),
        "unique_boxes":int(len(observed)),
        "missing_boxes":sorted(official-observed),
        "duplicate_box_count":int(len(box_union)-len(observed)),
        "transport_feasible_routes":int(df.transport_feasible.sum()),
        "communication_geometry_pass_routes":int(df.communication_geometry_pass.sum()),
        "route_feasible_routes":int(df.route_feasible.sum()),
        "communication_failures":int((~df.communication_geometry_pass).sum()),
        "route_failures":int((~df.route_feasible).sum()),
        "strict_route_level_all_pass":bool(
            len(observed)==80 and observed==official and
            df.transport_feasible.all() and df.route_feasible.all()
        ),
        "global_relay_resource_certified":False,
        "status":"ROUTE_LEVEL_PASS_NEEDS_GLOBAL_RESOURCE_GATE" if (
            len(observed)==80 and observed==official and df.route_feasible.all()
        ) else "Q2_SCHEDULE_NOT_STRICT_Q3_ROUTE_FEASIBLE",
        "note":"This is a strict route-level gate only. Global relay/component resource scheduling is not certified here."
    }
    (RES/"q2_final_route_level_strict_summary.json").write_text(
        json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"
    )
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
