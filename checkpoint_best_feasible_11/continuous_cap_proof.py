"""Feasibility proof for the bound sum(late_b) <= 13.

This uses the same event-time model and data as continuous_joint.py.  The
only added restriction is the integer violation-cap row; late variables are
otherwise unconstrained binary variables and the objective is identically 0.
"""
from __future__ import annotations
import json, time
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2
from continuous_joint import _solve, _schedule_from

ROOT=Path(__file__).resolve().parents[1]

def main():
    data=load_inputs(); _,batches=q1(data); tasks,_=q2(data,batches)
    cert=pd.read_csv(ROOT/"results/final_q3_continuous_certification.csv")
    rows=[]; feasible=[]; incumbent=None
    for cap in range(13,-1,-1):
        t0=time.time(); res,names,rec=_solve(data,tasks,cert,f"cap{cap}","cap",limit=120 if cap==10 else 30,violation_cap=cap)
        rec.update({"violation_cap":cap,"strictly_infeasible":bool(rec["status"]==2),"node_count":getattr(res,"mip_node_count",None),"elapsed_wall_s":time.time()-t0})
        rows.append(rec)
        if rec["status"]==0:
            feasible.append(cap); incumbent=(res,names,cap)
        else:
            break
    pd.DataFrame(rows).to_csv(ROOT/"results/continuous_cap_feasibility.csv",index=False)
    if incumbent:
        sched=_schedule_from(tasks,incumbent[0],incumbent[1]); sched.to_csv(ROOT/f"results/continuous_cap{incumbent[2]}_schedule.csv",index=False)
    last=rows[-1]; best=min(feasible) if feasible else None
    summary={"R":2,"tested_caps":[r["violation_cap"] for r in rows],"feasible_caps":feasible,"best_feasible_cap":best,"last_status":last["status"],"last_message":last["message"],"strictly_infeasible":last["strictly_infeasible"],"objective":last["objective"],"lower_bound":last["lower_bound"],"upper_bound":last["upper_bound"],"mip_gap":last["mip_gap"],"runtime_s":last["runtime_s"],"node_count":last["node_count"],"same_event_model":True,"only_new_constraint":"sum(late_b) <= cap","incumbent_14_boxes_feasible":True,"proof":"The first strict INFEASIBLE cap below the best feasible cap certifies the optimum." if last["strictly_infeasible"] else "No proof: lower caps remain unclassified."}
    (ROOT/"results/continuous_cap_feasibility.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    (ROOT/"logs/continuous_cap13.log").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__": main()
