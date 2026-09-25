"""Cap-10 strengthened feasibility checkpoint.

The installed SciPy MILP interface has no MIP-start parameter.  This runner
records that fact explicitly, applies the unchanged event constraints plus the
cap row, and records safe presolve/pruning diagnostics rather than claiming
unsupported cuts were used.
"""
from __future__ import annotations
import json, time
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2
from continuous_joint import _solve

ROOT=Path(__file__).resolve().parents[1]

def main():
    data=load_inputs(); _,b=q1(data); tasks,_=q2(data,b); cert=pd.read_csv(ROOT/"results/final_q3_continuous_certification.csv")
    incumbent=pd.read_csv(ROOT/"results/continuous_cap11_schedule.csv")
    finite=cert[cert["relay_feasible"].astype(bool)]
    diagnostics={"warm_start_requested":True,"warm_start_adopted":False,"warm_start_reason":"scipy.optimize.milp exposes no x0/MIP-start; highspy unavailable","relay_candidates_total":int(len(cert)),"relay_candidates_retained":int(len(finite)),"relay_candidates_removed":int(len(cert)-len(finite)),"task_resource_pruning":"type-compatible aircraft/battery sets already enumerated; no additional safe eliminations found","symmetry_breaking":"not applied: no solver-safe unit-order formulation was added without changing the contract","deadline_dominance":"not applied: pairwise dominance could alter allowed task ordering","window_cuts":"not applied: no valid cut generator in current contract implementation","incumbent_rows":int(len(incumbent))}
    t0=time.time(); res,names,rec=_solve(data,tasks,cert,"cap10_strengthened","cap",limit=120,violation_cap=10); rec.update({"violation_cap":10,"strictly_infeasible":bool(rec["status"]==2),"node_count":getattr(res,"mip_node_count",None),"presolve_variables":len(names),"presolve_constraints":int(rec["constraints"]),"elapsed_wall_s":time.time()-t0})
    out={"R":2,"cap":10,"solver":rec,"diagnostics":diagnostics,"same_event_model":True,"only_objective_constraint":"sum(late_b) <= 10","incumbent_11_boxes_retained":True,"P1":"PASS" if rec["strictly_infeasible"] else "FAIL"}
    (ROOT/"results/cap10_strengthened.json").write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding="utf-8"); (ROOT/"logs/cap10_strengthened.log").write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(out,ensure_ascii=False,indent=2))

if __name__=="__main__": main()
