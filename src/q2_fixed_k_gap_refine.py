"""Uniform fixed-K zero-lateness makespan refinement/proof run.

Targets one K at a time under the same 15,218-route candidate-pool model.
Zero soft lateness is enforced. An incumbent schedule is used as a hint and
an incumbent makespan is imposed as an upper bound. The result reports both
the best feasible makespan and CP-SAT lower bound, so a zero gap certifies
optimality for the current candidate-pool model.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import pandas as pd
from ortools.sat.python import cp_model
import q2_v2_solver as q2
import q2_fixed_k_lexicographic as lex

OUT=q2.OUT/"gap_refine"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--k",type=int,required=True)
    ap.add_argument("--seed",type=int,required=True)
    ap.add_argument("--time-limit",type=int,default=1200)
    ap.add_argument("--hint-csv",required=True)
    ap.add_argument("--upper-bound",type=float,required=True)
    ap.add_argument("--triple-limit",type=int,default=900)
    a=ap.parse_args()

    data=q2.load_inputs()
    pool=q2.generate_pool(data,a.triple_limit)
    hint=pd.read_csv(a.hint_csv)[["candidate_id","start_s"]].copy()

    m,cands,cmax,total_late,bm=lex.build(
        data,pool,a.k,"makespan",late_cap=0,hint=hint
    )
    m.Add(cmax<=int(math.floor(a.upper_bound*q2.SCALE+1e-9)))

    s=cp_model.CpSolver()
    s.parameters.max_time_in_seconds=float(a.time_limit)
    s.parameters.num_search_workers=8
    s.parameters.random_seed=int(a.seed)
    s.parameters.randomize_search=True
    status=s.Solve(m)
    name=s.StatusName(status)
    feasible=status in (cp_model.FEASIBLE,cp_model.OPTIMAL)

    summary={
        "K":a.k,
        "seed":a.seed,
        "status":name,
        "time_limit_s":a.time_limit,
        "candidate_count":len(pool),
        "upper_bound_input_s":a.upper_bound,
        "zero_lateness_enforced":True,
        "feasible_found":bool(feasible),
        "proven_optimal_current_pool":bool(status==cp_model.OPTIMAL),
        "scope_note":"Optimality/gap refers to the current 15,218-candidate route-pool model, not the unrestricted physical route space."
    }
    sch=dl=None
    if feasible:
        sch,dl=lex.reconstruct(data,cands,s,bm)
        obj=float(s.ObjectiveValue())/q2.SCALE
        bound=float(s.BestObjectiveBound())/q2.SCALE
        summary.update({
            "makespan_s":float(sch.return_s.max()),
            "objective_s":obj,
            "best_bound_s":bound,
            "gap_pct":0.0 if obj<=0 else 100.0*(obj-bound)/obj,
            "soft_late_boxes":int((dl.soft_lateness_s>1e-6).sum()),
            "soft_total_lateness_s":float(dl.soft_lateness_s.sum()),
            "hard_violations":int((~dl.hard_ok).sum()),
            "boxes":int(len(dl)),
            "unique_boxes":int(dl.box_id.nunique()),
            "energy_kwh":float(sch.energy_kwh.sum()),
            "A_sorties":int((sch.drone_type=="A").sum()),
            "B_sorties":int((sch.drone_type=="B").sum()),
            "C_sorties":int((sch.drone_type=="C").sum()),
        })

    d=OUT/f"K{a.k:02d}"/f"seed_{a.seed}"
    d.mkdir(parents=True,exist_ok=True)
    (d/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    if sch is not None:
        sch.to_csv(d/"schedule.csv",index=False,encoding="utf-8-sig")
        dl.to_csv(d/"deliveries.csv",index=False,encoding="utf-8-sig")
    print(json.dumps(summary,ensure_ascii=False))

if __name__=="__main__":
    main()
