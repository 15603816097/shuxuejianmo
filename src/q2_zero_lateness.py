"""Q2 zero-soft-lateness search.

Feasibility/optimization model built on q2_v2_solver candidate pool:
- exactly-once for all 80 boxes
- hard medical/first-batch deadlines
- additionally force every non-medical box to meet its expected delivery time
- optional fixed K
- minimize makespan among zero-soft-lateness schedules

This tests whether total soft lateness can be driven to zero under the current
candidate-pool model. FEASIBLE proves existence; UNKNOWN does not prove
infeasibility.
"""
from __future__ import annotations

import argparse, json, math
from pathlib import Path
import pandas as pd
from ortools.sat.python import cp_model
import q2_v2_solver as q2

OUT = q2.OUT / "zero_late"
OUT.mkdir(parents=True, exist_ok=True)

def solve_zero_late(data, pool, time_limit_s=360, fixed_k=None):
    box_ids = sorted(str(x["货箱编号"]) for x in data["boxes"])
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    m = cp_model.CpModel()
    H = int(math.ceil(13000.0 * q2.SCALE))

    candidates=[]; by_box={b:[] for b in box_ids}
    xvars=[]; svars=[]; evars=[]
    air_ints={}; bat_ints={}

    for i,r in pool.iterrows():
        ids=json.loads(r.box_ids)
        offs={str(a):float(b) for a,b in json.loads(r.delivery_offsets).items()}
        dur=int(math.ceil(float(r.duration_s)*q2.SCALE-1e-9))
        bdur=int(math.ceil((float(r.duration_s)+float(r.charge_s))*q2.SCALE-1e-9))
        x=m.NewBoolVar(f"x_{i}")
        s=m.NewIntVar(0,H,f"s_{i}")
        e=m.NewIntVar(0,H,f"e_{i}")
        be=m.NewIntVar(0,H+bdur,f"be_{i}")
        ai=m.NewOptionalIntervalVar(s,dur,e,x,f"air_{i}")
        bi=m.NewOptionalIntervalVar(s,bdur,be,x,f"bat_{i}")
        m.Add(s==0).OnlyEnforceIf(x.Not()); m.Add(e==0).OnlyEnforceIf(x.Not()); m.Add(be==0).OnlyEnforceIf(x.Not())

        # Existing hard deadline aggregation from the candidate.
        if pd.notna(r.latest_start_s):
            latest=int(math.floor(float(r.latest_start_s)*q2.SCALE+1e-9))
            m.Add(s<=latest).OnlyEnforceIf(x)

        # Zero-soft-lateness: every non-medical box must meet expected delivery.
        for bid in ids:
            b=bm[str(bid)]
            if b.get("物资类型") != "医疗物资":
                due=float(b["期望送达时间（s）"])
                off=offs[str(bid)]
                latest=int(math.floor((due-off)*q2.SCALE+1e-9))
                m.Add(s<=latest).OnlyEnforceIf(x)

        typ=str(r.drone_type)
        air_ints.setdefault(typ,[]).append(ai)
        bat_ints.setdefault(typ,[]).append(bi)
        candidates.append({"row":r,"ids":ids,"offs":offs,"x":x,"s":s,"e":e})
        xvars.append(x); svars.append(s); evars.append(e)
        for bid in ids: by_box[str(bid)].append(i)

    for bid in box_ids:
        idx=by_box[bid]
        if not idx: raise RuntimeError(f"pool misses {bid}")
        m.Add(sum(xvars[i] for i in idx)==1)

    if fixed_k is not None:
        m.Add(sum(xvars)==int(fixed_k))

    for typ,ints in air_ints.items():
        m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat_ints.items():
        m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    cmax=m.NewIntVar(0,H,"Cmax")
    for c in candidates:
        m.Add(cmax>=c["e"]).OnlyEnforceIf(c["x"])
    for typ in air_ints:
        idx=[i for i,c in enumerate(candidates) if str(c["row"].drone_type)==typ]
        terms=[]
        for i in idx:
            d=int(math.ceil(float(candidates[i]["row"].duration_s)*q2.SCALE-1e-9))
            terms.append(d*xvars[i])
        if terms: m.Add(sum(terms)<=len(data["aircraft"][typ])*cmax)

    m.Minimize(cmax)
    solver=cp_model.CpSolver()
    solver.parameters.max_time_in_seconds=float(time_limit_s)
    solver.parameters.num_search_workers=8
    status=solver.Solve(m)
    name=solver.StatusName(status)
    feasible=status in (cp_model.FEASIBLE,cp_model.OPTIMAL)

    summary={
        "status":name,
        "fixed_k":fixed_k,
        "candidate_count":len(pool),
        "zero_soft_lateness_feasible_found":bool(feasible),
        "proven_optimal_current_pool":bool(status==cp_model.OPTIMAL),
        "time_limit_s":time_limit_s,
    }
    if not feasible:
        return summary,None,None

    rows=[]
    for i,c in enumerate(candidates):
        if solver.Value(c["x"]):
            r=c["row"].to_dict()
            r["start_s"]=solver.Value(c["s"])/q2.SCALE
            r["return_s"]=solver.Value(c["e"])/q2.SCALE
            rows.append(r)
    sch=pd.DataFrame(rows).sort_values(["start_s","return_s"]).reset_index(drop=True)

    deliveries=[]
    for idx,r in sch.iterrows():
        offs=json.loads(r.delivery_offsets)
        for bid,off in offs.items():
            b=bm[str(bid)]
            ct=float(r.start_s)+float(off)
            hard=q2.box_hard_deadline(b)
            soft=None if b.get("物资类型")=="医疗物资" else float(b["期望送达时间（s）"])
            deliveries.append({
                "box_id":str(bid),"completion_s":ct,
                "hard_deadline_s":hard,
                "hard_ok":hard is None or ct<=hard+1e-6,
                "soft_deadline_s":soft,
                "soft_lateness_s":0.0 if soft is None else max(0.0,ct-soft),
            })
    dl=pd.DataFrame(deliveries)
    summary.update({
        "K":int(len(sch)),
        "makespan_s":float(sch.return_s.max()),
        "best_bound_s":float(solver.BestObjectiveBound())/q2.SCALE,
        "boxes":int(len(dl)),
        "unique_boxes":int(dl.box_id.nunique()),
        "hard_violations":int((~dl.hard_ok).sum()),
        "soft_late_boxes":int((dl.soft_lateness_s>1e-6).sum()),
        "soft_total_lateness_s":float(dl.soft_lateness_s.sum()),
        "energy_kwh":float(sch.energy_kwh.sum()),
        "A_sorties":int((sch.drone_type=="A").sum()),
        "B_sorties":int((sch.drone_type=="B").sum()),
        "C_sorties":int((sch.drone_type=="C").sum()),
    })
    return summary,sch,dl

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--time-limit",type=int,default=360)
    ap.add_argument("--triple-limit",type=int,default=900)
    ap.add_argument("--k",type=int,default=None)
    args=ap.parse_args()

    data=q2.load_inputs()
    pool=q2.generate_pool(data,args.triple_limit)
    summary,sch,dl=solve_zero_late(data,pool,args.time_limit,args.k)
    tag="freeK" if args.k is None else f"K{args.k:02d}"
    d=OUT/tag; d.mkdir(parents=True,exist_ok=True)
    (d/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    if sch is not None:
        sch.to_csv(d/"schedule.csv",index=False,encoding="utf-8-sig")
        dl.to_csv(d/"deliveries.csv",index=False,encoding="utf-8-sig")
    print(json.dumps(summary,ensure_ascii=False))

if __name__=="__main__":
    main()
