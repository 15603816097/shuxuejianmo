"""Fixed-K Q2 lexicographic search.

For a requested K:
1) minimize total soft lateness;
2) keep the best lateness value found and minimize makespan.

Uses the same 15,218-candidate Q2-v2 pool and resource/physics semantics.
"""
from __future__ import annotations
import argparse, json, math, time
from pathlib import Path
import pandas as pd
from ortools.sat.python import cp_model
import q2_v2_solver as q2

OUT = q2.OUT / "fixed_k_lexicographic"
OUT.mkdir(parents=True, exist_ok=True)

def build(data, pool, k, objective, late_cap=None, hint=None):
    bm={str(x["货箱编号"]):x for x in data["boxes"]}
    box_ids=sorted(bm)
    m=cp_model.CpModel()
    H=int(math.ceil(13000*q2.SCALE))
    candidates=[]; by_box={b:[] for b in box_ids}
    xvars=[]; air={}; bat={}

    for i,r in pool.iterrows():
        ids=[str(x) for x in json.loads(r.box_ids)]
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
        if pd.notna(r.latest_start_s):
            m.Add(s<=int(math.floor(float(r.latest_start_s)*q2.SCALE+1e-9))).OnlyEnforceIf(x)
        typ=str(r.drone_type)
        air.setdefault(typ,[]).append(ai); bat.setdefault(typ,[]).append(bi)
        candidates.append({"row":r,"ids":ids,"offs":offs,"x":x,"s":s,"e":e})
        xvars.append(x)
        for bid in ids: by_box[bid].append(i)

    for bid in box_ids:
        idx=by_box[bid]
        if not idx: raise RuntimeError(f"pool misses {bid}")
        m.Add(sum(xvars[i] for i in idx)==1)

    m.Add(sum(xvars)==int(k))

    for typ,ints in air.items():
        m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat.items():
        m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    cmax=m.NewIntVar(0,H,"cmax")
    for c in candidates:
        m.Add(cmax>=c["e"]).OnlyEnforceIf(c["x"])

    for typ in air:
        idx=[i for i,c in enumerate(candidates) if str(c["row"].drone_type)==typ]
        terms=[]
        for i in idx:
            d=int(math.ceil(float(candidates[i]["row"].duration_s)*q2.SCALE-1e-9))
            terms.append(d*xvars[i])
        if terms: m.Add(sum(terms)<=len(data["aircraft"][typ])*cmax)

    late_vars=[]
    for bid in box_ids:
        b=bm[bid]
        if b.get("物资类型")=="医疗物资": continue
        due=int(round(float(b["期望送达时间（s）"])*q2.SCALE))
        lv=m.NewIntVar(0,H,f"late_{bid}")
        for i in by_box[bid]:
            off=int(math.ceil(candidates[i]["offs"][bid]*q2.SCALE-1e-9))
            m.Add(lv>=candidates[i]["s"]+off-due).OnlyEnforceIf(xvars[i])
        late_vars.append(lv)
    total_late=sum(late_vars) if late_vars else 0
    if late_cap is not None: m.Add(total_late<=int(late_cap))

    if hint is not None and len(hint):
        h={str(r.candidate_id):float(r.start_s) for _,r in hint.iterrows()}
        for i,c in enumerate(candidates):
            cid=str(c["row"].candidate_id)
            if cid in h:
                m.AddHint(xvars[i],1)
                m.AddHint(c["s"],int(round(h[cid]*q2.SCALE)))
            else:
                m.AddHint(xvars[i],0)

    if objective=="lateness": m.Minimize(total_late)
    else: m.Minimize(cmax)
    return m,candidates,cmax,total_late,bm

def reconstruct(data,candidates,solver,bm):
    rows=[]
    for c in candidates:
        if solver.Value(c["x"]):
            r=c["row"].to_dict()
            r["start_s"]=solver.Value(c["s"])/q2.SCALE
            r["return_s"]=solver.Value(c["e"])/q2.SCALE
            rows.append(r)
    sch=pd.DataFrame(rows).sort_values(["start_s","return_s"]).reset_index(drop=True)
    sch["battery_end_s"]=sch.apply(lambda r: float(r.start_s)+math.ceil((float(r.duration_s)+float(r.charge_s))*q2.SCALE-1e-9)/q2.SCALE,axis=1)

    def color(g,end_col,labels):
        active={z:0.0 for z in labels}; ans={}
        for idx,r in g.sort_values("start_s").iterrows():
            free=[z for z,t in active.items() if t<=float(r.start_s)+1e-9]
            if not free: raise RuntimeError(f"color failed {end_col}")
            z=min(free,key=lambda a:active[a]); ans[idx]=z; active[z]=float(r[end_col])
        return ans

    ac={}; bt={}
    for typ,g in sch.groupby("drone_type"):
        ac.update(color(g,"return_s",list(data["aircraft"][typ])))
        bt.update(color(g,"battery_end_s",[f"{typ}-B{i+1:02d}" for i in range(int(data["batteries"][typ]))]))
    sch["aircraft"]=pd.Series(ac); sch["battery"]=pd.Series(bt)
    sch["sortie"]=[f"Q2K-S{i+1:03d}" for i in range(len(sch))]

    dl=[]
    for _,r in sch.iterrows():
        offs=json.loads(r.delivery_offsets)
        for bid,off in offs.items():
            b=bm[str(bid)]; ct=float(r.start_s)+float(off)
            hd=q2.box_hard_deadline(b)
            soft=None if b.get("物资类型")=="医疗物资" else float(b["期望送达时间（s）"])
            dl.append({"box_id":str(bid),"sortie":r.sortie,"completion_s":ct,
                       "hard_deadline_s":hd,"hard_ok":hd is None or ct<=hd+1e-6,
                       "soft_deadline_s":soft,
                       "soft_lateness_s":0.0 if soft is None else max(0.0,ct-soft)})
    return sch,pd.DataFrame(dl)

def phase(data,pool,k,obj,limit,late_cap=None,hint=None):
    m,cands,cmax,total_late,bm=build(data,pool,k,obj,late_cap,hint)
    s=cp_model.CpSolver(); s.parameters.max_time_in_seconds=float(limit); s.parameters.num_search_workers=8
    t=time.time(); st=s.Solve(m); elapsed=time.time()-t
    name=s.StatusName(st); feas=st in (cp_model.FEASIBLE,cp_model.OPTIMAL)
    summary={"K":k,"objective_kind":obj,"status":name,"runtime_s":elapsed,"feasible_found":bool(feas),
             "proven_optimal_current_pool":bool(st==cp_model.OPTIMAL),
             "lateness_cap_s":None if late_cap is None else late_cap/q2.SCALE}
    if not feas: return summary,None,None,None
    sch,dl=reconstruct(data,cands,s,bm)
    late_ticks=int(s.Value(total_late))
    summary.update({"makespan_s":float(sch.return_s.max()),
                    "best_bound_s":float(s.BestObjectiveBound())/q2.SCALE,
                    "soft_late_boxes":int((dl.soft_lateness_s>1e-6).sum()),
                    "soft_total_lateness_s":float(dl.soft_lateness_s.sum()),
                    "solver_total_lateness_s":late_ticks/q2.SCALE,
                    "hard_violations":int((~dl.hard_ok).sum()),
                    "boxes":int(len(dl)),"unique_boxes":int(dl.box_id.nunique()),
                    "energy_kwh":float(sch.energy_kwh.sum()),
                    "A_sorties":int((sch.drone_type=="A").sum()),
                    "B_sorties":int((sch.drone_type=="B").sum()),
                    "C_sorties":int((sch.drone_type=="C").sum())})
    return summary,sch,dl,late_ticks

def save(k,name,summary,sch,dl):
    d=OUT/f"K{k:02d}"/name; d.mkdir(parents=True,exist_ok=True)
    (d/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    if sch is not None:
        sch.to_csv(d/"schedule.csv",index=False,encoding="utf-8-sig")
        dl.to_csv(d/"deliveries.csv",index=False,encoding="utf-8-sig")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--k",type=int,required=True)
    ap.add_argument("--phase1-time",type=int,default=600)
    ap.add_argument("--phase2-time",type=int,default=360)
    ap.add_argument("--triple-limit",type=int,default=900)
    a=ap.parse_args()

    data=q2.load_inputs(); pool=q2.generate_pool(data,a.triple_limit)
    p1,s1,d1,late=phase(data,pool,a.k,"lateness",a.phase1_time)
    save(a.k,"phase1_min_lateness",p1,s1,d1)
    print(json.dumps({"phase":1,**p1},ensure_ascii=False))

    p2=None
    if s1 is not None:
        hint=s1[["candidate_id","start_s"]].copy()
        p2,s2,d2,_=phase(data,pool,a.k,"makespan",a.phase2_time,late,hint)
        save(a.k,"phase2_min_makespan",p2,s2,d2)
        print(json.dumps({"phase":2,**p2},ensure_ascii=False))

    root=OUT/f"K{a.k:02d}"
    (root/"run_summary.json").write_text(json.dumps({"phase1":p1,"phase2":p2},ensure_ascii=False,indent=2),encoding="utf-8")

if __name__=="__main__": main()
