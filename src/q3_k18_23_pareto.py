"""Joint Q3 grouping + timing search with four relay service windows.

Candidate batches are generated per service from deadline-ordered contiguous
subsets (plus hard/soft groups), so grouping is a decision variable rather than
being inherited from Q2.  The CP-SAT master jointly selects an exact cover of
all 80 boxes, schedules transport resources, and forces relay-required blocks
into four friend-paper geographic service windows.  The resulting schedule is
only a candidate: it is immediately intended for strict 1596-point hypergraph
re-certification and the separate four-sortie validator.
"""
from __future__ import annotations
import argparse, json, math
from collections import defaultdict
from pathlib import Path
import pandas as pd
from ortools.sat.python import cp_model

import q3_final_pipeline as pipe
from minimal_pipeline import load_inputs, route_stats

S=10
TOL=1e-9
CLUSTER={
 "S003":"G1","S005":"G1","S007":"G1","S015":"G1",
 "S004":"G2","S008":"G2","S009":"G2",
 "S002":"G3","S012":"G3","S013":"G3",
 "S010":"G4","S014":"G4",
}
DIRECT={"S001","S006","S011"}

def I(x): return int(round(float(x)*S))
def is_hard(b):
    return str(b.get("物资类型",""))=="医疗物资" or str(b.get("是否首批保障",""))=="是"
def hard_deadline(b):
    ds=[]
    if str(b.get("物资类型",""))=="医疗物资": ds.append(float(b["期望送达时间（s）"]))
    if str(b.get("是否首批保障",""))=="是": ds.append(float(b["首批截止时间（s）"]))
    return min(ds) if ds else None
def order_key(b):
    h=hard_deadline(b)
    return (0 if h is not None else 1, h if h is not None else float(b.get("期望送达时间（s）",14000)), str(b["货箱编号"]))

def charge_time(data,typ,energy):
    d=data["drones"][typ]; soc=1-float(energy)/float(d.energy)
    full={"A":1800.0,"B":2400.0,"C":3000.0}[typ]
    if soc<0.9: return full*(0.65*(0.9-soc)/0.9+0.35)
    return full*0.35*(1-soc)/0.1

def candidate_sets(rows):
    z=sorted(rows,key=order_key); n=len(z); seen=set(); out=[]
    # all contiguous deadline-ordered segments
    for i in range(n):
        for j in range(i+1,n+1):
            ids=tuple(str(x["货箱编号"]) for x in z[i:j])
            if ids not in seen: seen.add(ids); out.append(ids)
    # explicit hard-only / soft-only groups and prefixes/suffixes around them
    hard=[str(x["货箱编号"]) for x in z if is_hard(x)]
    soft=[str(x["货箱编号"]) for x in z if not is_hard(x)]
    for group in (hard,soft):
        if group:
            ids=tuple(group)
            if ids not in seen: seen.add(ids); out.append(ids)
    # singletons guaranteed
    for x in z:
        ids=(str(x["货箱编号"]),)
        if ids not in seen: seen.add(ids); out.append(ids)
    return out

def build_candidates(data):
    bm={str(x["货箱编号"]):x for x in data["boxes"]}
    bys=defaultdict(list)
    for b in data["boxes"]: bys[str(b["服务区编号"])].append(b)
    out=[]; cid=0
    for service in sorted(bys):
        sets=candidate_sets(bys[service])
        for ids in sets:
            rows=[bm[x] for x in ids]
            # keep up to two best feasible types by energy, retaining resource flexibility
            feasible=[]
            for typ in data["drones"]:
                st=route_stats(data,typ,service,rows)
                if st["safe"]: feasible.append((float(st["energy_kwh"]),typ,st))
            feasible=sorted(feasible,key=lambda x:(x[0],x[1]))[:2]
            for _,typ,st in feasible:
                tl=pipe.build_authoritative_timeline(data,(service,),typ,list(ids))
                dl=pipe.deadline_ledger(data,tl)
                hard_rows=[x for x in dl if x["hard"]]
                latest=14000.0
                if hard_rows:
                    latest=min(float(x["deadline_s"])-float(x["delivery_time_s"]) for x in hard_rows)
                if latest < -1e-6: continue
                blocks=pipe.direct_and_relay_blocks(data,tl)
                cid+=1
                out.append({
                  "candidate_id":f"J{cid:05d}","service":service,"visit_order":service,
                  "cluster":CLUSTER.get(service,"DIRECT"),"drone_type":typ,"type":typ,
                  "box_ids":list(ids),"duration_s":float(tl["makespan_s"]),
                  "latest_start_s":max(0.0,float(latest)),"charge_s":charge_time(data,typ,st["energy_kwh"]),
                  "transport_energy_kwh":float(st["energy_kwh"]),
                  "relay_blocks":[(float(b["start_s"]),float(b["end_s"])) for b in blocks],
                  "hard_count":sum(is_hard(bm[x]) for x in ids),
                  "soft_deadlines":[(str(x["box_id"]),float(x["delivery_time_s"]),float(x["deadline_s"])) for x in dl if not x["hard"]]
                })
    return out

def solve(exact_routes=18,time_limit=240):
    data=load_inputs(); cands=build_candidates(data)
    bm={str(x["货箱编号"]):x for x in data["boxes"]}
    all_boxes=sorted(bm)
    bybox=defaultdict(list)
    for i,c in enumerate(cands):
        for b in c["box_ids"]: bybox[b].append(i)
    missing=[b for b in all_boxes if not bybox[b]]
    if missing: raise RuntimeError(f"candidate pool missing boxes: {missing}")

    H=I(18000); m=cp_model.CpModel()
    x=[]; s=[]; e=[]; air=defaultdict(list); bat=defaultdict(list)
    for i,c in enumerate(cands):
        xi=m.NewBoolVar(f"x{i}"); si=m.NewIntVar(0,H,f"s{i}"); ei=m.NewIntVar(0,H*2,f"e{i}")
        d=I(c["duration_s"]); m.Add(ei==si+d)
        m.Add(si<=int(math.floor(c["latest_start_s"]*S+1e-9))).OnlyEnforceIf(xi)
        m.Add(si==0).OnlyEnforceIf(xi.Not())
        iv=m.NewOptionalIntervalVar(si,d,ei,xi,f"air{i}")
        air[c["drone_type"]].append(iv)
        bd=I(c["duration_s"]+c["charge_s"]); be=m.NewIntVar(0,H*2,f"be{i}"); m.Add(be==si+bd)
        biv=m.NewOptionalIntervalVar(si,bd,be,xi,f"bat{i}")
        bat[c["drone_type"]].append(biv)
        x.append(xi); s.append(si); e.append(ei)

    for b in all_boxes: m.Add(sum(x[i] for i in bybox[b])==1)
    m.Add(sum(x)==exact_routes)
    for typ,ints in air.items(): m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat.items(): m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    # Ordinary-box delivery timeliness on each selected candidate.
    late_flags=[]; late_amounts=[]
    for i,c in enumerate(cands):
        for j,(bid,delivery_off,deadline) in enumerate(c.get("soft_deadlines",[])):
            lv=m.NewIntVar(0,H*2,f"late_{i}_{j}")
            fl=m.NewBoolVar(f"lateflag_{i}_{j}")
            due=I(deadline); off=I(delivery_off)
            m.Add(lv >= s[i]+off-due).OnlyEnforceIf(x[i])
            m.Add(lv == 0).OnlyEnforceIf(x[i].Not())
            m.Add(fl == 0).OnlyEnforceIf(x[i].Not())
            m.Add(lv == 0).OnlyEnforceIf([x[i],fl.Not()])
            m.Add(lv >= 1).OnlyEnforceIf([x[i],fl])
            m.Add(lv <= H*2*fl)
            late_flags.append(fl); late_amounts.append(lv)

    # Exactly one relay service window per friend-paper geographic cluster.
    ws={g:m.NewIntVar(0,H*2,f"ws_{g}") for g in ("G1","G2","G3","G4")}
    we={g:m.NewIntVar(0,H*2,f"we_{g}") for g in ("G1","G2","G3","G4")}
    wdur={g:m.NewIntVar(1,H*2,f"wd_{g}") for g in ("G1","G2","G3","G4")}
    wiv={}
    for g in ws:
        m.Add(we[g]>=ws[g]+1); m.Add(wdur[g]==we[g]-ws[g])
        wiv[g]=m.NewIntervalVar(ws[g],wdur[g],we[g],f"win_{g}")
    m.AddCumulative(list(wiv.values()),[1,1,1,1],2)

    for i,c in enumerate(cands):
        g=c["cluster"]
        if g in ws:
            for a,b in c["relay_blocks"]:
                m.Add(ws[g] <= s[i]+I(a)).OnlyEnforceIf(x[i])
                m.Add(we[g] >= s[i]+I(b)).OnlyEnforceIf(x[i])

    # Exact K is fixed. Compare K=18..23 on the same lexicographic policy:
    # late boxes -> total lateness -> joint completion -> energy -> relay-window span.
    joint=m.NewIntVar(0,H*2,"joint")
    for i in range(len(cands)): m.Add(joint>=e[i]).OnlyEnforceIf(x[i])
    span=sum(wdur.values())
    energy_terms=[int(round(c["transport_energy_kwh"]*1000))*x[i] for i,c in enumerate(cands)]
    late_count=sum(late_flags); total_late=sum(late_amounts)
    m.Minimize(late_count*10**15 + total_late*10**7 + joint*10 + sum(energy_terms) + span)

    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=time_limit; solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    rec={"status":solver.StatusName(st),"candidate_count":len(cands),"exact_routes":exact_routes}
    selected=[]
    if st in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        for i,c in enumerate(cands):
            if solver.Value(x[i]):
                r=dict(c); r["start_s"]=solver.Value(s[i])/S; r["return_s"]=solver.Value(e[i])/S
                r["box_ids_json"]=json.dumps(r["box_ids"],ensure_ascii=False)
                selected.append(r)
        rec.update({"selected_routes":len(selected),"soft_violation_boxes":int(solver.Value(late_count)),
                    "total_soft_lateness_s":solver.Value(total_late)/S,"joint_completion_s":solver.Value(joint)/S,
                    "cluster_windows":{g:[solver.Value(ws[g])/S,solver.Value(we[g])/S] for g in ws},
                    "transport_energy_kwh":sum(r["transport_energy_kwh"] for r in selected),
                    "relay_window_span_s":solver.Value(span)/S})
    return data,cands,selected,rec

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--exact-routes",type=int,required=True); ap.add_argument("--time-limit",type=int,default=240)
    ap.add_argument("--out",default="results/q3_joint_grouping")
    a=ap.parse_args(); data,cands,sel,rec=solve(a.exact_routes,a.time_limit)
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    # candidate audit
    pd.DataFrame([{**c,"box_ids":json.dumps(c["box_ids"],ensure_ascii=False),"relay_blocks":json.dumps(c["relay_blocks"]),"soft_deadlines":json.dumps(c.get("soft_deadlines",[]),ensure_ascii=False)} for c in cands]).to_csv(out/"candidate_pool.csv",index=False,encoding="utf-8-sig")
    if sel:
        rows=[]
        for r in sel:
            rows.append({
              "candidate_id":r["candidate_id"],"visit_order":r["visit_order"],"service":r["service"],
              "drone_type":r["drone_type"],"type":r["type"],"box_ids":r["box_ids_json"],
              "duration_s":r["duration_s"],"latest_start_s":r["latest_start_s"],"charge_s":r["charge_s"],
              "transport_energy_kwh":r["transport_energy_kwh"],"start_s":r["start_s"],"return_s":r["return_s"]
            })
        pd.DataFrame(rows).to_csv(out/"selected_schedule.csv",index=False,encoding="utf-8-sig")
    (out/"summary.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(rec,ensure_ascii=False,indent=2))
if __name__=="__main__": main()
