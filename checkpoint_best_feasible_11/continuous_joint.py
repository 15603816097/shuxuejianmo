"""Continuous-time resource disjunctive MILP for the final P1 gate."""
from __future__ import annotations
import json, math, time, ast
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp, Bounds, LinearConstraint
from scipy.sparse import lil_matrix
from final_joint import strict_q3
from minimal_pipeline import load_inputs, q1, q2

ROOT=Path(__file__).resolve().parents[1]
M=100000.0

def charge_time(data,r):
    d=data["drones"][r["type"]]; soc=1-float(r["energy_kwh"])/d.energy; full={"A":1800.,"B":2400.,"C":3000.}[r["type"]]
    return full*(.65*(.9-soc)/.9+.35) if soc<.9 else full*.35*(1-soc)/.1

def build_model(data,tasks,cert,R,objective="zero",fixed=None,violation_cap=None):
    n=len(tasks); boxes={r["货箱编号"]:r for r in data["boxes"]}; starts=[]; box_task=[]; deadlines=[]
    for i,r in tasks.iterrows():
        ids=r["box_ids"] if isinstance(r["box_ids"],list) else ast.literal_eval(str(r["box_ids"]))
        for bid in ids:
            b=boxes[bid]; box_task.append(i); vals=[]
            if b.get("物资类型")=="医疗物资": vals.append(float(b["期望送达时间（s）"]))
            if b.get("是否首批保障")=="是": vals.append(float(b["首批截止时间（s）"]))
            deadlines.append(min(vals) if vals else M)
    # variable registry
    names=[]; lb=[]; ub=[]; integ=[]; c=[]
    def var(name,lo=0,hi=1,integer=0,cost=0): names.append(name); lb.append(lo); ub.append(hi); integ.append(integer); c.append(cost); return len(names)-1
    sidx=[var(f"s_{i}",0,M,0) for i in range(n)]
    y_air={}; y_bat={}; y_rel={};
    for i,r in tasks.iterrows():
        for u in data["aircraft"][r["type"]]: y_air[i,u]=var(f"air_{i}_{u}",0,1,1)
        for u in range(data["batteries"][r["type"]]): y_bat[i,(r["type"],u)]=var(f"bat_{i}_{u}",0,1,1)
        if bool(cert.iloc[i]["relay_needed"]):
            for u in range(R): y_rel[i,u]=var(f"rel_{i}_{u}",0,1,1)
    tard=[]; tard_sortie=[]; late=[]
    for k in range(len(deadlines)):
        tard.append(var(f"tardy_box_{k}",0,0 if objective=="zero" and violation_cap is None else 1,1,1 if objective=="boxes" else 0))
        late.append(var(f"late_{k}",0,0 if objective=="zero" else M,0,1 if objective=="late" else 0))
    for i in range(n): tard_sortie.append(var(f"tardy_task_{i}",0,1,1,1 if objective=="sorties" else 0))
    lmax=var("max_late",0,M,0,1 if objective=="maxlate" else 0); cmax=var("makespan",0,M,0,1 if objective=="makespan" else 0)
    rows=[]; lbs=[]; ubs=[]
    def add(co,lo=-np.inf,hi=np.inf): rows.append(co); lbs.append(lo); ubs.append(hi)
    # assignment and deadline/lateness constraints
    for i,r in tasks.iterrows():
        add({j:1 for (ii,_),j in y_air.items() if ii==i},1,1); add({j:1 for (ii,_),j in y_bat.items() if ii==i},1,1)
        if bool(cert.iloc[i]["relay_needed"]): add({j:1 for (ii,_),j in y_rel.items() if ii==i},1,1)
        if bool(cert.iloc[i]["relay_needed"]): add({sidx[i]:1},float(data["relay"]["prep_s"]),np.inf)
    k=0
    for i,r in tasks.iterrows():
        do=float(r["delivery_s"]-r["start_s"]); ro=float(r["return_s"]-r["start_s"])
        ids=r["box_ids"] if isinstance(r["box_ids"],list) else ast.literal_eval(str(r["box_ids"]))
        for _ in ids:
            # s + delivery offset - deadline <= M*tardy
            add({sidx[i]:1,tard[k]:-M},-np.inf,deadlines[k]-do)
            add({late[k]:-1,sidx[i]:1},-np.inf,deadlines[k]-do)
            add({late[k]:1,sidx[i]:-1},-np.inf,do-deadlines[k]+M) # redundant upper guard
            add({tard_sortie[i]:-1,tard[k]:1},-np.inf,0)
            add({lmax:-1,late[k]:1},-np.inf,0)
            k+=1
        add({cmax: -1,sidx[i]:1},-np.inf,-ro)
    if violation_cap is not None:
        add({j:1 for j in tard},-np.inf,float(violation_cap))
    if fixed:
        for co,lo,hi in fixed: add(co,lo,hi)
    # pairwise non-overlap helper, with offsets before/after s
    def pair(i,j,yi,yj,off_i,dur_i,off_j,dur_j):
        z=var(f"ord_{len(rows)}",0,1,1)
        # Correct disjunction for half-open intervals.  When yi=yj=1,
        # z=1 enforces i before j and z=0 enforces j before i.  If either
        # task is not assigned to this unit, the assignment terms relax it.
        add({sidx[i]:1,sidx[j]:-1,z:M,yi:M,yj:M},-np.inf,3*M-off_i-dur_i+off_j)
        add({sidx[j]:1,sidx[i]:-1,z:-M,yi:M,yj:M},-np.inf,2*M-off_j-dur_j+off_i)
    def add_pairs(items,off,dur):
        for a in range(len(items)):
            for b in range(a+1,len(items)):
                i,yi=items[a]; j,yj=items[b]; pair(i,j,yi,yj,off(i),dur(i),off(j),dur(j))
    for typ in ("A","B","C"):
        units=data["aircraft"][typ];
        for u in units: add_pairs([(i,y_air[i,u]) for i,r in tasks.iterrows() if r["type"]==typ],lambda i:0,lambda i:float(tasks.iloc[i]["return_s"]-tasks.iloc[i]["start_s"]))
        for u in range(data["batteries"][typ]): add_pairs([(i,y_bat[i,(typ,u)]) for i,r in tasks.iterrows() if r["type"]==typ],lambda i:0,lambda i:float(tasks.iloc[i]["return_s"]-tasks.iloc[i]["start_s"])+charge_time(data,tasks.iloc[i]))
    for u in range(R): add_pairs([(i,y_rel[i,u]) for i in range(n) if (i,u) in y_rel],lambda i:-data["relay"]["prep_s"],lambda i:float(tasks.iloc[i]["return_s"]-tasks.iloc[i]["start_s"])+data["relay"]["prep_s"]+data["relay"]["turn_s"])
    for service in tasks["service"].unique():
        ids=[i for i,r in tasks.iterrows() if r["service"]==service]
        for a in range(len(ids)):
            for b in range(a+1,len(ids)):
                i,j=ids[a],ids[b]; z=var(f"service_ord_{i}_{j}",0,1,1)
                oi=float(tasks.iloc[i]["delivery_s"]-tasks.iloc[i]["start_s"]); oj=float(tasks.iloc[j]["delivery_s"]-tasks.iloc[j]["start_s"])
                di=float(tasks.iloc[i]["handoff_s"]); dj=float(tasks.iloc[j]["handoff_s"])
                add({sidx[i]:1,sidx[j]:-1,z:M},-np.inf,M-oi-di+oj)
                add({sidx[j]:1,sidx[i]:-1,z:-M},-np.inf,M-oj-dj+oi)
    A=lil_matrix((len(rows),len(names)))
    for ri,co in enumerate(rows):
        for j,v in co.items(): A[ri,j]=v
    return np.array(c),np.array(integ),Bounds(np.array(lb),np.array(ub)),LinearConstraint(A.tocsr(),np.array(lbs),np.array(ubs)),names

def _solve(data,tasks,cert,stage,obj,limit=20,fixed=None,violation_cap=None,R=2):
    c,integ,bounds,cons,names=build_model(data,tasks,cert,R,obj,fixed=fixed,violation_cap=violation_cap)
    t0=time.time(); res=milp(c,integrality=integ,bounds=bounds,constraints=cons,options={"time_limit":limit})
    return res,names,{"stage":stage,"objective_name":obj,"status":int(res.status),"message":str(res.message),"objective":None if res.fun is None else float(res.fun),"lower_bound":getattr(res,"mip_dual_bound",None),"upper_bound":None if res.fun is None else float(res.fun),"mip_gap":getattr(res,"mip_gap",None),"runtime_s":time.time()-t0,"variables":len(names),"constraints":int(cons.A.shape[0])}

def _objective_expr(names,obj):
    if obj=="boxes": return {i:1.0 for i,n in enumerate(names) if n.startswith("tardy_box_")}
    if obj=="sorties": return {i:1.0 for i,n in enumerate(names) if n.startswith("tardy_task_")}
    if obj=="late": return {i:1.0 for i,n in enumerate(names) if n.startswith("late_")}
    if obj=="maxlate": return {i:1.0 for i,n in enumerate(names) if n=="max_late"}
    if obj=="makespan": return {i:1.0 for i,n in enumerate(names) if n=="makespan"}
    return {}

def _schedule_from(tasks,res,names):
    s=tasks.copy(); starts=[float(res.x[names.index(f"s_{i}")]) for i in range(len(tasks))]
    s["start_s"]=starts; s["delivery_s"]=s["start_s"]+(tasks["delivery_s"].to_numpy()-tasks["start_s"].to_numpy()); s["return_s"]=s["start_s"]+(tasks["return_s"].to_numpy()-tasks["start_s"].to_numpy()); return s

def _write_outputs(sched,data,cert):
    sched.to_csv(ROOT/"results/final_continuous_schedule.csv",index=False); deliveries=[]; events=[]
    for _,r in sched.iterrows():
        ids=r["box_ids"] if isinstance(r["box_ids"],list) else ast.literal_eval(str(r["box_ids"]))
        for bid in ids: deliveries.append({"box_id":bid,"sortie":r["sortie"],"arrival_s":float(r["delivery_s"]-r["handoff_s"]),"completion_s":float(r["delivery_s"])})
        events.append({"sortie":r["sortie"],"resource":f"transport:{r['aircraft']}","start_s":r["start_s"],"end_s":r["return_s"]})
        events.append({"sortie":r["sortie"],"resource":f"service:{r['service']}","start_s":r["delivery_s"]-r["handoff_s"],"end_s":r["delivery_s"]})
    pd.DataFrame(deliveries).to_csv(ROOT/"results/final_continuous_deliveries.csv",index=False); pd.DataFrame(events).to_csv(ROOT/"results/final_continuous_resource_events.csv",index=False)

def main():
    data=load_inputs(); _,batches=q1(data); base,_=q2(data,batches); _,c_batches=q1(data); regrouped,_=q2(data,c_batches); cert=pd.read_csv(ROOT/"results/final_q3_continuous_certification.csv")
    cert_ok=bool((cert["relay_feasible"].astype(bool)&(cert["segment1_pixels"]>0)&(cert["segment2_pixels"]>0)).all())
    golden=pd.read_csv(ROOT/"results/golden_grid60_schedule.csv")
    # A freezes the Golden groups; B rebuilds the same contract groups through
    # q1/q2; C opens only q1/q2's contract-approved regrouping decoder.
    stages={"A":golden,"B":base,"C":regrouped}; records=[]; chosen=None; best_incumbent=None
    for stage,tasks in stages.items():
        res,names,rec=_solve(data,tasks,cert,stage,"zero",limit=20); records.append(rec)
        if rec["status"]==0: chosen=(res,names,tasks); break
    # Strict lexicographic diagnostic after zero-violation infeasibility.
    lex=[]; tasks=base
    fixed=[]; prior_obj=None; prior_value=None
    for obj in ("boxes","sorties","late","maxlate","makespan"):
        if prior_obj is not None and prior_value is not None:
            # Fix the preceding lexicographic optimum before solving the next level.
            fixed.append((_objective_expr(last_names,prior_obj),float(prior_value),float(prior_value)))
        res,names,rec=_solve(data,tasks,cert,"A",obj,limit=25,fixed=fixed); lex.append(rec)
        if res.x is not None: best_incumbent=(res,names,tasks,rec)
        if rec["status"]==0 and rec["mip_gap"] is not None and rec["mip_gap"]==0:
            prior_obj=obj; prior_value=rec["objective"]; last_names=names
            chosen=(res,names,tasks)
        else: break
    if chosen: _write_outputs(_schedule_from(chosen[2],chosen[0],chosen[1]),data,cert)
    elif best_incumbent: _write_outputs(_schedule_from(best_incumbent[2],best_incumbent[0],best_incumbent[1]),data,cert)
    else:
        pd.DataFrame().to_csv(ROOT/"results/final_continuous_schedule.csv",index=False); pd.DataFrame().to_csv(ROOT/"results/final_continuous_deliveries.csv",index=False); pd.DataFrame().to_csv(ROOT/"results/final_continuous_resource_events.csv",index=False)
    allrec=records+lex; pd.DataFrame(allrec).to_csv(ROOT/"results/continuous_stage_comparison.csv",index=False)
    zero_ok=any(r["objective_name"]=="zero" and r["status"]==0 for r in records); strict=bool(lex and lex[-1].get("mip_gap")==0)
    summary={"R":2,"q3_certification_all_candidates_valid":cert_ok,"stages":records,"lexicographic":lex,"zero_violation":zero_ok,"strict_lexicographic":strict,"best_incumbent_boxes":None if best_incumbent is None else best_incumbent[3].get("objective"),"p1":"PASS" if zero_ok or strict else "FAIL","note":"Continuous event-time MILP; parity evaluator unchanged."}
    (ROOT/"results/final_continuous_feasibility.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"); (ROOT/"logs/continuous_final_milp.log").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
