"""K=18 zero-hard-deadline feasibility and K=19 final audit."""
from __future__ import annotations
import ast, json, time
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp

from minimal_pipeline import load_inputs, q1, q2
from continuous_joint import _solve
from parity_evaluators import eval_A, charge_A, _ids
from q2_sortie_count_sensitivity import pool_peak, TOL

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"; LOG = ROOT / "logs"


def deadlines(box):
    x=[]
    if box.get("物资类型") == "医疗物资": x.append(float(box["期望送达时间（s）"]))
    if box.get("是否首批保障") == "是": x.append(float(box["首批截止时间（s）"]))
    return x


def decode(tasks, res, names):
    out=tasks.copy(); out["start_s"]=[float(res.x[names.index(f"s_{i}")]) for i in range(len(tasks))]
    aircraft=[]; battery=[]
    for i,r in tasks.iterrows():
        au=[u for u in [*r"" ]]
        candidates=[(n,res.x[j]) for j,n in enumerate(names) if n.startswith(f"air_{i}_") and res.x[j]>.5]
        aircraft.append(candidates[0][0].split(f"air_{i}_",1)[1] if candidates else r.aircraft)
        bc=[(n,res.x[j]) for j,n in enumerate(names) if n.startswith(f"bat_{i}_") and res.x[j]>.5]
        if bc:
            idx=int(bc[0][0].rsplit("_",1)[1]); battery.append(f"{r.type}-B{idx+1:02d}")
        else: battery.append(r.battery)
    out["aircraft"]=aircraft; out["battery"]=battery
    out["delivery_s"]=out.start_s.to_numpy()+(tasks.delivery_s.to_numpy()-tasks.start_s.to_numpy())
    out["return_s"]=out.start_s.to_numpy()+(tasks.return_s.to_numpy()-tasks.start_s.to_numpy())
    return out


def timeliness(data, schedule):
    bm={str(x["货箱编号"]):x for x in data["boxes"]}; rows=[]
    for _,r in schedule.iterrows():
        for bid in _ids(r.box_ids):
            b=bm[bid]; hd=deadlines(b); hard=bool(hd and any(float(r.delivery_s)>d+TOL for d in hd))
            soft_dead=float(b["期望送达时间（s）"]) if b.get("物资类型")!="医疗物资" else np.nan
            sl=max(0.0,float(r.delivery_s)-soft_dead) if np.isfinite(soft_dead) else 0.0
            rows.append({"box_id":bid,"sortie":r.sortie,"completion_s":r.delivery_s,"hard_violation":hard,"soft_late":bool(np.isfinite(soft_dead) and sl>TOL),"soft_lateness_s":sl,"hard_deadline_s":min(hd) if hd else np.nan,"soft_deadline_s":soft_dead})
    return pd.DataFrame(rows)


def solve_k18(data):
    _, batches=q1(data); tasks,_=q2(data,batches)
    # Q2-only feasibility: all relay variables/candidates are disabled.
    cert=pd.read_csv(RES/"final_q3_continuous_certification.csv").copy(); cert["relay_needed"]=False; cert["relay_feasible"]=False
    t0=time.time(); res,names,rec=_solve(data,tasks,cert,"K18_zero_hard","zero",limit=120,violation_cap=0,R=0)
    rec.update({"solver_status_code":int(res.status),"solver_status":"OPTIMAL" if res.status==0 else ("INFEASIBLE" if res.status==2 else "TIME_LIMIT_OR_UNKNOWN"),"node_count":getattr(res,"mip_node_count",None),"primal_bound":None if res.fun is None else float(res.fun),"dual_bound":getattr(res,"mip_dual_bound",None),"runtime_s":time.time()-t0,"strict_infeasible":bool(res.status==2),"q3_relay_in_constraints":False})
    (RES/"q2_K18_zero_hard_feasibility.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
    (LOG/"q2_K18_zero_hard.log").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
    if res.x is None: return rec,None,None,None
    schedule=decode(tasks,res,names)
    t,d,e=eval_A(schedule,data); d2=timeliness(data,schedule)
    # Q2 output excludes relay event/resource from the event ledger.
    e=e[e.resource.astype(str)!="relay_pool"].copy()
    schedule.to_csv(RES/"q2_K18_zero_hard_schedule.csv",index=False)
    d2.to_csv(RES/"q2_K18_zero_hard_deliveries.csv",index=False)
    e.to_csv(RES/"q2_K18_zero_hard_resource_events.csv",index=False)
    rec["hard_violation_boxes"]=int(d2.hard_violation.sum()); rec["hard_violation_sorties"]=int(d2.loc[d2.hard_violation,"sortie"].nunique())
    rec["soft_late_boxes"]=int(d2.soft_late.sum()); rec["soft_total_lateness_s"]=float(d2.soft_lateness_s.sum()); rec["soft_max_lateness_s"]=float(d2.soft_lateness_s.max()); rec["weighted_timeliness_penalty"]=float(d2.soft_lateness_s.sum()); rec["makespan_s"]=float(schedule.return_s.max()); rec["transport_energy_kwh"]=float(schedule.energy_kwh.sum())
    rec["B_sorties"]=int((schedule.type=="B").sum()); rec["C_sorties"]=int((schedule.type=="C").sum()); rec["sorties"]=int(len(schedule)); rec["boxes"]=int(len(d2)); rec["single_point_sorties"]=int(len(schedule)); rec["multi_point_sorties"]=0; rec["max_service_areas_per_sortie"]=1
    rec["transport_peak"]=pool_peak(e,lambda x:x.resource.astype(str).str.startswith("transport:")); rec["B_battery_peak"]=pool_peak(e,lambda x:x.resource.astype(str).str.startswith("battery:B-")); rec["C_battery_peak"]=pool_peak(e,lambda x:x.resource.astype(str).str.startswith("battery:C-")); rec["service_peak_global"]=pool_peak(e,lambda x:x.resource.astype(str).str.startswith("service:")); rec["service_peak_per_station"]=max((pool_peak(g,lambda x:pd.Series(True,index=x.index)) for _,g in e[e.resource.astype(str).str.startswith("service:")].groupby("resource")),default=0)
    (RES/"q2_K18_zero_hard_feasibility.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
    return rec,schedule,d2,e


def audit_k19(data):
    schedule=pd.read_csv(RES/"q2_sensitivity_19_27"/"q2_sens_K19_schedule.csv")
    t,d,e=eval_A(schedule,data); d2=timeliness(data,schedule)
    # resource events are evaluated with Q2's physical resources; relay is not a Q2 constraint.
    e=e[e.resource.astype(str)!="relay_pool"].copy()
    def service_peak(g): return pool_peak(g,lambda x: pd.Series(True,index=x.index))
    rows=[]
    transport_observed=pool_peak(e,lambda x:x.resource.astype(str).str.startswith("transport:"))
    rows += [{"resource":"transport","scope":"global","inventory":8,"observed_peak":transport_observed,"feasible":transport_observed<=8}]
    for typ,inv in [("B",data["batteries"]["B"]),("C",data["batteries"]["C"])]:
        pk=pool_peak(e,lambda x,typ=typ:x.resource.astype(str).str.startswith(f"battery:{typ}-")); rows.append({"resource":f"{typ}_battery","scope":"global","inventory":inv,"observed_peak":pk,"feasible":pk<=inv})
    svc=e[e.resource.astype(str).str.startswith("service:")]
    rows.append({"resource":"service","scope":"global_diagnostic","inventory":15,"observed_peak":service_peak(svc),"feasible":service_peak(svc)<=15})
    service_rows=[]
    for name,g in svc.groupby("resource"):
        pk=service_peak(g); rows.append({"resource":"service","scope":name,"inventory":1,"observed_peak":pk,"feasible":pk<=1})
        service_rows.append({"service_area":name.split(":",1)[1],"inventory":1,"observed_peak":pk,"feasible":pk<=1})
    pd.DataFrame(service_rows).to_csv(RES/"q2_K19_service_audit.csv",index=False)
    pd.DataFrame(rows).to_csv(RES/"q2_K19_final_resource_audit.csv",index=False)
    batt=[]
    for _,r in schedule.iterrows():
        charge=charge_A(data,r); end=float(r.return_s)+charge
        batt.append({"record_type":"interval","battery_id":r.battery,"battery_type":r.type,"sortie":r.sortie,"discharge_start_s":r.start_s,"discharge_end_s":r.return_s,"recharge_start_s":r.return_s,"recharge_end_s":end,"available_again_s":end})
    ba=pd.DataFrame(batt)
    # independent type peak over full battery occupancy [discharge,recharge_end)
    br=[]
    for typ,inv in [("B",data["batteries"]["B"]),("C",data["batteries"]["C"])]:
        z=ba[ba.battery_type==typ]; pts=[]
        for _,q in z.iterrows(): pts += [(round(q.discharge_start_s/TOL)*TOL,1),(round(q.available_again_s/TOL)*TOL,-1)]
        cur=pk=0
        for _,dd in sorted(pts,key=lambda x:(x[0],x[1])): cur+=dd; pk=max(pk,cur)
        br.append({"record_type":"type_summary","battery_id":"","battery_type":typ,"sortie":"","discharge_start_s":np.nan,"discharge_end_s":np.nan,"recharge_start_s":np.nan,"recharge_end_s":np.nan,"available_again_s":np.nan,"inventory":inv,"observed_peak_with_recharge":pk,"feasible":pk<=inv})
    for row in batt:
        row.update({"inventory":np.nan,"observed_peak_with_recharge":np.nan,"feasible":np.nan})
    pd.concat([ba, pd.DataFrame(br)], ignore_index=True).to_csv(RES/"q2_K19_battery_audit.csv",index=False)
    metrics={"K":19,"hard_violation_boxes":int(d2.hard_violation.sum()),"hard_violation_sorties":int(d2.loc[d2.hard_violation,"sortie"].nunique()),"soft_late_boxes":int(d2.soft_late.sum()),"soft_total_lateness_s":float(d2.soft_lateness_s.sum()),"soft_max_lateness_s":float(d2.soft_lateness_s.max()),"weighted_timeliness_penalty":float(d2.soft_lateness_s.sum()),"makespan_s":float(schedule.return_s.max()),"transport_energy_kwh":float(schedule.energy_kwh.sum()),"B_sorties":int((schedule.type=="B").sum()),"C_sorties":int((schedule.type=="C").sum()),"single_point_sorties":19,"multi_point_sorties":0,"max_service_areas_per_sortie":1,"sorties":len(schedule),"boxes":len(d2)}
    pd.DataFrame([metrics]).to_csv(RES/"q2_K19_final_metrics.csv",index=False)
    return metrics


def main():
    data=load_inputs(); rec,s,d,e=solve_k18(data); m=audit_k19(data)
    print(json.dumps({"K18":rec,"K19":m},ensure_ascii=False,indent=2))


if __name__=="__main__": main()
