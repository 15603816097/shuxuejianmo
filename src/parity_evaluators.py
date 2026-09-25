"""Independent parity evaluators for the frozen Golden Schedule."""
from __future__ import annotations
import ast, json
from pathlib import Path
import numpy as np
import pandas as pd
from minimal_pipeline import load_inputs

ROOT = Path(__file__).resolve().parents[1]
TOL = 1e-8

def _boxes(data): return {str(x["货箱编号"]): x for x in data["boxes"]}
def _ids(v): return list(v) if isinstance(v, list) else list(ast.literal_eval(str(v)))

def deadline_A(box):
    vals=[]
    if box.get("物资类型")=="医疗物资": vals.append(float(box["期望送达时间（s）"]))
    if box.get("是否首批保障")=="是": vals.append(float(box["首批截止时间（s）"]))
    return min(vals) if vals else None

def deadline_B(box):
    med=float(box["期望送达时间（s）"]) if box.get("物资类型")=="医疗物资" else None
    first=float(box["首批截止时间（s）"]) if box.get("是否首批保障")=="是" else None
    values=[v for v in (med,first) if v is not None]
    return min(values) if values else None

def charge_A(data,row,scale=1.0):
    typ=str(row["type"]); d=data["drones"][typ]; soc=1.0-float(row["energy_kwh"])/float(d.energy)
    full={"A":1800.0,"B":2400.0,"C":3000.0}[typ]
    raw=full*(0.65*(0.9-soc)/0.9+0.35) if soc<0.9 else full*0.35*(1.0-soc)/0.1
    return max(0.0,raw)*float(scale)

def charge_B(data,row,scale=1.0):
    typ=str(row["type"]); soc=float(row["soc_return"]); full={"A":1800.0,"B":2400.0,"C":3000.0}[typ]
    if soc<0.9: duration=full*(0.35+0.65*(0.9-soc)/0.9)
    else: duration=full*0.35*(1.0-soc)/0.1
    return max(0.0,duration)*float(scale)

def _relay(data,cert,r):
    q=cert.loc[cert["sortie"].astype(str)==str(r["sortie"])]
    return q.iloc[0] if len(q) else None

def _event(resource,task,start,end,source):
    return {"resource":resource,"task_id":str(task["sortie"]),"start_s":float(start),"end_s":float(end),"source_function":source}

def sweep(events):
    peaks={}
    for resource,grp in events.groupby("resource") if len(events) else []:
        pts=[]
        for _,e in grp.iterrows(): pts.extend([(float(e.start_s),1),(float(e.end_s),-1)])
        level=peak=0
        for _,delta in sorted(pts,key=lambda x:(x[0],0 if x[1]<0 else 1)):
            level+=delta; peak=max(peak,level)
        peaks[str(resource)]=peak
    return peaks

def eval_A(schedule,data,service_delta=0.0,charge_scale=1.0):
    boxes=_boxes(data); cert=pd.read_csv(ROOT/"results/final_q3_continuous_certification.csv"); tasks=[]; deliveries=[]; events=[]
    for _,r in schedule.iterrows():
        ids=_ids(r.box_ids); start=float(r.start_s); completion=float(r.delivery_s); handoff=float(r.handoff_s)+float(service_delta); arrival=completion-handoff; ret=float(r.return_s)
        ds=[deadline_A(boxes[x]) for x in ids]; flags=[d is not None and completion>d for d in ds]
        for bid,d,flag in zip(ids,ds,flags): deliveries.append({"box_id":bid,"task_id":str(r.sortie),"arrival_time":arrival,"completion_time":completion,"hard_deadline":d,"late_flag":bool(flag),"lateness":0.0 if d is None else max(0.0,completion-d),"source_function":"eval_A.deadline_A"})
        cr=_relay(data,cert,r); need=bool(cr is not None and bool(cr.relay_needed)); relay_energy=float(data["relay"]["power_kw"])*float(cr.total_distance_m)/float(data["relay"]["speed"])/3600.0 if need and bool(cr.relay_feasible) else 0.0
        charge=charge_A(data,r,charge_scale)
        events.extend([_event(f"transport:{r.aircraft}",r,start,ret,"eval_A.transport_interval"),_event(f"service:{r.service}",r,arrival,completion,"eval_A.service_interval"),_event(f"battery:{r.battery}",r,start,ret+charge,"eval_A.battery_interval")])
        if need: events.append(_event("relay_pool",r,start-float(data["relay"]["prep_s"]),ret+float(data["relay"]["turn_s"]),"eval_A.relay_interval"))
        hard=min((d for d in ds if d is not None),default=np.nan)
        tasks.append({"task_id":str(r.sortie),"batch_id":str(r.sortie),"start_time":start,"arrival_time":arrival,"completion_time":completion,"hard_deadline":hard,"deadline_slack":np.nan if pd.isna(hard) else hard-completion,"late_flag":any(flags),"late_box_count":sum(flags),"transport_resource_interval":f"[{start},{ret})","relay_resource_interval":f"[{start-float(data['relay']['prep_s'])},{ret+float(data['relay']['turn_s'])})" if need else "","service_resource_interval":f"[{arrival},{completion})","battery_resource_interval":f"[{start},{ret+charge})","relay_energy":relay_energy,"transport_energy":float(r.energy_kwh),"total_energy":float(r.energy_kwh)+relay_energy,"source_function":"eval_A"})
    return pd.DataFrame(tasks),pd.DataFrame(deliveries),pd.DataFrame(events)

def eval_B(schedule,data,service_delta=0.0,charge_scale=1.0):
    boxes=_boxes(data); cert=pd.read_csv(ROOT/"results/final_q3_continuous_certification.csv"); tasks=[]; deliveries=[]; events=[]
    for _,r in schedule.iterrows():
        ids=_ids(r.box_ids); t0=float(r.start_s); arrival=float(r.delivery_s)-float(r.handoff_s); handoff=float(r.handoff_s)+float(service_delta); completion=arrival+handoff; ret=float(r.return_s)
        ds=[]
        for bid in ids:
            b=boxes[bid]; med=float(b["期望送达时间（s）"]) if b.get("物资类型")=="医疗物资" else None; first=float(b["首批截止时间（s）"]) if b.get("是否首批保障")=="是" else None; vals=[v for v in (med,first) if v is not None]; ds.append(min(vals) if vals else None)
        flags=[d is not None and completion>d for d in ds]
        for bid,d,flag in zip(ids,ds,flags): deliveries.append({"box_id":bid,"task_id":str(r.sortie),"arrival_time":arrival,"completion_time":completion,"hard_deadline":d,"late_flag":bool(flag),"lateness":0.0 if d is None else max(0.0,completion-d),"source_function":"eval_B.deadline_B"})
        cr=_relay(data,cert,r); need=bool(cr is not None and bool(cr.relay_needed)); relay_energy=(float(cr.total_distance_m)/float(data["relay"]["speed"]))*float(data["relay"]["power_kw"])/3600.0 if need and bool(cr.relay_feasible) else 0.0; charge=charge_B(data,r,charge_scale)
        events.extend([_event(f"transport:{r.aircraft}",r,t0,ret,"eval_B.transport_event"),_event(f"service:{r.service}",r,arrival,completion,"eval_B.service_event"),_event(f"battery:{r.battery}",r,t0,ret+charge,"eval_B.charge_event")])
        if need: events.append(_event("relay_pool",r,t0-float(data["relay"]["prep_s"]),ret+float(data["relay"]["turn_s"]),"eval_B.relay_event"))
        hard=min((d for d in ds if d is not None),default=np.nan)
        tasks.append({"task_id":str(r.sortie),"batch_id":str(r.sortie),"start_time":t0,"arrival_time":arrival,"completion_time":completion,"hard_deadline":hard,"deadline_slack":np.nan if pd.isna(hard) else hard-completion,"late_flag":any(flags),"late_box_count":sum(flags),"transport_resource_interval":f"[{t0},{ret})","relay_resource_interval":f"[{t0-float(data['relay']['prep_s'])},{ret+float(data['relay']['turn_s'])})" if need else "","service_resource_interval":f"[{arrival},{completion})","battery_resource_interval":f"[{t0},{ret+charge})","relay_energy":relay_energy,"transport_energy":float(r.energy_kwh),"total_energy":float(r.energy_kwh)+relay_energy,"source_function":"eval_B"})
    return pd.DataFrame(tasks),pd.DataFrame(deliveries),pd.DataFrame(events)

def _write(schedule,data,a_t,b_t,a_d,b_d,a_e,b_e):
    logs=ROOT/"logs"; logs.mkdir(exist_ok=True); rows=[]
    for i in range(len(a_t)):
        a,b=a_t.iloc[i],b_t.iloc[i]
        for f in ["start_time","arrival_time","completion_time","hard_deadline","deadline_slack","late_box_count","late_flag","transport_resource_interval","relay_resource_interval","service_resource_interval","battery_resource_interval","relay_energy","transport_energy","total_energy"]:
            av,bv=a[f],b[f]; delta=(int(bool(bv))-int(bool(av))) if f=="late_flag" else (float(bv)-float(av) if f not in ["transport_resource_interval","relay_resource_interval","service_resource_interval","battery_resource_interval"] and pd.notna(av) and pd.notna(bv) else (0.0 if av==bv else np.nan))
            rows.append({"task_id":a.task_id,"field":f,"A_value":av,"B_value":bv,"delta":delta,"A_source_function":"eval_A","B_source_function":"eval_B"})
    pd.DataFrame(rows).to_csv(logs/"parity_task_diff.csv",index=False)
    dr=[]
    for i in range(len(a_d)):
        a,b=a_d.iloc[i],b_d.iloc[i]
        for f in ["arrival_time","completion_time","hard_deadline","late_flag","lateness"]:
            av,bv=a[f],b[f]; delta=(int(bool(bv))-int(bool(av))) if f=="late_flag" else (float(bv)-float(av) if pd.notna(av) and pd.notna(bv) else np.nan)
            dr.append({"box_id":a.box_id,"task_id":a.task_id,"field":f,"A_value":av,"B_value":bv,"delta":delta,"A_source_function":"eval_A","B_source_function":"eval_B"})
    pd.DataFrame(dr).to_csv(logs/"parity_delivery_diff.csv",index=False)
    rr=[]
    for res in sorted(set(a_e.resource)|set(b_e.resource)):
        aa=a_e[a_e.resource==res].sort_values("task_id"); bb=b_e[b_e.resource==res].sort_values("task_id")
        for j in range(max(len(aa),len(bb))):
            a=aa.iloc[j] if j<len(aa) else None; b=bb.iloc[j] if j<len(bb) else None
            rr.append({"resource":res,"task_id":a.task_id if a is not None else b.task_id,"A_start":a.start_s if a is not None else np.nan,"A_end":a.end_s if a is not None else np.nan,"B_start":b.start_s if b is not None else np.nan,"B_end":b.end_s if b is not None else np.nan,"delta_start":(b.start_s-a.start_s) if a is not None and b is not None else np.nan,"delta_end":(b.end_s-a.end_s) if a is not None and b is not None else np.nan,"A_source_function":a.source_function if a is not None else "missing","B_source_function":b.source_function if b is not None else "missing"})
    pa,pb=sweep(a_e),sweep(b_e)
    for res in sorted(set(pa)|set(pb)): rr.append({"resource":res,"task_id":"__PEAK__","A_start":pa.get(res,0),"A_end":pa.get(res,0),"B_start":pb.get(res,0),"B_end":pb.get(res,0),"delta_start":pb.get(res,0)-pa.get(res,0),"delta_end":pb.get(res,0)-pa.get(res,0),"A_source_function":"sweep_A","B_source_function":"sweep_B"})
    pd.DataFrame(rr).to_csv(logs/"parity_resource_diff.csv",index=False)
    pd.DataFrame([x for x in rows if x["field"] in ("relay_energy","transport_energy","total_energy")]).to_csv(logs/"parity_energy_diff.csv",index=False)
    return pa,pb

def _checks(a_t,b_t,a_d,b_d,a_e,b_e):
    checks={"deadline":bool(np.array_equal(a_d.late_flag.to_numpy(),b_d.late_flag.to_numpy()))}
    groups=[("transport_interval",lambda x:x.resource.str.startswith("transport:")),("relay_interval",lambda x:x.resource=="relay_pool"),("service_interval",lambda x:x.resource.str.startswith("service:")),("battery_interval",lambda x:x.resource.str.startswith("battery:"))]
    for name,sel in groups:
        aa,bb=a_e[sel(a_e)],b_e[sel(b_e)]; checks[name]=len(aa)==len(bb) and len(aa)>0 and bool(np.allclose(aa[["start_s","end_s"]].to_numpy(),bb[["start_s","end_s"]].to_numpy(),atol=TOL,rtol=0))
    checks["resource_peak"]=sweep(a_e)==sweep(b_e)
    checks["relay_energy"]=bool(np.allclose(a_t.relay_energy,b_t.relay_energy,atol=TOL,rtol=0)); checks["transport_energy"]=bool(np.allclose(a_t.transport_energy,b_t.transport_energy,atol=TOL,rtol=0))
    return checks

def main():
    schedule=pd.read_csv(ROOT/"results/golden_grid60_schedule.csv"); data=load_inputs(); a_t,a_d,a_e=eval_A(schedule,data); b_t,b_d,b_e=eval_B(schedule,data); pa,pb=_write(schedule,data,a_t,b_t,a_d,b_d,a_e,b_e)
    dd=pd.read_csv(ROOT/"logs/parity_delivery_diff.csv"); rd=pd.read_csv(ROOT/"logs/parity_resource_diff.csv");
    checks=_checks(a_t,b_t,a_d,b_d,a_e,b_e)
    metrics_A={"tardy_boxes":int(a_d.late_flag.sum()),"tardy_tasks":int(a_t.late_flag.sum()),"total_lateness":float(a_d.lateness.sum()),"max_lateness":float(a_d.lateness.max()),"completion_time":float(a_t.completion_time.max()),"total_energy":float(a_t.total_energy.sum()),"resource_peaks":pa}
    metrics_B={"tardy_boxes":int(b_d.late_flag.sum()),"tardy_tasks":int(b_t.late_flag.sum()),"total_lateness":float(b_d.lateness.sum()),"max_lateness":float(b_d.lateness.max()),"completion_time":float(b_t.completion_time.max()),"total_energy":float(b_t.total_energy.sum()),"resource_peaks":pb}
    result={"checks":checks,"all_pass":all(checks.values()),"grid60_parity":"PASS" if all(checks.values()) else "FAIL","metrics_A":metrics_A,"metrics_B":metrics_B,"golden":"results/golden_grid60_schedule.csv"}; (ROOT/"logs/grid60_parity_test.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    sm_t,sm_d,sm_e=eval_B(schedule,data,service_delta=1.0); cm_t,cm_d,cm_e=eval_B(schedule,data,charge_scale=1.1); sm_checks=_checks(a_t,sm_t,a_d,sm_d,a_e,sm_e); cm_checks=_checks(a_t,cm_t,a_d,cm_d,a_e,cm_e)
    mut={"service_duration_plus_1s_fails":not all(sm_checks.values()),"service_mutated_checks":sm_checks,"charging_scale_plus_10pct_fails":not all(cm_checks.values()),"charging_mutated_checks":cm_checks,"baseline_grid60_parity":result["grid60_parity"]}; (ROOT/"logs/parity_mutation_test.json").write_text(json.dumps(mut,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=="__main__": main()
