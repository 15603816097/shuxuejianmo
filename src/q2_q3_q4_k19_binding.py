"""Rebind Q3 and Q4 evidence to the frozen K=19 Q2 schedule."""
from pathlib import Path
import ast, json
import numpy as np
import pandas as pd
from minimal_pipeline import load_inputs
from final_joint import strict_q3
from parity_evaluators import charge_A, _ids
from q2_sortie_count_sensitivity import pool_peak, TOL
from q2_k18_zero_hard import timeliness

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/"results"; LOG=ROOT/"logs"
K19=RES/"q2_sensitivity_19_27"/"q2_sens_K19_schedule.csv"

def assign_relays(events, capacity=2):
    avail=[0.0]*capacity; ids=[]
    for _,r in events.sort_values("relay_start_s").iterrows():
        j=min(range(capacity),key=lambda x:avail[x]); ids.append((r.sortie,f"R{j+1:02d}")); avail[j]=float(r.relay_end_s)
    return dict(ids)

def group_maps():
    maps={}
    for K in (2,3):
        p=pd.read_csv(RES/f"q4_partition_k{K}_adjusted.csv"); rows=p[p.resource=="transport_aircraft_all"]; m={}
        for _,r in rows.iterrows():
            for s in str(r.services).split(","): m[s]=int(r.group)
        maps[K]=m
    return maps

def main():
    data=load_inputs(); schedule=pd.read_csv(K19)
    cert=strict_q3(data,schedule)
    cert.to_csv(RES/"q3_K19_certification.csv",index=False)
    cert["certification_pass"]=cert.relay_feasible.astype(bool)&(cert.segment1_pixels>0)&(cert.segment2_pixels>0)
    direct=int(cert.direct_feasible.sum()); relay=int(cert.relay_feasible.sum()); infeasible=int((~cert.relay_feasible).sum())
    # Construct one common event ledger and relay assignment for Q3/Q4.
    rows=[]
    for _,r in schedule.iterrows():
        ch=charge_A(data,r); arrival=float(r.delivery_s-r.handoff_s)
        cr=cert[cert.sortie.astype(str)==str(r.sortie)].iloc[0]
        rows.append({"sortie":r.sortie,"service":r.service,"start_s":r.start_s,"return_s":r.return_s,"relay_start_s":max(0.0,float(r.start_s)-data["relay"]["prep_s"]),"relay_end_s":float(r.return_s)+data["relay"]["turn_s"],"battery_end_s":float(r.return_s)+ch,"arrival_s":arrival,"completion_s":r.delivery_s,"aircraft":r.aircraft,"battery":r.battery,"relay_needed":bool(cr.relay_needed and cr.relay_feasible)})
    evbase=pd.DataFrame(rows); relay_ids=assign_relays(evbase[evbase.relay_needed])
    evbase["relay_id"]=evbase.sortie.map(relay_ids)
    ev=[]
    for _,r in evbase.iterrows():
        ev += [{"resource":f"transport:{r.aircraft}","sortie":r.sortie,"start_s":r.start_s,"end_s":r.return_s},
               {"resource":f"battery:{r.battery}","sortie":r.sortie,"start_s":r.start_s,"end_s":r.battery_end_s},
               {"resource":f"service:{r.service}","sortie":r.sortie,"start_s":r.arrival_s,"end_s":r.completion_s}]
        if r.relay_needed: ev.append({"resource":f"relay:{r.relay_id}","sortie":r.sortie,"start_s":r.relay_start_s,"end_s":r.relay_end_s})
    events=pd.DataFrame(ev); events.to_csv(RES/"q4_K19_resource_events.csv",index=False)
    # Q3 plan and resource summary.
    plan=cert.merge(evbase[["sortie","relay_id"]],on="sortie",how="left",validate="one_to_one")
    plan["relay_position"]=plan.apply(lambda r:f"fraction={r.relay_fraction:.6f};height_offset={r.relay_height_offset_m:.3f}m" if pd.notna(r.relay_fraction) else "",axis=1)
    plan["transport_to_relay_distance_m"]=plan["relay_fraction"]*plan["total_distance_m"]
    plan["relay_to_target_distance_m"]=(1-plan["relay_fraction"])*plan["total_distance_m"]
    plan[["sortie","service","relay_id","relay_position","transport_to_relay_distance_m","relay_to_target_distance_m","min_clearance_m","link_margin_db","segment1_pixels","segment2_pixels","certification_pass"]].rename(columns={"sortie":"sortie_id","service":"service_area"}).to_csv(RES/"q3_K19_relay_plan.csv",index=False)
    relay_peak=pool_peak(events,lambda x:x.resource.astype(str).str.startswith("relay:")); transport_peak=pool_peak(events,lambda x:x.resource.astype(str).str.startswith("transport:")); bpeak=pool_peak(events,lambda x:x.resource.astype(str).str.startswith("battery:B-")); cpeak=pool_peak(events,lambda x:x.resource.astype(str).str.startswith("battery:C-")); speak=pool_peak(events,lambda x:x.resource.astype(str).str.startswith("service:"))
    per_service={str(k):pool_peak(g,lambda x:pd.Series(True,index=x.index)) for k,g in events[events.resource.astype(str).str.startswith("service:")].groupby("resource")}
    pd.DataFrame([{"resource":"relay","inventory":2,"observed_peak":relay_peak,"feasible":relay_peak<=2},{"resource":"transport","inventory":8,"observed_peak":transport_peak,"feasible":transport_peak<=8},{"resource":"B_battery","inventory":4,"observed_peak":bpeak,"feasible":bpeak<=4},{"resource":"C_battery","inventory":4,"observed_peak":cpeak,"feasible":cpeak<=4},{"resource":"service_global_diagnostic","inventory":15,"observed_peak":speak,"feasible":speak<=15},{"resource":"service_per_area_max","inventory":1,"observed_peak":max(per_service.values()),"feasible":max(per_service.values())<=1}]).to_csv(RES/"q3_K19_resource_audit.csv",index=False)
    pd.DataFrame([{"metric":"total_sorties","value":len(schedule)},{"metric":"direct_feasible","value":direct},{"metric":"relay_feasible","value":relay},{"metric":"infeasible","value":infeasible},{"metric":"all_19_certification_records_bound","value":bool(cert.certification_pass.all())},{"metric":"relay_peak_R2","value":relay_peak},{"metric":"R2_feasible","value":relay_peak<=2}]).to_csv(RES/"q3_K19_main_metrics.csv",index=False)
    # Q4 group display and global-vs-independent resource requirements.
    tm=timeliness(data,schedule); maps=group_maps(); g_rows=[]; k_rows=[]
    for K,m in maps.items():
        for g in sorted(set(m.values())):
            sv=[s for s,z in m.items() if z==g]; sub=evbase[evbase.service.astype(str).isin(sv)]; ids=set(sub.sortie); ee=events[events.sortie.isin(ids)]
            g_rows.append({"K":K,"group":g,"services":",".join(sv),"sorties":len(sub),"box_count":sum(len(_ids(x)) for x in schedule[schedule.sortie.isin(ids)].box_ids),"completion_time_s":sub.return_s.max(),"energy_kwh":float(schedule[schedule.sortie.isin(ids)].energy_kwh.sum()),"relay_peak":pool_peak(ee,lambda x:x.resource.astype(str).str.startswith("relay:")),"transport_peak":pool_peak(ee,lambda x:x.resource.astype(str).str.startswith("transport:")),"B_battery_peak":pool_peak(ee,lambda x:x.resource.astype(str).str.startswith("battery:B-")),"C_battery_peak":pool_peak(ee,lambda x:x.resource.astype(str).str.startswith("battery:C-")),"service_peak":pool_peak(ee,lambda x:x.resource.astype(str).str.startswith("service:"))})
        gg=pd.DataFrame([x for x in g_rows if x["K"]==K]); k_rows.append({"K":K,"group_members":" | ".join(f"G{int(r.group)}:{r.services}" for _,r in gg.iterrows()),"late_boxes":int(tm.hard_violation.sum()),"soft_late_boxes":int(tm.soft_late.sum()),"makespan_s":float(schedule.return_s.max()),"energy_kwh":float(schedule.energy_kwh.sum()),"relay_peak_global":relay_peak,"transport_peak_global":transport_peak,"B_battery_peak_global":bpeak,"C_battery_peak_global":cpeak,"service_peak_global":speak,"service_peak_per_area":max(per_service.values()),"group_relay_peak_sum":int(gg.relay_peak.sum()),"group_B_battery_peak_sum":int(gg.B_battery_peak.sum()),"global_shared_feasible":bool(relay_peak<=2 and transport_peak<=8 and bpeak<=4 and cpeak<=4 and max(per_service.values())<=1),"independent_group_config_feasible":bool(gg.relay_peak.sum()<=2 and gg.B_battery_peak.sum()<=4)})
    pd.DataFrame(g_rows).to_csv(RES/"q4_K19_group_summary.csv",index=False); pd.DataFrame([{"metric":"sorties","value":len(schedule)},{"metric":"hard_violation_boxes","value":int(tm.hard_violation.sum())},{"metric":"soft_late_boxes","value":int(tm.soft_late.sum())},{"metric":"makespan_s","value":float(schedule.return_s.max())},{"metric":"transport_peak","value":transport_peak},{"metric":"relay_peak","value":relay_peak},{"metric":"B_battery_peak","value":bpeak},{"metric":"C_battery_peak","value":cpeak},{"metric":"service_peak_global","value":speak},{"metric":"service_peak_per_area","value":max(per_service.values())}]).to_csv(RES/"q4_K19_main_metrics.csv",index=False)
    pd.DataFrame([{"resource":"transport","inventory":8,"observed_peak":transport_peak,"feasible":transport_peak<=8},{"resource":"relay","inventory":2,"observed_peak":relay_peak,"feasible":relay_peak<=2},{"resource":"B_battery","inventory":4,"observed_peak":bpeak,"feasible":bpeak<=4},{"resource":"C_battery","inventory":4,"observed_peak":cpeak,"feasible":cpeak<=4},{"resource":"service_per_area","inventory":1,"observed_peak":max(per_service.values()),"feasible":max(per_service.values())<=1}]).to_csv(RES/"q4_K19_resource_audit.csv",index=False)
    bind=plan[["sortie","service","certification_pass","relay_id"]].rename(columns={"sortie":"sortie_id","service":"service_area"}).copy(); bind["q2_schedule_found"]=bind.sortie_id.isin(schedule.sortie); bind["binding_pass"]=bind.certification_pass&bind.q2_schedule_found; bind.to_csv(RES/"q4_K19_q3_binding.csv",index=False)
    relay_status = "PASS" if relay_peak <= 2 else "FAIL"
    (LOG/"q2_q3_q4_K19_binding.md").write_text(f"# K=19统一绑定\n\nQ2、Q3、Q4共同使用19架次schedule。Q3认证记录：{direct} direct/{relay} relay/{infeasible} infeasible；逐DEM栅格连续几何认证记录绑定 {int(cert.certification_pass.sum())}/{len(cert)}。R=2中继峰值={relay_peak}，资源口径结论={relay_status}。K2/K3仅为分组展示，独立配置需求另列；本文件不将静态认证记录绑定等同于R=2下的动态通信资源可行性。\n",encoding='utf-8')
    (RES/"q2_q3_q4_K19_unified_summary.json").write_text(json.dumps({"schedule":"results/q2_sensitivity_19_27/q2_sens_K19_schedule.csv","q3":{"direct":direct,"relay":relay,"infeasible":infeasible,"binding":int(cert.certification_pass.sum())},"q4":{"relay_peak":relay_peak,"transport_peak":transport_peak,"B_peak":bpeak,"C_peak":cpeak,"service_per_area":max(per_service.values())}},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({"q3":{"direct":direct,"relay":relay,"infeasible":infeasible},"q4":{"relay_peak":relay_peak,"transport_peak":transport_peak,"B_peak":bpeak,"C_peak":cpeak,"service_peak":speak,"service_per_area":max(per_service.values())},"binding":int(cert.certification_pass.sum())},ensure_ascii=False,indent=2))

if __name__=='__main__': main()
