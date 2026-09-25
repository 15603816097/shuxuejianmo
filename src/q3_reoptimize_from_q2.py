"""Q3-only re-scheduling from the frozen Q2 K=19 task structure.

Q2's 19 task/box/type columns are fixed.  This script only changes the
continuous event times and physical resource assignment so that the two relay
units are explicitly respected.  Q2 result files are never overwritten.
"""
from __future__ import annotations
import ast, json, time
from pathlib import Path
import numpy as np
import pandas as pd

from minimal_pipeline import load_inputs
from final_joint import strict_q3
from parity_evaluators import _ids, charge_A
from q2_k18_zero_hard import deadlines, timeliness
from q2_sortie_count_sensitivity import pool_peak, TOL
from continuous_joint import _solve
from minimal_pipeline import route_stats

ROOT = Path(__file__).resolve().parents[1]
RES, LOG = ROOT / "results", ROOT / "logs"
Q2 = RES / "q2_sensitivity_19_27" / "q2_sens_K19_schedule.csv"


def _event_peak(df, mask):
    return pool_peak(df, mask) if len(df) else 0


def _build_stage_a(data, q2, cert):
    """Earliest-deadline deterministic list schedule with two relays."""
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    rows = []
    for pos, (_, r) in enumerate(q2.iterrows()):
        ids = _ids(r.box_ids)
        ds = [d for bid in ids for d in deadlines(bm[bid])]
        rows.append((min(ds) if ds else 1e99, pos, r.copy()))
    rows.sort(key=lambda x: (x[0], x[1]))
    air_avail = {u: 0.0 for us in data["aircraft"].values() for u in us}
    bat_avail = {typ: {f"{typ}-B{i+1:02d}": 0.0 for i in range(int(n))}
                 for typ, n in data["batteries"].items()}
    svc_avail = {}
    relay_avail = {f"R{i+1:02d}": 0.0 for i in range(2)}
    out = []
    for _, _, r0 in rows:
        typ, service = str(r0.type), str(r0.service)
        cands = []
        cr = cert[cert.sortie.astype(str) == str(r0.sortie)].iloc[0]
        if not bool(cr.relay_feasible):
            raise RuntimeError(f"No certified relay candidate for {r0.sortie}")
        d = data["drones"][typ]
        dep_off = float(r0.delivery_s - r0.start_s)
        ret_off = float(r0.return_s - r0.start_s)
        handoff = float(r0.handoff_s)
        charge = float(charge_A(data, r0))
        for au in data["aircraft"][typ]:
            for bat, bt in bat_avail[typ].items():
                for rid, rt in relay_avail.items():
                    # arrival = start + dep_off - handoff; service is [arrival,delivery)
                    start = max(float(air_avail[au]), float(bt),
                                float(svc_avail.get(service, 0.0) - (dep_off - handoff)),
                                float(rt + data["relay"]["prep_s"] + data["relay"].get("link_s", 0.0)),
                                float(data["relay"]["prep_s"] + data["relay"].get("link_s", 0.0)))
                    end_relay = start + ret_off + float(data["relay"]["turn_s"])
                    cands.append((start, end_relay, au, bat, rid, charge))
        start, end_relay, au, bat, rid, charge = min(cands, key=lambda x: (x[0], x[2], x[3], x[4]))
        delivery = start + dep_off
        arrival = delivery - handoff
        ret = start + ret_off
        air_avail[au] = ret
        bat_avail[typ][bat] = ret + charge
        svc_avail[service] = delivery
        relay_avail[rid] = end_relay
        nr = r0.copy()
        nr["start_s"], nr["delivery_s"], nr["return_s"] = start, delivery, ret
        nr["aircraft"], nr["battery"] = au, bat
        nr["relay_id"] = rid
        out.append(nr)
    return pd.DataFrame(out).sort_values("sortie").reset_index(drop=True)


def _evaluate(data, schedule, cert):
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    drows, erows = [], []
    for _, r in schedule.iterrows():
        ids = _ids(r.box_ids)
        cr = cert[cert.sortie.astype(str) == str(r.sortie)].iloc[0]
        relay_energy = float(cr.total_distance_m) * float(data["relay"]["power_kw"]) / float(data["relay"]["speed"]) / 3600.0
        for bid in ids:
            b = bm[bid]; ds = deadlines(b); hard = min(ds) if ds else np.nan
            soft_dead = float(b["期望送达时间（s）"]) if b.get("物资类型") != "医疗物资" else np.nan
            hard_bad = bool(ds and float(r.delivery_s) > max(ds) + TOL) if False else bool(ds and any(float(r.delivery_s) > x + TOL for x in ds))
            soft_late = max(0.0, float(r.delivery_s) - soft_dead) if np.isfinite(soft_dead) else 0.0
            drows.append({"box_id": bid, "sortie_id": r.sortie, "service_area": r.service,
                          "arrival_time_s": float(r.delivery_s-r.handoff_s), "completion_time_s": float(r.delivery_s),
                          "hard_deadline_s": hard, "hard_violation": hard_bad,
                          "soft_deadline_s": soft_dead, "soft_lateness_s": soft_late,
                          "relay_id": r.relay_id})
        charge = float(charge_A(data, r))
        erows += [
            {"resource": f"transport:{r.aircraft}", "sortie_id": r.sortie, "start_s": r.start_s, "end_s": r.return_s},
            {"resource": f"battery:{r.battery}", "sortie_id": r.sortie, "start_s": r.start_s, "end_s": r.return_s+charge},
            {"resource": f"service:{r.service}", "sortie_id": r.sortie, "start_s": r.delivery_s-r.handoff_s, "end_s": r.delivery_s},
            {"resource": f"relay:{r.relay_id}", "sortie_id": r.sortie, "start_s": max(0.0, r.start_s-data["relay"]["prep_s"]), "end_s": r.return_s+data["relay"]["turn_s"]},
        ]
    return pd.DataFrame(drows), pd.DataFrame(erows)


def _stage_b_retype(data, q2):
    """Stage-B candidate: same 19 service/box batches, choose a feasible B/C type.

    This is deliberately a restricted contract-preserving relaxation.  The exact
    MILP below still decides ordering, aircraft, battery, and relay assignment.
    """
    out=[]
    bm={str(x["货箱编号"]):x for x in data["boxes"]}
    for _,r in q2.iterrows():
        ids=_ids(r.box_ids); br=[bm[x] for x in ids]; cand=[]
        for typ in ("B","C"):
            st=route_stats(data,typ,str(r.service),br)
            if st["safe"]: cand.append((float(st["flight_s"]),typ,st))
        if not cand: raise RuntimeError(f"No B/C feasible type for {r.sortie}")
        _,typ,st=min(cand,key=lambda z:(z[0],z[1]))
        nr=r.copy(); nr["type"]=typ; nr["aircraft"]=data["aircraft"][typ][0]; nr["battery"]=f"{typ}-B01"
        for k,v in st.items(): nr[k]=v
        # Keep the original identifiers and derive the fields required by the MILP.
        nr["start_s"]=0.0; nr["delivery_s"]=float(st["out_flight_s"])+float(r["handoff_s"]); nr["return_s"]=float(st["flight_s"])+float(r["handoff_s"])
        out.append(nr)
    return pd.DataFrame(out)


def main():
    t0 = time.time(); data = load_inputs(); q2 = pd.read_csv(Q2); cert = strict_q3(data, q2)
    # Certification is recomputed for the fixed Q2 task structure and retained separately.
    cert["certification_pass"] = cert.relay_feasible.astype(bool) & (cert.segment1_pixels > 0) & (cert.segment2_pixels > 0)
    stage = _build_stage_a(data, q2, cert)
    deliveries, events = _evaluate(data, stage, cert)
    # Hard feasibility includes both medical expected deadlines and first-batch deadlines.
    hard_boxes = int(deliveries.hard_violation.sum()); hard_sorties = int(deliveries.loc[deliveries.hard_violation, "sortie_id"].nunique())
    soft = deliveries[deliveries.soft_deadline_s.notna()]
    relay_peak = _event_peak(events, lambda x: x.resource.astype(str).str.startswith("relay:"))
    tr_peak = _event_peak(events, lambda x: x.resource.astype(str).str.startswith("transport:"))
    bp = _event_peak(events, lambda x: x.resource.astype(str).str.startswith("battery:B-")); cp = _event_peak(events, lambda x: x.resource.astype(str).str.startswith("battery:C-"))
    sp = _event_peak(events, lambda x: x.resource.astype(str).str.startswith("service:"))
    metrics = {"stage":"A","task_structure":"fixed_Q2_K19","sorties":len(stage),"direct_feasible":int(cert.direct_feasible.sum()),"relay_feasible":int(cert.relay_feasible.sum()),"infeasible":int((~cert.relay_feasible).sum()),"certification_bound":int(cert.certification_pass.sum()),"hard_violation_boxes":hard_boxes,"hard_violation_sorties":hard_sorties,"soft_late_boxes":int((soft.soft_lateness_s>TOL).sum()),"soft_total_lateness_s":float(soft.soft_lateness_s.sum()),"soft_max_lateness_s":float(soft.soft_lateness_s.max()),"makespan_s":float(stage.return_s.max()),"transport_energy_kwh":float(stage.energy_kwh.sum()),"relay_energy_kwh":float(cert.total_distance_m.sum()*data["relay"]["power_kw"]/data["relay"]["speed"]/3600.0),"transport_peak":tr_peak,"relay_peak":relay_peak,"B_battery_peak":bp,"C_battery_peak":cp,"service_peak":sp,"R2_feasible":bool(relay_peak<=2),"physical_constraints_pass":bool(hard_boxes==0 and tr_peak<=sum(map(len,data["aircraft"].values())) and bp<=data["batteries"]["B"] and cp<=data["batteries"]["C"] and sp<=15),"solver_status":"DETERMINISTIC_FEASIBLE_SCHEDULE","proven_optimal":False,"runtime_s":time.time()-t0}
    stage.to_csv(RES/"q3_K19_R2_schedule_stageA.csv",index=False)
    deliveries.to_csv(RES/"q3_K19_R2_deliveries_stageA.csv",index=False)
    events.to_csv(RES/"q3_K19_R2_resource_events_stageA.csv",index=False)
    (RES/"q3_K19_R2_metrics_stageA.json").write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding="utf-8")
    cert.to_csv(RES/"q3_K19_R2_certification_stageA.csv",index=False)
    # Exact Stage-A feasibility check: same fixed 19 tasks, continuous starts,
    # explicit R01/R02 disjunctions and all hard deadlines.
    _, _, rec_a = _solve(data, q2, cert, "Q3_StageA_exact", "zero", limit=120, violation_cap=0, R=2)
    # Stage B allows type reassignment while retaining K=19, service areas and
    # box batches.  Its exact result is recorded separately from Stage A.
    stage_b = _stage_b_retype(data, q2)
    cert_b = strict_q3(data, stage_b)
    _, _, rec_b = _solve(data, stage_b, cert_b, "Q3_StageB_exact", "zero", limit=120, violation_cap=0, R=2)
    rec_a["stage_label"]="A_fixed_structure"; rec_b["stage_label"]="B_retype_fixed_batches"
    pd.DataFrame([rec_a,rec_b]).to_csv(RES/"q3_K19_R2_stage_solver_comparison.csv",index=False)
    (RES/"q3_K19_R2_metrics_stageB.json").write_text(json.dumps({"stage":"B","solver":rec_b,"task_structure":"K19_fixed_service_box_batches_type_reassignment","proven_optimal":False},ensure_ascii=False,indent=2),encoding="utf-8")
    # If both K=19 formulations are strictly infeasible, test the existing
    # exactly-K Q2 columns under the same Q3 relay model.  These are separate
    # Q3 schedules and never overwrite Q2 files.
    scan=[]; first=None
    if rec_a.get("status")==2 and rec_b.get("status")==2:
        for K in range(20,28):
            p=RES/"q2_sensitivity_19_27"/f"q2_sens_K{K}_schedule.csv"
            if not p.exists(): continue
            tk=pd.read_csv(p); ck=strict_q3(data,tk); _,_,rr=_solve(data,tk,ck,f"Q3_K{K}","zero",limit=60,violation_cap=0,R=2)
            scan.append(rr|{"K":K})
            if rr.get("status")==0:
                first=K; break
    pd.DataFrame(scan).to_csv(RES/"q3_relay_capacity_K_scan.csv",index=False)
    # A deadline-window lower bound for the current single-service route
    # contract: eight service areas have hard boxes due by 3600 s.  Even if
    # each area is given its shortest physically safe B/C sortie, the sum of
    # relay occupancy intervals exceeds the capacity of two relays in [0,3600].
    hard3600_services=[]; lb_occ=0.0
    for service in sorted({str(x["服务区编号"]) for x in data["boxes"]}):
        hb=[x for x in data["boxes"] if str(x["货箱编号"]).startswith(service+"-") and (x["物资类型"]=="医疗物资" or x["是否首批保障"]=="是")]
        if not hb: continue
        ds=[float(x["首批截止时间（s）"]) if x["是否首批保障"]=="是" else float(x["期望送达时间（s）"]) for x in hb]
        if min(ds) > 3600 + TOL: continue
        candidates=[]
        for typ in ("B","C"):
            st=route_stats(data,typ,service,hb)
            if st["safe"]: candidates.append(float(st["flight_s"])+float(data["relay"]["prep_s"])+float(data["relay"]["turn_s"]))
        if candidates:
            hard3600_services.append(service); lb_occ += min(candidates)
    # The full relay interval extends beyond a delivery deadline, so its sum
    # cannot be used as a valid [0,3600] capacity proof.  Preserve it only as
    # a diagnostic and explicitly disable any infeasibility claim from it.
    capacity_lb={"deadline_s":3600.0,"service_count":len(hard3600_services),"services":hard3600_services,"full_interval_sum_diagnostic_s":lb_occ,"R2_window_capacity_s":7200.0,"proves_zero_hard_impossible_under_single_service_contract":False,"note":"Diagnostic only: post-deadline return/turn portions cannot be counted in the 0-3600 s capacity window."}
    (RES/"q3_R2_deadline_capacity_lower_bound.json").write_text(json.dumps(capacity_lb,ensure_ascii=False,indent=2),encoding="utf-8")
    metrics.update({"stage_a_exact":rec_a,"stage_b_exact":rec_b,"q3_first_feasible_K":first})
    (LOG/"q3_reoptimization_from_q2.md").write_text("\n".join([
        "# Q3 从 Q2 K=19 运输任务出发的二次调度",
        "", "Q2 的 K=19 任务结构、服务区、货箱集合、机型和总架次固定；Q3 仅重新安排起飞时刻、物理运输机、电池和 R01/R02 中继。",
        f"Stage A 启发式时序状态：{metrics['solver_status']}；硬截止违约箱数={hard_boxes}，软迟到箱数={metrics['soft_late_boxes']}。",
        f"Stage A 精确R=2零硬违约判定：{rec_a['message']}。Stage B先对每个固定批次确定性预选最短安全B/C机型，再将该类型固定后交给精确MILP；该固定候选模型判定：{rec_b['message']}。",
        f"通信认证记录：direct={metrics['direct_feasible']}，relay={metrics['relay_feasible']}，infeasible={metrics['infeasible']}，逐DEM记录绑定={metrics['certification_bound']}/{len(cert)}。",
        f"R=2 中继峰值={relay_peak}，该启发式时序的中继资源可行性={'PASS' if relay_peak<=2 else 'FAIL'}；但硬截止违约使整体Q3方案不可行。",
        f"截止窗口诊断：{len(hard3600_services)}个服务区含3600 s前硬箱；最短完整中继区间总和={lb_occ:.3f} s，但返航/周转可越过3600 s，故该数值不作为不可行证明。Stage B和K20–K27结论仅依据各自精确模型的status=INFEASIBLE。",
        "Q2 与 Q3 不共用完全相同的时序：Q2 不含通信资源约束，Q3 在相同运输任务结构上加入中继占用和通信认证，因此允许二次调度。",
        "本轮未修改 Q2 冻结结果；Q3 仅在其上进行二次调度。",
    ])+"\n",encoding="utf-8")
    q2row={"plan":"Q2_K19_frozen","K":19,"hard_violation_boxes":0,"soft_late_boxes":6,"soft_total_lateness_s":25505.986,"soft_max_lateness_s":5619.883,"makespan_s":10825.759,"transport_energy_kwh":61.047,"relay_energy_kwh":np.nan,"relay_peak":np.nan,"communication_feasible":False}
    q3row={"plan":"Q3_K19_R2_stageA","K":19,**{k:metrics[k] for k in ["hard_violation_boxes","soft_late_boxes","soft_total_lateness_s","soft_max_lateness_s","makespan_s","transport_energy_kwh","relay_energy_kwh","relay_peak"]},"communication_feasible":bool(metrics["R2_feasible"] and metrics["infeasible"]==0 and metrics["hard_violation_boxes"]==0 and metrics["physical_constraints_pass"])}
    pd.DataFrame([q2row,q3row]).to_csv(RES/"q2_q3_plan_comparison.csv",index=False)
    print(json.dumps(metrics,ensure_ascii=False,indent=2))

if __name__ == "__main__": main()
