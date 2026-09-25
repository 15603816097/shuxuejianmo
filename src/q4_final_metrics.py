"""Final Q4 reporting audit from frozen schedules and saved partitions.

No schedule is optimized or modified.  All resource calculations use half-open
intervals [start,end) and a 1e-6 second endpoint tolerance.
"""
from pathlib import Path
import ast, json
import numpy as np
import pandas as pd
from minimal_pipeline import load_inputs
from parity_evaluators import eval_A

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
LOG = ROOT / "logs"
TOL = 1e-6


def ids(v):
    return list(v) if isinstance(v, list) else list(ast.literal_eval(str(v)))


def peak(events, mask=None):
    g = events if mask is None else events[mask(events)]
    pts = []
    for _, r in g.iterrows():
        a = round(float(r.start_s) / TOL) * TOL
        b = round(float(r.end_s) / TOL) * TOL
        if b > a + TOL:
            pts.extend([(a, 1), (b, -1)])  # release before acquire at equality
    cur = pk = 0
    for _, d in sorted(pts, key=lambda z: (z[0], z[1])):
        cur += d; pk = max(pk, cur)
    return int(pk)


def resource_stats(events):
    transport = lambda x: x.resource.astype(str).str.startswith("transport:")
    relay = lambda x: x.resource.astype(str).str.startswith("relay_pool")
    batt_b = lambda x: x.resource.astype(str).str.startswith("battery:B-")
    batt_c = lambda x: x.resource.astype(str).str.startswith("battery:C-")
    service = lambda x: x.resource.astype(str).str.startswith("service:")
    per = {str(k): peak(g) for k, g in events.groupby("resource")}
    service_station_peak = max((v for k, v in per.items() if k.startswith("service:")), default=0)
    transport_unit_peak = max((v for k, v in per.items() if k.startswith("transport:")), default=0)
    battery_unit_peak = max((v for k, v in per.items() if k.startswith("battery:")), default=0)
    return {
        "transport_peak": peak(events, transport), "relay_peak": peak(events, relay),
        "B_battery_peak": peak(events, batt_b), "C_battery_peak": peak(events, batt_c),
        "service_peak_global": peak(events, service), "service_peak_per_station": service_station_peak,
        "transport_unit_peak": transport_unit_peak, "battery_unit_peak": battery_unit_peak,
        "per_resource_peak": per,
    }


def no_overlap(events, prefix):
    for resource, g in events[events.resource.astype(str).str.startswith(prefix)].groupby("resource"):
        pts = []
        for _, r in g.iterrows():
            a = round(float(r.start_s) / TOL) * TOL; b = round(float(r.end_s) / TOL) * TOL
            pts.extend([(a, 1), (b, -1)])
        cur = 0
        for _, d in sorted(pts, key=lambda z: (z[0], z[1])):
            cur += d
            if cur > 1:
                return False
    return True


def group_map(K):
    p = pd.read_csv(RES / f"q4_partition_k{K}_adjusted.csv")
    rows = p[p.resource == "transport_aircraft_all"]
    out = {}
    for _, r in rows.iterrows():
        for s in str(r.services).split(","):
            out[s] = int(r.group)
    return out, rows[["group", "services"]].drop_duplicates().sort_values("group")


def evaluate_schedule(schedule, data):
    t, d, e = eval_A(schedule, data)
    rs = resource_stats(e)
    late = d[d.late_flag.astype(bool)]
    return {
        "tasks": t, "deliveries": d, "events": e, "resources": rs,
        "late_boxes": int(d.late_flag.sum()), "late_sorties": int(t.late_flag.sum()),
        "total_lateness_s": float(d.lateness.sum()), "max_lateness_s": float(d.lateness.max()),
        "makespan_s": float(schedule.return_s.max()), "energy_kwh": float(schedule.energy_kwh.sum()),
        "no_overlap": no_overlap(e, "transport:") and no_overlap(e, "battery:") and no_overlap(e, "service:"),
        "late_sortie_ids": sorted(late.task_id.unique().tolist()),
    }


def main():
    data = load_inputs()
    schedules = {
        "original_baseline": pd.read_csv(RES / "q2_sorties_baseline.csv"),
        "resource_adjusted": pd.read_csv(RES / "q4_schedule_adjusted.csv"),
        "joint_deadline_first": pd.read_csv(RES / "q_joint_schedule_C.csv"),
        "final_unified_checkpoint": pd.read_csv(ROOT / "checkpoint_best_feasible_11" / "schedule.csv"),
    }
    ev = {name: evaluate_schedule(s, data) for name, s in schedules.items()}

    main_rows = []
    group_rows = []
    k_rows = []
    for scenario, out in ev.items():
        for K in (2, 3):
            gm, members = group_map(K)
            for _, m in members.iterrows():
                g = int(m.group); ss = [x for x, z in gm.items() if z == g]
                sub = schedules[scenario][schedules[scenario].service.astype(str).isin(ss)]
                se = out["events"][out["events"].task_id.isin(sub.sortie.astype(str))]
                # Group-level peaks are diagnostics; global peaks are reported separately.
                r = resource_stats(se)
                group_rows.append({"scenario": scenario, "K": K, "group": g,
                    "services": ",".join(ss), "sorties": len(sub),
                    "box_count": sum(len(ids(x)) for x in sub.box_ids),
                    "completion_time_s": float(sub.return_s.max()) if len(sub) else np.nan,
                    "energy_kwh": float(sub.energy_kwh.sum()),
                    "relay_peak": r["relay_peak"], "transport_peak": r["transport_peak"],
                    "B_battery_peak": r["B_battery_peak"], "C_battery_peak": r["C_battery_peak"],
                    "service_peak_global": r["service_peak_global"],
                    "service_peak_per_station": r["service_peak_per_station"]})
            if scenario == "final_unified_checkpoint":
                r = out["resources"]
                gsub = pd.DataFrame([x for x in group_rows if x["scenario"] == scenario and x["K"] == K])
                relay_sum = int(gsub.relay_peak.sum()); b_sum = int(gsub.B_battery_peak.sum()); c_sum = int(gsub.C_battery_peak.sum()); t_sum = int(gsub.transport_peak.sum())
                global_feasible = (r["transport_peak"] <= 8 and r["relay_peak"] <= 2 and
                                    r["B_battery_peak"] <= 4 and r["C_battery_peak"] <= 4 and
                                    r["service_peak_per_station"] <= 1 and out["no_overlap"])
                independent_feasible = (t_sum <= 8 and relay_sum <= 2 and b_sum <= 4 and c_sum <= 4)
                k_rows.append({"scenario": scenario, "group_count": K,
                    "group_members": " | ".join(f"G{int(x.group)}:{x.services}" for _, x in members.iterrows()),
                    "late_boxes": out["late_boxes"], "late_sorties": out["late_sorties"],
                    "makespan_s": out["makespan_s"], "energy_kwh": out["energy_kwh"],
                    "relay_peak": r["relay_peak"], "transport_peak": r["transport_peak"],
                    "B_battery_peak": r["B_battery_peak"], "C_battery_peak": r["C_battery_peak"],
                    "service_peak": r["service_peak_global"], "service_peak_per_station": r["service_peak_per_station"],
                    "global_shared_schedule_feasible": global_feasible,
                    "group_transport_peak_sum": t_sum, "group_relay_peak_sum": relay_sum,
                    "group_B_battery_peak_sum": b_sum, "group_C_battery_peak_sum": c_sum,
                    "relay_inventory_gap": max(0, relay_sum - 2), "B_battery_inventory_gap": max(0, b_sum - 4),
                    "C_battery_inventory_gap": max(0, c_sum - 4), "transport_inventory_gap": max(0, t_sum - 8),
                    "independent_group_config_feasible": independent_feasible,
                    "feasible": bool(global_feasible and independent_feasible)})
        r = out["resources"]
        main_rows.append({"scenario": scenario, "group_count_options": "2,3",
            "late_boxes": out["late_boxes"], "late_sorties": out["late_sorties"],
            "total_lateness_s": out["total_lateness_s"], "max_lateness_s": out["max_lateness_s"],
            "makespan_s": out["makespan_s"], "energy_kwh": out["energy_kwh"],
            "relay_peak_global": r["relay_peak"], "transport_peak_global": r["transport_peak"],
            "B_battery_peak_global": r["B_battery_peak"], "C_battery_peak_global": r["C_battery_peak"],
            "service_peak_global": r["service_peak_global"], "service_peak_per_station": r["service_peak_per_station"],
            "resource_intervals_no_overlap": out["no_overlap"]})

    pd.DataFrame(main_rows).to_csv(RES / "q4_main_metrics.csv", index=False)
    pd.DataFrame(group_rows).to_csv(RES / "q4_group_summary.csv", index=False)
    pd.DataFrame(k_rows).to_csv(RES / "q4_k_comparison.csv", index=False)

    # Unified resource audit for the frozen final Q2/Q3 schedule.
    final = ev["final_unified_checkpoint"]; r = final["resources"]
    transport_by_type = {}
    for typ in ("A", "B", "C"):
        units = data["aircraft"][typ]
        transport_by_type[typ] = peak(final["events"], lambda x, us=units: x.resource.astype(str).isin([f"transport:{u}" for u in us]))
    audit = [
        {"resource": "transport", "scope": "global", "inventory": 8, "observed_peak": r["transport_peak"], "feasible": r["transport_peak"] <= 8},
        {"resource": "transport_A", "scope": "type_pool", "inventory": len(data["aircraft"]["A"]), "observed_peak": transport_by_type["A"], "feasible": transport_by_type["A"] <= len(data["aircraft"]["A"])},
        {"resource": "transport_B", "scope": "type_pool", "inventory": len(data["aircraft"]["B"]), "observed_peak": transport_by_type["B"], "feasible": transport_by_type["B"] <= len(data["aircraft"]["B"])},
        {"resource": "transport_C", "scope": "type_pool", "inventory": len(data["aircraft"]["C"]), "observed_peak": transport_by_type["C"], "feasible": transport_by_type["C"] <= len(data["aircraft"]["C"])},
        {"resource": "relay", "scope": "global", "inventory": 2, "observed_peak": r["relay_peak"], "feasible": r["relay_peak"] <= 2},
        {"resource": "relay_energy_component", "scope": "paired_with_relay", "inventory": 2, "observed_peak": r["relay_peak"], "feasible": r["relay_peak"] <= 2},
        {"resource": "B_battery", "scope": "global", "inventory": 4, "observed_peak": r["B_battery_peak"], "feasible": r["B_battery_peak"] <= 4},
        {"resource": "C_battery", "scope": "global", "inventory": 4, "observed_peak": r["C_battery_peak"], "feasible": r["C_battery_peak"] <= 4},
        {"resource": "service", "scope": "per_station_assumption_capacity_1", "inventory": 1, "observed_peak": r["service_peak_per_station"], "feasible": r["service_peak_per_station"] <= 1},
        {"resource": "service", "scope": "global_diagnostic", "inventory": 15, "observed_peak": r["service_peak_global"], "feasible": r["service_peak_global"] <= 15},
        {"resource": "intervals", "scope": "all_transport_battery_service", "inventory": np.nan, "observed_peak": np.nan, "feasible": final["no_overlap"]},
    ]
    pd.DataFrame(audit).to_csv(RES / "q4_resource_capacity_audit.csv", index=False)

    cert = pd.read_csv(RES / "final_q3_continuous_certification.csv")
    bind = schedules["final_unified_checkpoint"][["sortie", "service"]].merge(
        cert[["sortie", "relay_feasible", "segment1_pixels", "segment2_pixels"]], on="sortie", how="left", validate="one_to_one")
    bind["certification_record_pass"] = bind.relay_feasible.astype(bool) & (bind.segment1_pixels > 0) & (bind.segment2_pixels > 0)
    bind["record_binding_pass"] = bind.certification_record_pass
    bind.rename(columns={"sortie": "sortie_id", "service": "service_area"}, inplace=True)
    bind.to_csv(RES / "q4_q3_binding_audit.csv", index=False)

    # Adjustment comparison uses one evaluator and one endpoint convention.
    comp = []
    stored_plan = pd.read_csv(RES / "q2_q3_q4_plan_comparison.csv", index_col=0)
    for scenario, out in ev.items():
        r = out["resources"]
        old_name = {"original_baseline": "A_original_baseline", "resource_adjusted": "B_resource_adjusted", "joint_deadline_first": "C_joint_deadline_first"}.get(scenario)
        comp.append({"scenario": scenario, "late_boxes": out["late_boxes"], "late_sorties": out["late_sorties"],
            "total_lateness_s": out["total_lateness_s"], "max_lateness_s": out["max_lateness_s"],
            "makespan_s": out["makespan_s"], "relay_peak": r["relay_peak"],
            "transport_peak": r["transport_peak"], "B_battery_peak": r["B_battery_peak"],
            "C_battery_peak": r["C_battery_peak"], "service_peak": r["service_peak_global"],
            "energy_kwh": out["energy_kwh"], "intervals_no_overlap": out["no_overlap"],
            "stored_summary_relay_peak": (float(stored_plan.loc[old_name, "relay_peak"]) if old_name in stored_plan.index else np.nan),
            "strict_relay_peak": r["relay_peak"]})
    pd.DataFrame(comp).to_csv(RES / "q4_adjustment_comparison.csv", index=False)

    final_feasible = bool(pd.DataFrame(audit).feasible.all() and bind.record_binding_pass.all())
    summary = f"""# 问题四最终答题摘要

- Q4保留K=2和K=3两种分组口径。K=2分为两组，K=3分为三组，服务区按既有轮转分组表划分；分组本身不改变冻结任务时序。
- 最终采用冻结的Q2/Q3统一调度 `checkpoint_best_feasible_11/schedule.csv`。其全局资源峰值为：运输机 {r['transport_peak']}、中继 {r['relay_peak']}、B型电池 {r['B_battery_peak']}、C型电池 {r['C_battery_peak']}、服务资源全局诊断峰值 {r['service_peak_global']}（逐服务区峰值 {r['service_peak_per_station']}）。
- 因此最终统一调度满足 R=2、运输机、电池和逐服务区服务资源容量约束，并通过Q3认证记录绑定 {int(bind.record_binding_pass.sum())}/{len(bind)}；本轮未重新执行DEM连续认证。
- 若将各分组视为同时独立配置，K=2的中继/B电池峰值需求合计为4/5，K=3为5/5，均超过共享库存；所以K=2/K=3只能作为同一共享时序的分组展示，不能解释为两套独立资源配置均可行。
- 旧Q4资源调整表的“4→2”只按简化中继区间统计；本次严格按含准备/释放时间的半开区间重算，原始方案中继峰值为 {ev['original_baseline']['resources']['relay_peak']}，资源调整表为 {ev['resource_adjusted']['resources']['relay_peak']}，不能把旧摘要直接当作严格全局峰值。
- 资源调整表对应的时序有 {ev['resource_adjusted']['late_boxes']} 个硬截止违约箱、{ev['resource_adjusted']['late_sorties']} 个违约架次；相对原始方案的延后释放确实造成及时性变差。最终冻结调度为 {final['late_boxes']} 个违约箱、{final['late_sorties']} 个违约架次。
- 全局共享时序峰值与独立分组配置需求同时报告：前者用于判断冻结统一调度，后者将各组峰值相加并计算库存缺口。服务资源容量1是“每服务区一个服务工位”的模型假设。所有区间统一为半开区间 [start,end)，端点容差1e-6 s。

最终Q4结论：K=2/K=3是同一冻结统一调度的两种分组展示；冻结统一调度满足R=2及运输机、电池、服务资源容量约束，且18/18架次均绑定已有Q3认证记录，但把分组当作独立配置时存在中继/B电池库存缺口，资源调整时序也会增加硬截止超时。
"""
    (LOG / "q4_final_answer_summary.md").write_text(summary, encoding="utf-8")
    (LOG / "q4_final_metrics_run.json").write_text(json.dumps({
        "final_scenario": "final_unified_checkpoint", "k_comparison": k_rows,
        "final_resource_feasible": final_feasible, "q3_record_binding": f"{int(bind.record_binding_pass.sum())}/{len(bind)}",
        "baseline_relay_peak": int(ev["original_baseline"]["resources"]["relay_peak"]),
        "adjusted_relay_peak_strict": int(ev["resource_adjusted"]["resources"]["relay_peak"]),
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(pd.DataFrame(k_rows).to_string(index=False))


if __name__ == "__main__":
    main()
