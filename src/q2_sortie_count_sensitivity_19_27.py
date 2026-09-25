"""Q2 fixed-count sensitivity for K=19..27.

This is a new, non-overwriting experiment directory.  It reuses the audited
Q2 route-column construction and event decoder, keeps Q3 communication out of
the constraints, and reports hard and soft timeliness separately.
"""
from __future__ import annotations
import ast, json, time
from pathlib import Path
import numpy as np
import pandas as pd

from minimal_pipeline import load_inputs, q1
from parity_evaluators import eval_A, _ids
from q2_sortie_count_sensitivity import split_to_k, schedule_batches, pool_peak, box_map, TOL

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "q2_sensitivity_19_27"
RES.mkdir(parents=True, exist_ok=True)


def hard_deadlines(box):
    vals = []
    if box.get("物资类型") == "医疗物资":
        vals.append(float(box["期望送达时间（s）"]))
    if box.get("是否首批保障") == "是":
        vals.append(float(box["首批截止时间（s）"]))
    return vals


def evaluate(data, schedule):
    bm = box_map(data)
    tasks, _, events = eval_A(schedule, data)
    rows = []
    for _, r in schedule.iterrows():
        completion = float(r.delivery_s)
        for bid in _ids(r.box_ids):
            b = bm[bid]
            hd = hard_deadlines(b)
            hard_deadline = min(hd) if hd else np.nan
            hard_violation = bool(hd and any(completion > x + TOL for x in hd))
            # Soft objective is defined only for non-medical boxes' expected
            # delivery time. Equal unit weights are used and recorded below.
            soft_deadline = float(b["期望送达时间（s）"]) if b.get("物资类型") != "医疗物资" else np.nan
            soft_lateness = max(0.0, completion - soft_deadline) if np.isfinite(soft_deadline) else 0.0
            rows.append({"box_id": bid, "sortie": r.sortie, "service": r.service,
                         "completion_s": completion, "hard_deadline_s": hard_deadline,
                         "hard_violation": hard_violation, "soft_deadline_s": soft_deadline,
                         "soft_late": bool(np.isfinite(soft_deadline) and soft_lateness > TOL),
                         "soft_lateness_s": soft_lateness})
    d = pd.DataFrame(rows)
    hard_sorties = int(d.loc[d.hard_violation, "sortie"].nunique())
    s_late = d[d.soft_late]
    # Equal weights per soft-late box; weighted penalty therefore has seconds
    # as its unit and is directly auditable from the row-level table.
    transport = lambda x: x.resource.astype(str).str.startswith("transport:")
    bb = lambda x: x.resource.astype(str).str.startswith("battery:B-")
    cc = lambda x: x.resource.astype(str).str.startswith("battery:C-")
    ss = lambda x: x.resource.astype(str).str.startswith("service:")
    peaks = {
        "transport_peak": pool_peak(events, transport),
        "B_battery_peak": pool_peak(events, bb),
        "C_battery_peak": pool_peak(events, cc),
        "service_peak": pool_peak(events, ss),
    }
    service_per_station = {str(k): pool_peak(g, lambda x: pd.Series(True, index=x.index)) for k, g in events[events.resource.astype(str).str.startswith("service:")].groupby("resource")}
    peaks["service_peak_per_station"] = max(service_per_station.values(), default=0)
    def no_overlap(prefix):
        for _, g in events[events.resource.astype(str).str.startswith(prefix)].groupby("resource"):
            pts = []
            for _, z in g.iterrows():
                a = round(float(z.start_s) / TOL) * TOL; b = round(float(z.end_s) / TOL) * TOL
                pts.extend([(a, 1), (b, -1)])
            cur = 0
            for _, delta in sorted(pts, key=lambda x: (x[0], x[1])):
                cur += delta
                if cur > 1: return False
        return True
    # Physical constraints are validated independently of timeliness.
    all_ids = [x for v in schedule.box_ids for x in _ids(v)]
    mass_vol_ok = True
    for _, r in schedule.iterrows():
        ids = _ids(r.box_ids); mass = sum(float(bm[x]["单箱质量（kg）"]) for x in ids); vol = sum(float(bm[x]["单箱体积（m³）"]) for x in ids)
        dt = data["drones"][str(r.type)]
        mass_vol_ok &= mass <= dt.max_mass + TOL and vol <= dt.volume + TOL
    soc_ok = all(float(r.soc_return) + TOL >= float(data["drones"][str(r.type)].reserve) for _, r in schedule.iterrows())
    physical_ok = (len(all_ids) == len(data["boxes"]) == len(set(all_ids)) and mass_vol_ok and soc_ok and len(d) == len(data["boxes"])
                   and no_overlap("transport:") and no_overlap("battery:") and no_overlap("service:")
                   and peaks["transport_peak"] <= sum(len(x) for x in data["aircraft"].values())
                   and peaks["B_battery_peak"] <= data["batteries"]["B"]
                   and peaks["C_battery_peak"] <= data["batteries"]["C"]
                   and peaks["service_peak_per_station"] <= 1)
    return tasks, d, events, peaks, physical_ok, hard_sorties, s_late


def main():
    data = load_inputs(); _, base = q1(data)
    records = []
    for K in range(19, 28):
        t0 = time.time()
        batches = split_to_k(data, base, K)
        schedule = schedule_batches(data, batches, f"SENS{K}")
        tasks, deliveries, events, peaks, physical_ok, hard_sorties, soft = evaluate(data, schedule)
        schedule.to_csv(RES / f"q2_sens_K{K}_schedule.csv", index=False)
        deliveries.to_csv(RES / f"q2_sens_K{K}_deliveries.csv", index=False)
        events.to_csv(RES / f"q2_sens_K{K}_resource_events.csv", index=False)
        route = pd.DataFrame([{"sortie_id": r.sortie, "route": f"O01 -> {r.service} -> O01",
                               "service_sequence": r.service, "service_area_count": 1,
                               "box_count": len(_ids(r.box_ids)), "box_ids": ";".join(_ids(r.box_ids)),
                               "aircraft_id": r.aircraft, "aircraft_type": r.type,
                               "start_s": r.start_s, "return_s": r.return_s} for _, r in schedule.iterrows()])
        route.to_csv(RES / f"q2_sens_K{K}_route_summary.csv", index=False)
        hard_boxes = int(deliveries.hard_violation.sum())
        hard_feasible = bool(hard_boxes == 0 and physical_ok)
        records.append({"K": K, "solver_status": "FEASIBLE_HEURISTIC" if hard_feasible else "BEST_KNOWN_HEURISTIC_WITH_HARD_VIOLATIONS",
            "hard_feasible": hard_feasible, "proven_optimal": False,
            "hard_violation_boxes": hard_boxes, "hard_violation_sorties": hard_sorties,
            "soft_late_boxes": int(soft.soft_late.sum()), "soft_total_lateness_s": float(soft.soft_lateness_s.sum()),
            "soft_max_lateness_s": float(soft.soft_lateness_s.max()) if len(soft) else 0.0,
            "weighted_timeliness_penalty": float(soft.soft_lateness_s.sum()),
            "makespan_s": float(schedule.return_s.max()), "transport_energy_kwh": float(schedule.energy_kwh.sum()),
            "single_point_sorties": int(len(schedule)), "multi_point_sorties": 0,
            "max_service_areas_per_sortie": 1, "B_sorties": int((schedule.type == "B").sum()),
            "C_sorties": int((schedule.type == "C").sum()), **peaks,
            "runtime_s": time.time() - t0, "mip_gap": np.nan,
            "physical_constraints_pass": bool(physical_ok),
            "exact_sortie_count": int(len(schedule)) == K,
            "note": "Deterministic split/earliest-available Q2 route-column search; Q3 communication excluded; no global optimality proof"})
    table = pd.DataFrame(records).sort_values("K")
    table.to_csv(ROOT / "results/q2_sortie_count_sensitivity_19_27.csv", index=False)
    table[["K", "hard_violation_boxes", "hard_violation_sorties", "soft_late_boxes", "soft_total_lateness_s", "soft_max_lateness_s", "weighted_timeliness_penalty", "hard_feasible", "proven_optimal"]].to_csv(ROOT / "results/q2_hard_soft_timeliness_19_27.csv", index=False)
    (ROOT / "logs/q2_sensitivity_19_27.md").write_text(
        "# Q2 K=19..27固定架次数实验\n\n"
        "每个K均由确定性路线列拆分和最早可用事件解码器生成恰好K架次。"
        "硬截止单独统计医疗物资期望时间与首批截止；软迟到仅统计非医疗物资期望送达时间，等权加总秒数。"
        "Q3通信不进入Q2约束；所有结果为best-known heuristic，未证明全局最优。\n",
        encoding="utf-8")
    print(table.to_string(index=False))


if __name__ == "__main__":
    main()
