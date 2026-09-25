"""Q2 fixed-K sensitivity scan for K=18..28.

Uses the same Q2-v2 candidate pool and official transport/resource semantics as
q2_v2_solver.py.  The only added model constraint is sum(selected_routes) == K.
For every K, optimize makespan first and report solver status/bound/gap plus
secondary metrics from the best feasible incumbent.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import pandas as pd

import q2_v2_solver as q2

OUT = q2.OUT / "k_scan"
OUT.mkdir(parents=True, exist_ok=True)


def solve_fixed_k(data, pool, k: int, time_limit_s: int):
    from ortools.sat.python import cp_model

    m = cp_model.CpModel()
    box_ids = sorted(str(x["货箱编号"]) for x in data["boxes"])
    if len(box_ids) != 80:
        raise RuntimeError(f"expected 80 boxes, got {len(box_ids)}")

    candidates = []
    by_box = {b: [] for b in box_ids}
    intervals_air = {}
    intervals_bat = {}
    xvars = []
    svars = []
    evars = []

    H = int(math.ceil(13000.0 * q2.SCALE))
    for i, r in pool.iterrows():
        ids = json.loads(r.box_ids)
        offs = json.loads(r.delivery_offsets)
        dur = int(math.ceil(float(r.duration_s) * q2.SCALE - 1e-9))
        bdur = int(math.ceil((float(r.duration_s) + float(r.charge_s)) * q2.SCALE - 1e-9))

        xv = m.NewBoolVar(f"x_{i}")
        sv = m.NewIntVar(0, H, f"s_{i}")
        ev = m.NewIntVar(0, H, f"e_{i}")
        bev = m.NewIntVar(0, H + bdur, f"be_{i}")
        ai = m.NewOptionalIntervalVar(sv, dur, ev, xv, f"air_{i}")
        bi = m.NewOptionalIntervalVar(sv, bdur, bev, xv, f"bat_{i}")

        m.Add(sv == 0).OnlyEnforceIf(xv.Not())
        m.Add(ev == 0).OnlyEnforceIf(xv.Not())
        m.Add(bev == 0).OnlyEnforceIf(xv.Not())

        if pd.notna(r.latest_start_s):
            latest = int(math.floor(float(r.latest_start_s) * q2.SCALE + 1e-9))
            m.Add(sv <= latest).OnlyEnforceIf(xv)

        xvars.append(xv)
        svars.append(sv)
        evars.append(ev)

        typ = str(r.drone_type)
        intervals_air.setdefault(typ, []).append(ai)
        intervals_bat.setdefault(typ, []).append(bi)
        candidates.append({
            "row": r,
            "ids": ids,
            "offs": {str(a): float(b) for a, b in offs.items()},
            "x": xv, "s": sv, "e": ev,
        })
        for bid in ids:
            by_box[str(bid)].append(i)

    for bid in box_ids:
        idx = by_box[bid]
        if not idx:
            raise RuntimeError(f"candidate pool does not cover {bid}")
        m.Add(sum(xvars[i] for i in idx) == 1)

    # The sensitivity experiment: exactly K selected transport sorties.
    m.Add(sum(xvars) == int(k))

    for typ, ints in intervals_air.items():
        m.AddCumulative(ints, [1] * len(ints), len(data["aircraft"][typ]))
    for typ, ints in intervals_bat.items():
        m.AddCumulative(ints, [1] * len(ints), int(data["batteries"][typ]))

    cmax = m.NewIntVar(0, H, "Cmax")
    for i, c in enumerate(candidates):
        m.Add(cmax >= c["e"]).OnlyEnforceIf(c["x"])

    # Strengthening: type-wise selected workload cannot exceed parallel capacity.
    for typ in sorted(intervals_air):
        idx = [i for i, c in enumerate(candidates) if str(c["row"].drone_type) == typ]
        terms = []
        for i in idx:
            dur_i = int(math.ceil(float(candidates[i]["row"].duration_s) * q2.SCALE - 1e-9))
            terms.append(dur_i * xvars[i])
        if terms:
            m.Add(sum(terms) <= len(data["aircraft"][typ]) * cmax)

    # Makespan is the primary objective for cross-K comparability.
    m.Minimize(cmax)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit_s)
    solver.parameters.num_search_workers = 8
    solver.parameters.log_search_progress = False

    status = solver.Solve(m)
    name = solver.StatusName(status)
    feasible = status in (cp_model.OPTIMAL, cp_model.FEASIBLE)

    base = {
        "K": int(k),
        "solver_status": name,
        "feasible_found": bool(feasible),
        "proven_optimal_current_pool": bool(status == cp_model.OPTIMAL),
        "runtime_limit_s": int(time_limit_s),
        "candidate_count": int(len(pool)),
    }
    if not feasible:
        base.update({
            "makespan_s": None, "best_bound_s": None, "gap_pct": None,
            "boxes": None, "unique_boxes": None, "hard_violations": None,
            "soft_late_boxes": None, "soft_total_lateness_s": None,
            "energy_kwh": None, "A_sorties": None, "B_sorties": None,
            "C_sorties": None, "multi_point_sorties": None,
        })
        return base, None, None

    selected = []
    for i, c in enumerate(candidates):
        if solver.Value(c["x"]):
            r = c["row"].to_dict()
            r["start_s"] = solver.Value(c["s"]) / q2.SCALE
            r["return_s"] = solver.Value(c["e"]) / q2.SCALE
            selected.append(r)
    sch = pd.DataFrame(selected).sort_values(["start_s", "return_s"]).reset_index(drop=True)
    sch["battery_end_s"] = sch.apply(
        lambda r: float(r.start_s)
        + math.ceil((float(r.duration_s) + float(r.charge_s)) * q2.SCALE - 1e-9) / q2.SCALE,
        axis=1,
    )

    # Reconstruct concrete aircraft/battery IDs exactly as in Q2-v2.
    def color(group, end_col, labels):
        active = {lab: 0.0 for lab in labels}
        ans = {}
        for idx, r in group.sort_values("start_s").iterrows():
            free = [lab for lab, t in active.items() if t <= float(r.start_s) + 1e-9]
            if not free:
                raise RuntimeError(f"K={k}: interval coloring failed for {end_col}")
            lab = min(free, key=lambda z: active[z])
            ans[idx] = lab
            active[lab] = float(r[end_col])
        return ans

    aircraft = {}
    battery = {}
    for typ, g in sch.groupby("drone_type"):
        aircraft.update(color(g, "return_s", list(data["aircraft"][typ])))
        blabs = [f"{typ}-B{i+1:02d}" for i in range(int(data["batteries"][typ]))]
        battery.update(color(g, "battery_end_s", blabs))
    sch["aircraft"] = pd.Series(aircraft)
    sch["battery"] = pd.Series(battery)

    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    deliveries = []
    for idx, r in sch.iterrows():
        sid = f"K{k}-S{idx+1:03d}"
        offs = json.loads(r.delivery_offsets)
        for bid, off in offs.items():
            b = bm[str(bid)]
            ct = float(r.start_s) + float(off)
            hd = q2.box_hard_deadline(b)
            soft = None if b.get("物资类型") == "医疗物资" else float(b["期望送达时间（s）"])
            deliveries.append({
                "box_id": str(bid), "sortie": sid, "completion_s": ct,
                "hard_deadline_s": hd,
                "hard_ok": hd is None or ct <= hd + 1e-6,
                "soft_deadline_s": soft,
                "soft_lateness_s": 0.0 if soft is None else max(0.0, ct - soft),
            })
    dl = pd.DataFrame(deliveries)

    cmax_val = float(sch.return_s.max())
    bound = float(solver.BestObjectiveBound()) / q2.SCALE
    gap = max(0.0, (cmax_val - bound) / cmax_val * 100.0) if cmax_val > 0 else 0.0
    visits = sch.visit_order.astype(str).str.count(">") + 1

    base.update({
        "makespan_s": cmax_val,
        "best_bound_s": bound,
        "gap_pct": gap,
        "boxes": int(len(dl)),
        "unique_boxes": int(dl.box_id.nunique()),
        "hard_violations": int((~dl.hard_ok).sum()),
        "soft_late_boxes": int((dl.soft_lateness_s > 1e-6).sum()),
        "soft_total_lateness_s": float(dl.soft_lateness_s.sum()),
        "energy_kwh": float(sch.energy_kwh.sum()),
        "A_sorties": int((sch.drone_type == "A").sum()),
        "B_sorties": int((sch.drone_type == "B").sum()),
        "C_sorties": int((sch.drone_type == "C").sum()),
        "multi_point_sorties": int((visits > 1).sum()),
    })
    return base, sch, dl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--k", type=int, required=True)
    ap.add_argument("--time-limit", type=int, default=360)
    ap.add_argument("--triple-limit", type=int, default=900)
    args = ap.parse_args()

    if not (18 <= args.k <= 28):
        raise SystemExit("--k must be in [18, 28]")

    data = q2.load_inputs()
    pool = q2.generate_pool(data, triple_limit=args.triple_limit)
    summary, sch, dl = solve_fixed_k(data, pool, args.k, args.time_limit)

    kd = OUT / f"K{args.k:02d}"
    kd.mkdir(parents=True, exist_ok=True)
    (kd / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    if sch is not None:
        sch.to_csv(kd / "schedule.csv", index=False, encoding="utf-8-sig")
        dl.to_csv(kd / "deliveries.csv", index=False, encoding="utf-8-sig")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
