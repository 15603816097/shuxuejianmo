"""Q2 lexicographic optimization:
Phase 1: minimize total soft lateness of non-medical boxes.
Phase 2: fix the best lateness found in phase 1 and minimize makespan.

All physical/resource/hard-deadline semantics use the current Q2-v2 candidate pool.
A FEASIBLE phase-1 result is a best-known incumbent unless CP-SAT returns OPTIMAL.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import pandas as pd
from ortools.sat.python import cp_model

import q2_v2_solver as q2

OUT = q2.OUT / "lexicographic"
OUT.mkdir(parents=True, exist_ok=True)


def build_model(data, pool, objective, lateness_cap_ticks=None, hint_schedule=None):
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    box_ids = sorted(bm)
    if len(box_ids) != 80:
        raise RuntimeError(f"expected 80 boxes, got {len(box_ids)}")

    m = cp_model.CpModel()
    H = int(math.ceil(13000.0 * q2.SCALE))

    candidates = []
    by_box = {b: [] for b in box_ids}
    xvars, svars, evars = [], [], []
    air_ints, bat_ints = {}, {}

    for i, r in pool.iterrows():
        ids = [str(x) for x in json.loads(r.box_ids)]
        offs = {str(k): float(v) for k, v in json.loads(r.delivery_offsets).items()}
        dur = int(math.ceil(float(r.duration_s) * q2.SCALE - 1e-9))
        bdur = int(math.ceil((float(r.duration_s) + float(r.charge_s)) * q2.SCALE - 1e-9))

        x = m.NewBoolVar(f"x_{i}")
        s = m.NewIntVar(0, H, f"s_{i}")
        e = m.NewIntVar(0, H, f"e_{i}")
        be = m.NewIntVar(0, H + bdur, f"be_{i}")

        ai = m.NewOptionalIntervalVar(s, dur, e, x, f"air_{i}")
        bi = m.NewOptionalIntervalVar(s, bdur, be, x, f"bat_{i}")

        m.Add(s == 0).OnlyEnforceIf(x.Not())
        m.Add(e == 0).OnlyEnforceIf(x.Not())
        m.Add(be == 0).OnlyEnforceIf(x.Not())

        if pd.notna(r.latest_start_s):
            latest = int(math.floor(float(r.latest_start_s) * q2.SCALE + 1e-9))
            m.Add(s <= latest).OnlyEnforceIf(x)

        typ = str(r.drone_type)
        air_ints.setdefault(typ, []).append(ai)
        bat_ints.setdefault(typ, []).append(bi)

        candidates.append({"row": r, "ids": ids, "offs": offs, "x": x, "s": s, "e": e})
        xvars.append(x); svars.append(s); evars.append(e)
        for bid in ids:
            by_box[bid].append(i)

    # Every official box is delivered exactly once.
    for bid in box_ids:
        idx = by_box[bid]
        if not idx:
            raise RuntimeError(f"candidate pool misses {bid}")
        m.Add(sum(xvars[i] for i in idx) == 1)

    # Shared aircraft and battery resources.
    for typ, ints in air_ints.items():
        m.AddCumulative(ints, [1] * len(ints), len(data["aircraft"][typ]))
    for typ, ints in bat_ints.items():
        m.AddCumulative(ints, [1] * len(ints), int(data["batteries"][typ]))

    cmax = m.NewIntVar(0, H, "Cmax")
    for c in candidates:
        m.Add(cmax >= c["e"]).OnlyEnforceIf(c["x"])

    # Valid type-wise workload lower bound.
    for typ in sorted(air_ints):
        idx = [i for i, c in enumerate(candidates) if str(c["row"].drone_type) == typ]
        terms = []
        for i in idx:
            dur = int(math.ceil(float(candidates[i]["row"].duration_s) * q2.SCALE - 1e-9))
            terms.append(dur * xvars[i])
        if terms:
            m.Add(sum(terms) <= len(data["aircraft"][typ]) * cmax)

    # Soft lateness variables. Medical expected times are already hard deadlines
    # and therefore are not counted as soft lateness.
    late_vars = []
    for bid in box_ids:
        b = bm[bid]
        if b.get("物资类型") == "医疗物资":
            continue
        due = int(round(float(b["期望送达时间（s）"]) * q2.SCALE))
        lv = m.NewIntVar(0, H, f"late_{bid}")
        for i in by_box[bid]:
            off = int(math.ceil(candidates[i]["offs"][bid] * q2.SCALE - 1e-9))
            m.Add(lv >= svars[i] + off - due).OnlyEnforceIf(xvars[i])
        late_vars.append(lv)

    total_late = sum(late_vars) if late_vars else 0
    if lateness_cap_ticks is not None:
        m.Add(total_late <= int(lateness_cap_ticks))

    # Warm start from an audited incumbent or from phase 1.
    if hint_schedule is not None and len(hint_schedule):
        hint = {str(r.candidate_id): float(r.start_s) for _, r in hint_schedule.iterrows()}
        for i, c in enumerate(candidates):
            cid = str(c["row"].candidate_id)
            if cid in hint:
                m.AddHint(xvars[i], 1)
                m.AddHint(svars[i], int(round(hint[cid] * q2.SCALE)))
            else:
                m.AddHint(xvars[i], 0)

    if objective == "lateness":
        m.Minimize(total_late)
    elif objective == "makespan":
        m.Minimize(cmax)
    else:
        raise ValueError(objective)

    return m, candidates, xvars, cmax, total_late, bm


def reconstruct(data, candidates, solver, bm):
    rows = []
    for c in candidates:
        if solver.Value(c["x"]):
            r = c["row"].to_dict()
            r["start_s"] = solver.Value(c["s"]) / q2.SCALE
            r["return_s"] = solver.Value(c["e"]) / q2.SCALE
            rows.append(r)

    sch = pd.DataFrame(rows).sort_values(["start_s", "return_s"]).reset_index(drop=True)
    sch["battery_end_s"] = sch.apply(
        lambda r: float(r.start_s)
        + math.ceil((float(r.duration_s) + float(r.charge_s)) * q2.SCALE - 1e-9) / q2.SCALE,
        axis=1,
    )

    def color(group, end_col, labels):
        active = {lab: 0.0 for lab in labels}
        ans = {}
        for idx, r in group.sort_values("start_s").iterrows():
            free = [lab for lab, t in active.items() if t <= float(r.start_s) + 1e-9]
            if not free:
                raise RuntimeError(f"interval coloring failed for {end_col}")
            lab = min(free, key=lambda z: active[z])
            ans[idx] = lab
            active[lab] = float(r[end_col])
        return ans

    aircraft, battery = {}, {}
    for typ, g in sch.groupby("drone_type"):
        aircraft.update(color(g, "return_s", list(data["aircraft"][typ])))
        blabs = [f"{typ}-B{i+1:02d}" for i in range(int(data["batteries"][typ]))]
        battery.update(color(g, "battery_end_s", blabs))
    sch["aircraft"] = pd.Series(aircraft)
    sch["battery"] = pd.Series(battery)
    sch["sortie"] = [f"Q2LEX-S{i+1:03d}" for i in range(len(sch))]

    deliveries = []
    for _, r in sch.iterrows():
        offs = json.loads(r.delivery_offsets)
        for bid, off in offs.items():
            b = bm[str(bid)]
            ct = float(r.start_s) + float(off)
            hd = q2.box_hard_deadline(b)
            soft = None if b.get("物资类型") == "医疗物资" else float(b["期望送达时间（s）"])
            deliveries.append({
                "box_id": str(bid),
                "sortie": r.sortie,
                "completion_s": ct,
                "hard_deadline_s": hd,
                "hard_ok": hd is None or ct <= hd + 1e-6,
                "soft_deadline_s": soft,
                "soft_lateness_s": 0.0 if soft is None else max(0.0, ct - soft),
            })
    dl = pd.DataFrame(deliveries)
    return sch, dl


def solve_phase(data, pool, objective, time_limit_s, lateness_cap_ticks=None, hint_schedule=None):
    m, candidates, xvars, cmax, total_late, bm = build_model(
        data, pool, objective, lateness_cap_ticks=lateness_cap_ticks, hint_schedule=hint_schedule
    )
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit_s)
    solver.parameters.num_search_workers = 8
    solver.parameters.log_search_progress = True

    t0 = time.time()
    status = solver.Solve(m)
    elapsed = time.time() - t0
    name = solver.StatusName(status)
    feasible = status in (cp_model.FEASIBLE, cp_model.OPTIMAL)

    summary = {
        "objective_kind": objective,
        "status": name,
        "runtime_s": elapsed,
        "candidate_count": len(pool),
        "feasible_found": bool(feasible),
        "proven_optimal_current_pool": bool(status == cp_model.OPTIMAL),
        "lateness_cap_s": None if lateness_cap_ticks is None else lateness_cap_ticks / q2.SCALE,
    }
    if not feasible:
        return summary, None, None, None

    sch, dl = reconstruct(data, candidates, solver, bm)
    actual_late = float(dl.soft_lateness_s.sum())
    summary.update({
        "K": int(len(sch)),
        "makespan_s": float(sch.return_s.max()),
        "solver_cmax_s": solver.Value(cmax) / q2.SCALE,
        "soft_late_boxes": int((dl.soft_lateness_s > 1e-6).sum()),
        "soft_total_lateness_s": actual_late,
        "solver_total_lateness_s": solver.Value(total_late) / q2.SCALE,
        "hard_violations": int((~dl.hard_ok).sum()),
        "boxes": int(len(dl)),
        "unique_boxes": int(dl.box_id.nunique()),
        "energy_kwh": float(sch.energy_kwh.sum()),
        "A_sorties": int((sch.drone_type == "A").sum()),
        "B_sorties": int((sch.drone_type == "B").sum()),
        "C_sorties": int((sch.drone_type == "C").sum()),
        "objective_value_s": solver.ObjectiveValue() / q2.SCALE,
        "best_bound_s": solver.BestObjectiveBound() / q2.SCALE,
    })
    return summary, sch, dl, int(round(solver.Value(total_late)))


def save_phase(name, summary, sch, dl):
    d = OUT / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    if sch is not None:
        sch.to_csv(d / "schedule.csv", index=False, encoding="utf-8-sig")
        dl.to_csv(d / "deliveries.csv", index=False, encoding="utf-8-sig")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase1-time", type=int, default=600)
    ap.add_argument("--phase2-time", type=int, default=360)
    ap.add_argument("--triple-limit", type=int, default=900)
    args = ap.parse_args()

    data = q2.load_inputs()
    pool = q2.generate_pool(data, args.triple_limit)
    pool.to_csv(OUT / "candidate_pool.csv", index=False, encoding="utf-8-sig")

    # Audited 6043.8 s checkpoint gives phase 1 a valid starting incumbent.
    hint0 = q2.known_6043_hint()
    p1, s1, d1, late_ticks = solve_phase(
        data, pool, "lateness", args.phase1_time, hint_schedule=hint0
    )
    save_phase("phase1_min_lateness", p1, s1, d1)
    print(json.dumps({"phase": 1, **p1}, ensure_ascii=False))

    p2 = None
    if s1 is not None:
        # Preserve the best lateness found, then minimize makespan.
        hint1 = s1[["candidate_id", "start_s"]].copy()
        p2, s2, d2, _ = solve_phase(
            data, pool, "makespan", args.phase2_time,
            lateness_cap_ticks=late_ticks, hint_schedule=hint1
        )
        save_phase("phase2_min_makespan", p2, s2, d2)
        print(json.dumps({"phase": 2, **p2}, ensure_ascii=False))

    (OUT / "run_summary.json").write_text(
        json.dumps({"phase1": p1, "phase2": p2}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
