"""Q2-v2: multi-point route-pool + CP-SAT resource scheduling.

This replaces the legacy fixed-K single-point heuristic for Q2.  It keeps Q1
frozen and uses the official Q2 transport/resource rules only (no communication).
"""
from __future__ import annotations

import argparse, ast, itertools, json, math, time
from pathlib import Path
from collections import defaultdict

import pandas as pd

from minimal_pipeline import load_inputs, q1, line_geometry, energy_leg

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
OUT = RES / "q2_v2"
OUT.mkdir(parents=True, exist_ok=True)
SCALE = 10  # 0.1 s, conservative ceil for durations/deadline offsets
TOL = 1e-9

# Best independently audited incumbent from GitHub Actions run 36166631572.
# This is a search hint/checkpoint, not a global-optimality claim.
KNOWN_6043_STARTS = {
    "Q2V2-000369": 0.0, "Q2V2-000241": 0.0, "Q2V2-000242": 0.0,
    "Q2V2-000154": 0.0, "Q2V2-000257": 0.0, "Q2V2-000480": 0.0,
    "Q2V2-002252": 0.0, "Q2V2-000253": 0.1, "Q2V2-000387": 1312.7,
    "Q2V2-000256": 1554.7, "Q2V2-000298": 1561.5, "Q2V2-000248": 1627.5,
    "Q2V2-000368": 1667.4, "Q2V2-000255": 1868.6, "Q2V2-000403": 2175.5,
    "Q2V2-000058": 2859.1, "Q2V2-000252": 3065.2, "Q2V2-002449": 3133.3,
    "Q2V2-000169": 3489.1, "Q2V2-000319": 3507.6, "Q2V2-000247": 3738.6,
    "Q2V2-000083": 3792.5, "Q2V2-000136": 4351.8, "Q2V2-000140": 4626.0,
    "Q2V2-000440": 4806.9,
}

def known_6043_hint():
    return pd.DataFrame([
        {"candidate_id": cid, "start_s": st} for cid, st in KNOWN_6043_STARTS.items()
    ])


def box_hard_deadline(b):
    vals = []
    if b.get("物资类型") == "医疗物资":
        vals.append(float(b["期望送达时间（s）"]))
    if b.get("是否首批保障") == "是":
        vals.append(float(b["首批截止时间（s）"]))
    return min(vals) if vals else None


def charge_seconds(drone_type, drone, energy_kwh):
    soc = 1.0 - float(energy_kwh) / float(drone.energy)
    full = {"A": 1800.0, "B": 2400.0, "C": 3000.0}[str(drone_type)]
    if soc < 0.9:
        raw = full * (0.65 * (0.9 - soc) / 0.9 + 0.35)
    else:
        raw = full * 0.35 * (1.0 - soc) / 0.1
    return max(0.0, raw)


def evaluate_candidate(data, visit_order, drone_type, box_ids):
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    ids = tuple(str(x) for x in box_ids)
    boxes = [bm[x] for x in ids]
    d = data["drones"][drone_type]
    mass = sum(float(x["单箱质量（kg）"]) for x in boxes)
    volume = sum(float(x["单箱体积（m³）"]) for x in boxes)
    if mass > d.max_mass + TOL or volume > d.volume + TOL:
        return None
    services = {str(x["服务区编号"]) for x in boxes}
    if services != set(visit_order):
        return None

    nm = {"O01": data["center"], **{str(x["服务区编号"]): x for x in data["nodes"]}}
    payload = mass
    t = float(d.prep + d.load_box * len(ids))
    energy = 0.0
    deliveries = {}
    prev = "O01"

    for nxt in tuple(visit_order) + ("O01",):
        a, b = nm[prev], nm[nxt]
        h0 = float(a["海拔（m）"]) + (30.0 if prev != "O01" else 0.0)
        h1 = float(b["海拔（m）"]) + (30.0 if nxt != "O01" else 0.0)
        dist, peak, *_ = line_geometry(a, b, data["dem"])
        e, dt = energy_leg(d, dist, peak, h0, h1, payload)
        energy += float(e)
        t += float(dt)
        if nxt != "O01":
            here = [x for x in boxes if str(x["服务区编号"]) == nxt]
            # Q2 legacy/audited interpretation: one service handoff block,
            # base handoff + per-box increment; all boxes complete at block end.
            t += float(d.handoff + d.handoff_box * len(here))
            for x in here:
                deliveries[str(x["货箱编号"])] = t
                payload -= float(x["单箱质量（kg）"])
        prev = nxt

    usable = (1.0 - float(d.reserve)) * float(d.energy)
    if energy > usable + TOL:
        return None

    latest = math.inf
    for bid, off in deliveries.items():
        hd = box_hard_deadline(bm[bid])
        if hd is not None:
            latest = min(latest, float(hd) - float(off))
    if latest < -TOL:
        return None

    return {
        "visit_order": ">".join(visit_order),
        "drone_type": str(drone_type),
        "box_ids": list(ids),
        "box_count": len(ids),
        "mass_kg": mass,
        "volume_m3": volume,
        "duration_s": t,
        "energy_kwh": energy,
        "soc_return": 1.0 - energy / float(d.energy),
        "charge_s": charge_seconds(drone_type, d, energy),
        "latest_start_s": None if math.isinf(latest) else max(0.0, latest),
        "delivery_offsets": deliveries,
    }


def skeletons(data, triple_limit=900):
    services = sorted(str(x["服务区编号"]) for x in data["nodes"])
    nm = {"O01": data["center"], **{str(x["服务区编号"]): x for x in data["nodes"]}}
    dist = {}
    for a in ["O01"] + services:
        for b in ["O01"] + services:
            if a == b:
                continue
            dist[a, b] = float(line_geometry(nm[a], nm[b], data["dem"])[0])
    out = [(s,) for s in services]
    out += list(itertools.permutations(services, 2))
    triples = []
    for r in itertools.permutations(services, 3):
        cost = dist["O01", r[0]] + dist[r[0], r[1]] + dist[r[1], r[2]] + dist[r[2], "O01"]
        triples.append((cost, r))
    triples.sort(key=lambda z: z[0])
    kept = [r for _, r in triples[:triple_limit]]
    # Coverage guard: each service must appear in a healthy number of triples.
    for s in services:
        extra = [r for _, r in triples if s in r][:40]
        kept.extend(extra)
    out += sorted(set(kept))
    return out


def _priority_rows(rows, mode):
    def hard(b):
        d = box_hard_deadline(b)
        return d if d is not None else 10**12
    if mode == "deadline":
        return sorted(rows, key=lambda b: (hard(b), float(b["期望送达时间（s）"]), -float(b["单箱质量（kg）"])))
    if mode == "heavy":
        return sorted(rows, key=lambda b: (-float(b["单箱质量（kg）"]), hard(b)))
    if mode == "volume":
        return sorted(rows, key=lambda b: (-float(b["单箱体积（m³）"]), hard(b)))
    if mode == "light":
        return sorted(rows, key=lambda b: (float(b["单箱质量（kg）"]), hard(b)))
    return sorted(rows, key=lambda b: (float(b["期望送达时间（s）"]), hard(b)))


def bundles_for_skeleton(data, skel, typ):
    d = data["drones"][typ]
    bys = defaultdict(list)
    for b in data["boxes"]:
        if str(b["服务区编号"]) in skel:
            bys[str(b["服务区编号"])].append(b)
    results = set()
    modes = ("deadline", "heavy", "volume", "light", "expected")
    for mode in modes:
        ordered = {s: _priority_rows(bys[s], mode) for s in skel}
        # Seed every visited service so the route skeleton is semantically exact.
        seed = [ordered[s][0] for s in skel if ordered[s]]
        if len(seed) != len(skel):
            continue
        pool = []
        seen = {str(x["货箱编号"]) for x in seed}
        for s in skel:
            pool.extend([x for x in ordered[s] if str(x["货箱编号"]) not in seen])
        chosen = list(seed)
        m = sum(float(x["单箱质量（kg）"]) for x in chosen)
        v = sum(float(x["单箱体积（m³）"]) for x in chosen)
        snapshots = []
        if m <= d.max_mass + TOL and v <= d.volume + TOL:
            snapshots.append(tuple(sorted(str(x["货箱编号"]) for x in chosen)))
        for b in pool:
            nm = m + float(b["单箱质量（kg）"])
            nv = v + float(b["单箱体积（m³）"])
            if nm <= d.max_mass + TOL and nv <= d.volume + TOL:
                chosen.append(b); m = nm; v = nv
                if len(chosen) in {len(skel)+1, len(skel)+2, len(skel)+4, len(skel)+6}:
                    snapshots.append(tuple(sorted(str(x["货箱编号"]) for x in chosen)))
        snapshots.append(tuple(sorted(str(x["货箱编号"]) for x in chosen)))
        results.update(x for x in snapshots if x)
    return sorted(results)


def generate_pool(data, triple_limit=900):
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    seen = set(); rows = []

    def add(order, typ, ids, source):
        key = (tuple(order), str(typ), tuple(sorted(str(x) for x in ids)))
        if key in seen:
            return
        z = evaluate_candidate(data, tuple(order), str(typ), key[2])
        if z is None:
            return
        seen.add(key)
        z["source"] = source
        rows.append(z)

    # Guaranteed coverage: single-box candidates for every feasible type.
    for bid, b in bm.items():
        for typ in data["drones"]:
            add((str(b["服务区编号"]),), typ, (bid,), "single_box_guard")

    # Frozen Q1 batches are fallback columns, not the new Q2 solution.
    _, qb = q1(data)
    for _, r in qb.iterrows():
        ids = r["box_ids"] if isinstance(r["box_ids"], list) else ast.literal_eval(str(r["box_ids"]))
        add((str(r["service"]),), str(r["type"]), ids, "q1_fallback")

    for sk in skeletons(data, triple_limit=triple_limit):
        for typ in data["drones"]:
            for ids in bundles_for_skeleton(data, sk, typ):
                add(sk, typ, ids, "multi_point_pool")

    df = pd.DataFrame(rows)
    if df.empty:
        raise RuntimeError("empty Q2-v2 candidate pool")
    df.insert(0, "candidate_id", [f"Q2V2-{i:06d}" for i in range(1, len(df)+1)])
    df["box_ids"] = df["box_ids"].map(json.dumps)
    df["delivery_offsets"] = df["delivery_offsets"].map(json.dumps)
    return df


def solve(data, pool, target_s, time_limit_s, optimize_makespan, hint_schedule=None):
    from ortools.sat.python import cp_model
    m = cp_model.CpModel()
    box_ids = sorted(str(x["货箱编号"]) for x in data["boxes"])
    if len(box_ids) != 80:
        raise RuntimeError(f"expected 80 boxes, got {len(box_ids)}")
    candidates = []
    by_box = defaultdict(list)
    intervals_air = defaultdict(list)
    intervals_bat = defaultdict(list)
    xvars = []; svars = []; evars = []; bevars = []

    # Keep the scheduling horizon tight.  A loose 18k-26k second domain made
    # the 7000 s feasibility model unnecessarily difficult.
    horizon_s = float(target_s) if target_s is not None else 13000.0
    H = int(math.ceil(horizon_s * SCALE))

    for i, r in pool.iterrows():
        ids = json.loads(r.box_ids)
        offs = json.loads(r.delivery_offsets)
        dur = int(math.ceil(float(r.duration_s) * SCALE - 1e-9))
        bdur = int(math.ceil((float(r.duration_s) + float(r.charge_s)) * SCALE - 1e-9))
        xv = m.NewBoolVar(f"x_{i}")
        sv = m.NewIntVar(0, H, f"s_{i}")
        ev = m.NewIntVar(0, H, f"e_{i}")
        # Battery recharge may legitimately finish after the transport makespan.
        bev = m.NewIntVar(0, H + bdur, f"be_{i}")
        ai = m.NewOptionalIntervalVar(sv, dur, ev, xv, f"air_{i}")
        bi = m.NewOptionalIntervalVar(sv, bdur, bev, xv, f"bat_{i}")
        m.Add(sv == 0).OnlyEnforceIf(xv.Not())
        m.Add(ev == 0).OnlyEnforceIf(xv.Not())
        m.Add(bev == 0).OnlyEnforceIf(xv.Not())
        if pd.notna(r.latest_start_s):
            latest = int(math.floor(float(r.latest_start_s) * SCALE + 1e-9))
            m.Add(sv <= latest).OnlyEnforceIf(xv)
        xvars.append(xv); svars.append(sv); evars.append(ev); bevars.append(bev)
        typ = str(r.drone_type)
        intervals_air[typ].append(ai); intervals_bat[typ].append(bi)
        ci = {"row": r, "ids": ids, "offs": {str(k): float(v) for k,v in offs.items()}, "x": xv, "s": sv, "e": ev}
        candidates.append(ci)
        for b in ids:
            by_box[str(b)].append(i)

    for b in box_ids:
        idx = by_box.get(b, [])
        if not idx:
            raise RuntimeError(f"candidate pool does not cover {b}")
        m.Add(sum(xvars[i] for i in idx) == 1)

    for typ, ints in intervals_air.items():
        m.AddCumulative(ints, [1]*len(ints), len(data["aircraft"][typ]))
    for typ, ints in intervals_bat.items():
        m.AddCumulative(ints, [1]*len(ints), int(data["batteries"][typ]))

    cmax = m.NewIntVar(0, H, "Cmax")
    for i, c in enumerate(candidates):
        m.Add(cmax >= c["e"]).OnlyEnforceIf(c["x"])
    if target_s is not None:
        m.Add(cmax <= int(math.floor(float(target_s) * SCALE + 1e-9)))

    # Valid workload lower bounds strengthen the parallel-aircraft schedule.
    # Every selected transport interval must fit before Cmax.
    for typ in sorted(intervals_air):
        idx = [i for i,c in enumerate(candidates) if str(c["row"].drone_type) == typ]
        if idx:
            dur_terms = []
            for i in idx:
                dur_i = int(math.ceil(float(candidates[i]["row"].duration_s) * SCALE - 1e-9))
                dur_terms.append(dur_i * xvars[i])
            m.Add(sum(dur_terms) <= len(data["aircraft"][typ]) * cmax)

    # Warm-start a stricter target run from the best makespan incumbent.
    if hint_schedule is not None and len(hint_schedule):
        hint = {str(r.candidate_id): float(r.start_s) for _,r in hint_schedule.iterrows()}
        for i,c in enumerate(candidates):
            cid = str(c["row"].candidate_id)
            if cid in hint:
                m.AddHint(xvars[i], 1)
                m.AddHint(svars[i], int(round(hint[cid] * SCALE)))
            else:
                m.AddHint(xvars[i], 0)

    # Soft lateness for non-medical expected delivery times.
    late_vars = []
    bm = {str(x["货箱编号"]): x for x in data["boxes"]}
    for b in box_ids:
        if bm[b].get("物资类型") == "医疗物资":
            continue
        due = int(round(float(bm[b]["期望送达时间（s）"]) * SCALE))
        lv = m.NewIntVar(0, H, f"late_{b}")
        for i in by_box[b]:
            off = int(math.ceil(candidates[i]["offs"][b] * SCALE - 1e-9))
            m.Add(lv >= svars[i] + off - due).OnlyEnforceIf(xvars[i])
        late_vars.append(lv)

    sortie_count = sum(xvars)
    total_late = sum(late_vars) if late_vars else 0
    if optimize_makespan:
        # Phase 1 is deliberately pure makespan minimization.  Mixing lateness
        # and route-count terms here greatly enlarged the search tree.
        m.Minimize(cmax)
    else:
        # Pure feasibility under the requested Cmax cap.  Secondary criteria
        # are optimized only after feasibility has been established.
        pass

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit_s)
    solver.parameters.num_search_workers = 8
    solver.parameters.log_search_progress = True
    t0 = time.time()
    status = solver.Solve(m)
    elapsed = time.time() - t0
    name = solver.StatusName(status)
    summary = {
        "status": name,
        "target_s": target_s,
        "optimize_makespan": bool(optimize_makespan),
        "runtime_s": elapsed,
        "candidate_count": len(pool),
        "objective": solver.ObjectiveValue() if status in (cp_model.OPTIMAL, cp_model.FEASIBLE) and optimize_makespan else None,
        "best_bound": solver.BestObjectiveBound() if status in (cp_model.OPTIMAL, cp_model.FEASIBLE) and optimize_makespan else None,
        "solver_cmax_s": solver.Value(cmax) / SCALE if status in (cp_model.OPTIMAL, cp_model.FEASIBLE) else None,
    }
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return summary, sch, None, None

    selected = []
    for i, c in enumerate(candidates):
        if solver.Value(c["x"]):
            r = c["row"].to_dict()
            r["start_s"] = solver.Value(c["s"]) / SCALE
            r["return_s"] = solver.Value(c["e"]) / SCALE
            selected.append(r)
    sch = pd.DataFrame(selected).sort_values(["start_s", "return_s"]).reset_index(drop=True)

    def color(group, end_col, labels):
        active = {lab: 0.0 for lab in labels}; ans = {}
        for idx, r in group.sort_values("start_s").iterrows():
            free = [lab for lab,t in active.items() if t <= float(r.start_s)+1e-9]
            if not free:
                raise RuntimeError("interval coloring failed despite cumulative feasibility")
            lab = min(free, key=lambda q: active[q])
            ans[idx] = lab
            active[lab] = float(r[end_col])
        return ans

    # Match the exact integer interval used by CP-SAT.  Using
    # return_s + raw charge_s can be up to one discretization tick longer than
    # the modeled battery interval and previously caused a false coloring crash.
    sch["battery_end_s"] = sch.apply(
        lambda r: float(r.start_s) + math.ceil((float(r.duration_s) + float(r.charge_s)) * SCALE - 1e-9) / SCALE,
        axis=1,
    )
    aircraft = {}
    battery = {}
    for typ, g in sch.groupby("drone_type"):
        aircraft.update(color(g, "return_s", list(data["aircraft"][typ])))
        blabs = [f"{typ}-B{i+1:02d}" for i in range(int(data["batteries"][typ]))]
        battery.update(color(g, "battery_end_s", blabs))
    sch["aircraft"] = pd.Series(aircraft)
    sch["battery"] = pd.Series(battery)
    sch["sortie"] = [f"Q2V2-S{i+1:03d}" for i in range(len(sch))]

    deliveries = []
    for _, r in sch.iterrows():
        offs = json.loads(r.delivery_offsets)
        for bid, off in offs.items():
            b = bm[str(bid)]
            ct = float(r.start_s) + float(off)
            hd = box_hard_deadline(b)
            soft = None if b.get("物资类型") == "医疗物资" else float(b["期望送达时间（s）"])
            deliveries.append({
                "box_id": str(bid), "sortie": r.sortie, "completion_s": ct,
                "hard_deadline_s": hd, "hard_ok": hd is None or ct <= hd + 1e-6,
                "soft_deadline_s": soft,
                "soft_lateness_s": 0.0 if soft is None else max(0.0, ct-soft),
            })
    dl = pd.DataFrame(deliveries)
    summary.update({
        "makespan_s": float(sch.return_s.max()),
        "sorties": int(len(sch)),
        "boxes": int(len(dl)),
        "unique_boxes": int(dl.box_id.nunique()),
        "hard_violations": int((~dl.hard_ok).sum()),
        "soft_late_boxes": int((dl.soft_lateness_s > 1e-6).sum()),
        "soft_total_lateness_s": float(dl.soft_lateness_s.sum()),
        "transport_energy_kwh": float(sch.energy_kwh.sum()),
        "target_feasible": None if target_s is None else float(sch.return_s.max()) <= float(target_s)+1e-6,
    })
    return summary, sch, dl


def run_case(data, pool, name, target, time_limit, optimize, hint_schedule=None):
    summary, sch, dl = solve(data, pool, target, time_limit, optimize, hint_schedule=hint_schedule)
    (OUT / f"{name}_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    if sch is not None:
        sch.to_csv(OUT / f"{name}_schedule.csv", index=False, encoding="utf-8-sig")
        dl.to_csv(OUT / f"{name}_deliveries.csv", index=False, encoding="utf-8-sig")
    print(json.dumps({"case": name, **summary}, ensure_ascii=False))
    return summary, sch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--triple-limit", type=int, default=900)
    ap.add_argument("--time-limit", type=int, default=300)
    ap.add_argument("--target", type=float, default=7000.0)
    ap.add_argument("--skip-optimize", action="store_true")
    args = ap.parse_args()

    data = load_inputs()
    pool_path = OUT / "candidate_pool.csv"
    pool = generate_pool(data, args.triple_limit)
    pool.to_csv(pool_path, index=False, encoding="utf-8-sig")
    coverage = set()
    for z in pool.box_ids:
        coverage.update(json.loads(z))
    print(json.dumps({"candidate_count": len(pool), "coverage": len(coverage)}, ensure_ascii=False))
    if len(coverage) != 80:
        raise RuntimeError(f"Q2-v2 pool covers only {len(coverage)}/80 boxes")

    best = None
    best_schedule = None
    checkpoint_hint = known_6043_hint()
    if not args.skip_optimize:
        best, best_schedule = run_case(
            data, pool, "min_makespan", None, args.time_limit, True,
            hint_schedule=checkpoint_hint,
        )

    target_name = f"target{int(round(args.target))}"
    # A makespan incumbent <= target is itself a constructive feasibility
    # certificate for that target; do not waste another CP-SAT phase.
    if best_schedule is not None and float(best_schedule.return_s.max()) <= float(args.target) + 1e-6:
        target = {
            "status": "FEASIBLE_BY_CONSTRUCTIVE_INCUMBENT",
            "target_s": float(args.target),
            "makespan_s": float(best_schedule.return_s.max()),
            "boxes": int(sum(len(json.loads(x)) for x in best_schedule.box_ids)),
            "unique_boxes": 80,
            "hard_violations": int(best.get("hard_violations", 0)),
            "target_feasible": True,
            "source": "min_makespan incumbent",
        }
        best_schedule.to_csv(OUT / f"{target_name}_schedule.csv", index=False, encoding="utf-8-sig")
    else:
        target, _ = run_case(
            data, pool, target_name, args.target, args.time_limit, False,
            hint_schedule=best_schedule if best_schedule is not None else checkpoint_hint,
        )

    (OUT / "run_summary.json").write_text(
        json.dumps({"target": target, "min_makespan": best}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
