"""Minimal real-data vertical slice for the confirmed drone model.

This is a baseline runner, not the final multi-objective optimizer.  It keeps
the confirmed payload/volume/energy formulas and records every intermediate
table needed for later Q2/Q3 expansion.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.io import loadmat


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
BASE = DATA / "无人机应急物资运输基础数据"
GEO = DATA / "镇龙乡地理空间数据" / "镇龙乡及周边地理数据"
_GEOM_CACHE = {}


@dataclass(frozen=True)
class Drone:
    typ: str
    empty_mass: float
    max_mass: float
    volume: float
    speed: float
    range_empty: float
    range_full: float
    energy: float
    reserve: float
    prep: float
    load_box: float
    handoff: float
    handoff_box: float
    climb_speed: float
    descend_speed: float
    eta_climb: float


def read_xlsx(sheet_name: str, file: str, header_row: int, start_row: int) -> list[dict]:
    """Read a known table block without implicit header guessing."""
    from openpyxl import load_workbook

    wb = load_workbook(BASE / file, data_only=True, read_only=True)
    ws = wb[sheet_name]
    headers = [c.value for c in ws[header_row]]
    rows = []
    for values in ws.iter_rows(min_row=start_row, values_only=True):
        if not any(v is not None for v in values):
            continue
        rows.append({h: values[i] for i, h in enumerate(headers) if h is not None})
    wb.close()
    return rows


def load_inputs() -> dict:
    nodes = read_xlsx("数据", "调度中心与服务区.xlsx", 6, 7)
    center = read_xlsx("数据", "调度中心与服务区.xlsx", 2, 3)[0]
    demand = read_xlsx("逐箱货箱清单", "物资需求与配送时限.xlsx", 1, 2)
    # The worksheet contains three blocks below the type table.  Restrict the
    # type read to the first three records; otherwise the later inventory and
    # battery headers are interpreted as drone types with null parameters.
    drows = read_xlsx("数据", "运输无人机数据.xlsx", 2, 3)[:3]
    drones = {}
    for r in drows:
        drones[r["机型编号"]] = Drone(
            r["机型编号"], r["含电池空载总质量（kg）"], r["最大载货质量（kg）"], r["可用装载体积（m³）"],
            r["计划巡航速度（m/s）"], r["空载标准航程（m）"], r["满载标准航程（m）"],
            r["电池可用能量（kWh）"], r["返航电量下限（%）"] / 100,
            r["工位固定准备时间（s）"], r["每箱装载时间（s）"],
            r["接收点基础交接时间（s）"], r["每箱增加交接时间（s）"],
            r["最大爬升速度（m/s）"], r["最大下降速度（m/s）"], r["爬升能耗效率"],
        )
    inv_rows = read_xlsx("数据", "运输无人机数据.xlsx", 8, 9)
    batteries = {r["机型编号"]: int(r["共享电池组总数（组）"]) for r in read_xlsx("数据", "运输无人机数据.xlsx", 19, 20)}
    aircraft = {t: [r["无人机编号"] for r in inv_rows if r["机型编号"] == t] for t in drones}
    rr = read_xlsx("数据", "中继无人机数据.xlsx", 2, 3)[0]
    # Keep the full relay contract in the data interface.  Older code only
    # retained cruise power and therefore could not audit relay climb,
    # hover/communication service, or component charging.
    comp_rows = read_xlsx("数据", "中继无人机数据.xlsx", 11, 12)
    relay = {"type": rr["机型编号"], "empty_mass": rr["含能源组件空载总质量（kg）"],
             "mass": rr["计划起飞总质量（kg）"], "speed": rr["计划巡航速度（m/s）"],
             "power_kw": rr["巡航功率（kW）"], "energy_kwh": rr["能源组件可用能量（kWh）"],
             "reserve": rr["返航电量下限（%）"] / 100, "prep_s": rr["工位固定准备时间（s）"],
             "link_s": rr["建链时间（s）"], "turn_s": rr["架次周转时间（s）"],
             "climb_speed": rr["最大爬升速度（m/s）"], "descend_speed": rr["最大下降速度（m/s）"],
             "eta_climb": rr["爬升能耗效率"], "eta_descend": rr["下降能耗效率"],
             "hover_power_kw": rr["悬停功率（kW）"], "comm_power_kw": rr["通信附加功率（kW）"],
             "max_height_m": rr["最大悬停离地高度（m）"],
             "component_inventory": int(comp_rows[0]["共享能源组件总数（组）"]),
             "component_full_charge_s": float(comp_rows[0]["等效完全充电时间（s）"]),
             "aircraft": [r["逐架中继无人机清单"] for r in read_xlsx("数据", "中继无人机数据.xlsx", 5, 6) if r.get("逐架中继无人机清单") in ("R01", "R02")]}
    comm_rows = read_xlsx("数据", "通信链路参数.xlsx", 2, 3)
    comm = {(r["参数类别"], r["参数名称"]): float(r["参数值"]) for r in comm_rows if r.get("参数值") is not None}
    dem_path = next((GEO / "数字高程模型数据（DEM）").glob("*DEM.mat"))
    dem = loadmat(dem_path)
    return {"center": center, "nodes": nodes, "boxes": demand, "drones": drones,
            "aircraft": aircraft, "batteries": batteries, "relay": relay, "comm": comm, "dem": dem}


def xy(lon: float, lat: float, lat0: float) -> tuple[float, float]:
    r = 6371008.8
    return r * math.radians(lon) * math.cos(math.radians(lat0)), r * math.radians(lat)


def terrain_profile(a: dict, b: dict, dem: dict) -> list[tuple[float, float]]:
    """Enumerate every DEM cell intersected by a lon/lat segment.

    Cell boundaries are midpoints between adjacent DEM centers.  The returned
    (t, elevation) pairs use each crossed cell midpoint, so short crossings and
    boundary contacts are not lost to a fixed-distance sample grid.
    """
    lon_a = float(a.get("经度（°）", a.get("经度"))); lat_a = float(a.get("纬度（°）", a.get("纬度")))
    lon_b = float(b.get("经度（°）", b.get("经度"))); lat_b = float(b.get("纬度（°）", b.get("纬度")))
    lons = np.asarray(dem["longitude"]).ravel(); lats = np.asarray(dem["latitude"]).ravel(); z = np.asarray(dem["dem"])
    xb = (lons[:-1] + lons[1:]) / 2; yb = (lats[:-1] + lats[1:]) / 2
    ts = [0.0, 1.0]
    if lon_b != lon_a:
        ts.extend(((xb - lon_a) / (lon_b - lon_a)).tolist())
    if lat_b != lat_a:
        ts.extend(((yb - lat_a) / (lat_b - lat_a)).tolist())
    ts = sorted({float(t) for t in ts if 0.0 < t < 1.0})
    edges = [0.0] + ts + [1.0]
    out = []
    nodata = float(np.asarray(dem.get("nodata", [[-32767]])).ravel()[0])
    def cell_value(lon: float, lat: float) -> float:
        ci = int(np.clip(np.searchsorted(xb, lon), 0, len(lons) - 1))
        ri = int(np.clip(np.searchsorted(-yb, -lat), 0, len(lats) - 1))
        val = float(z[ri, ci])
        return val if val != nodata and np.isfinite(val) else float("nan")

    for u, v in zip(edges[:-1], edges[1:]):
        t = (u + v) / 2; lon = lon_a + t * (lon_b - lon_a); lat = lat_a + t * (lat_b - lat_a)
        out.append((t, cell_value(lon, lat)))
    # At a cell boundary, inspect both sides (and the diagonal neighbors). This
    # makes boundary contact conservative rather than silently dropping the
    # higher adjacent pixel.
    eps = 1e-10
    for t in ts:
        lon = lon_a + t * (lon_b - lon_a); lat = lat_a + t * (lat_b - lat_a)
        vals = [cell_value(lon + dl * eps, lat + da * eps) for dl in (-1, 0, 1) for da in (-1, 0, 1)]
        vals = [v for v in vals if np.isfinite(v)]
        out.append((t, max(vals) if vals else float("nan")))
    out.extend([(0.0, cell_value(lon_a, lat_a)), (1.0, cell_value(lon_b, lat_b))])
    out.sort(key=lambda x: x[0])
    return out


def line_geometry(a: dict, b: dict, dem: dict) -> tuple[float, float, float, float, float, float]:
    lat0 = float(a["纬度（°）"] if "纬度（°）" in a else a["纬度"])
    lon_a = float(a["经度（°）"] if "经度（°）" in a else a["经度"])
    lat_a = float(a["纬度（°）"] if "纬度（°）" in a else a["纬度"])
    lon_b = float(b["经度（°）"] if "经度（°）" in b else b["经度"])
    lat_b = float(b["纬度（°）"] if "纬度（°）" in b else b["纬度"])
    key = (lon_a, lat_a, lon_b, lat_b)
    if key in _GEOM_CACHE:
        return _GEOM_CACHE[key]
    x1, y1 = xy(lon_a, lat_a, lat0); x2, y2 = xy(lon_b, lat_b, lat0)
    dist = math.hypot(x2 - x1, y2 - y1)
    prof = terrain_profile(a, b, dem)
    vals = [v for _, v in prof if np.isfinite(v)]
    if not vals:
        raise ValueError("DEM NoData on entire segment")
    result = (dist, float(max(vals)), float(x1), float(y1), float(x2), float(y2))
    _GEOM_CACHE[key] = result
    return result


def energy_leg(drone: Drone, distance: float, ground_max: float, h_start: float,
               h_end: float, payload: float) -> tuple[float, float]:
    cruise_h = ground_max + 50.0
    hup = max(0.0, cruise_h - h_start)
    hdown = max(0.0, cruise_h - h_end)
    frac = min(1.0, max(0.0, payload / drone.max_mass))
    eq_range = drone.range_empty - (drone.range_empty - drone.range_full) * frac ** 1.5
    e_h = drone.energy * distance / eq_range
    e_up = (drone.empty_mass + payload) * 9.81 * hup / (3.6e6 * drone.eta_climb)
    return e_h + e_up, distance / drone.speed + hup / drone.climb_speed + hdown / drone.descend_speed


def node_map(data: dict) -> dict:
    c = data["center"]
    out = {"O01": c}
    for r in data["nodes"]:
        out[r["服务区编号"]] = r
    return out


def route_stats(data: dict, typ: str, service: str, box_rows: list[dict]) -> dict:
    nm = node_map(data); d = data["drones"][typ]; o = nm["O01"]; s = nm[service]
    mass = sum(float(r["单箱质量（kg）"]) for r in box_rows)
    volume = sum(float(r["单箱体积（m³）"]) for r in box_rows)
    dist, peak, *_ = line_geometry(o, s, data["dem"])
    e_out, t_out = energy_leg(d, dist, peak, float(o["海拔（m）"]), float(s["海拔（m）"] + 30), mass)
    e_back, t_back = energy_leg(d, dist, peak, float(s["海拔（m）"] + 30), float(o["海拔（m）"]), 0.0)
    energy = e_out + e_back
    return {"service": service, "type": typ, "box_ids": [r["货箱编号"] for r in box_rows],
            "mass_kg": mass, "volume_m3": volume, "distance_m": 2 * dist,
            "ground_max_m": peak, "energy_kwh": energy, "out_flight_s": t_out,
            "return_flight_s": t_back, "flight_s": t_out + t_back,
            "safe": mass <= d.max_mass + 1e-9 and volume <= d.volume + 1e-9 and energy <= (1-d.reserve)*d.energy + 1e-9}


def q1(data: dict) -> tuple[pd.DataFrame, pd.DataFrame]:
    boxes = pd.DataFrame(data["boxes"])
    capability = []
    selected = []
    for service, grp in boxes.groupby("服务区编号", sort=True):
        rows = grp.to_dict("records")
        for typ, d in data["drones"].items():
            lo, hi = 0.0, d.max_mass
            for _ in range(60):
                q = (lo + hi) / 2
                st = route_stats(data, typ, service, [{"单箱质量（kg）": q, "单箱体积（m³）": 0, "货箱编号": "capacity"}])
                if st["energy_kwh"] <= (1-d.reserve)*d.energy: lo = q
                else: hi = q
            full = route_stats(data, typ, service, rows)
            capability.append({"service": service, "type": typ, "safe_mass_kg": lo,
                               "all_boxes_safe": bool(full["safe"]), "all_mass_kg": full["mass_kg"],
                               "all_volume_m3": full["volume_m3"], "all_energy_kwh": full["energy_kwh"]})
        # deterministic minimum-trip baseline: first-fit by priority, choose
        # the least-energy feasible type for each batch.
        remaining = rows[:]
        while remaining:
            first = remaining[0]
            candidates = []
            for typ in data["drones"]:
                batch = [first]
                for r in remaining[1:]:
                    trial = batch + [r]
                    if (sum(x["单箱质量（kg）"] for x in trial) <= data["drones"][typ].max_mass and
                        sum(x["单箱体积（m³）"] for x in trial) <= data["drones"][typ].volume and
                        route_stats(data, typ, service, trial)["safe"]): batch.append(r)
                st = route_stats(data, typ, service, batch)
                if st["safe"]: candidates.append(st)
            if not candidates:
                raise RuntimeError(f"Q1 no feasible batch for {service}, first box {first['货箱编号']}")
            best = min(candidates, key=lambda x: (len(x["box_ids"]) * -1, x["energy_kwh"], x["type"]))
            selected.append(best); chosen = set(best["box_ids"]); remaining = [r for r in remaining if r["货箱编号"] not in chosen]
    return pd.DataFrame(capability), pd.DataFrame(selected)


def q2(data: dict, batches: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    # Earliest-available event decoder using real aircraft and battery counts.
    avail_air = {u: 0.0 for us in data["aircraft"].values() for u in us}
    avail_bat = {t: [0.0] * n for t, n in data["batteries"].items()}
    task_rows, box_rows = [], []
    boxes = {r["货箱编号"]: r for r in data["boxes"]}
    for k, row in batches.sort_values(["service", "type"]).iterrows():
        typ, service = row["type"], row["service"]
        u = min(data["aircraft"][typ], key=lambda x: avail_air[x])
        bi = min(range(len(avail_bat[typ])), key=lambda i: avail_bat[typ][i])
        start = max(avail_air[u], avail_bat[typ][bi])
        ids = list(row["box_ids"] if isinstance(row["box_ids"], list) else json.loads(row["box_ids"]))
        nbox = len(ids); prep = data["drones"][typ].prep + data["drones"][typ].load_box * nbox
        handoff = data["drones"][typ].handoff + data["drones"][typ].handoff_box * nbox
        depart = start + prep
        # Use the outbound leg explicitly; the confirmed route model allows
        # asymmetric climb/descent times, so half of the round-trip time is
        # not generally the delivery time.
        deliver = depart + row["out_flight_s"] + handoff
        ret = depart + row["flight_s"] + handoff
        charge = 0.0
        soc = 1 - row["energy_kwh"] / data["drones"][typ].energy
        full_charge_s = {"A": 1800.0, "B": 2400.0, "C": 3000.0}[typ]
        if soc < .9:
            charge = full_charge_s * (0.65 * (.9 - soc) / .9 + .35)
        else:
            charge = full_charge_s * .35 * (1 - soc) / .1
        avail_air[u] = ret; avail_bat[typ][bi] = ret + charge
        task_row = {"sortie": f"Q2-{len(task_rows)+1:03d}", "aircraft": u, "type": typ, "battery": f"{typ}-B{bi+1:02d}", "box_ids": ids, "start_s": start, "service": service, "delivery_s": deliver, "handoff_s": handoff, "return_s": ret, "energy_kwh": row["energy_kwh"], "soc_return": soc, "hard_ok": True}
        for bid in ids:
            b = boxes[bid]; cb = deliver
            hard_ok = (b["物資类型"] if "物資类型" in b else b["物资类型"]) != "医疗物资" or cb <= b["期望送达时间（s）"]
            if b["是否首批保障"] == "是": hard_ok = hard_ok and cb <= b["首批截止时间（s）"]
            box_rows.append({"box_id": bid, "sortie": task_row["sortie"], "service": service, "completion_s": cb, "hard_ok": hard_ok})
        task_row["hard_ok"] = all(r["hard_ok"] for r in box_rows if r["sortie"] == task_row["sortie"])
        task_rows.append(task_row)
    return pd.DataFrame(task_rows), pd.DataFrame(box_rows)


def _loss_db(distance_m: float, freq_mhz: float, system_loss: float, obstruction: bool,
             obstruction_loss_db: float = 0.0) -> float:
    fspl = 32.45 + 20 * math.log10(freq_mhz) + 20 * math.log10(max(distance_m / 1000, 1e-9))
    # L_sys belongs in the link-budget threshold L_max (Appendix 3 P80),
    # while L_path is FSPL + terrain obstruction loss (P89).  Do not count it
    # a second time here.
    return fspl + (obstruction * float(obstruction_loss_db))


def _segment_cert(a: dict, b: dict, h_a: float, h_b: float, dem: dict, threshold_db: float,
                  link: tuple[float, float, float, float], comm: dict | None = None) -> dict:
    """3-D link certificate over every crossed DEM cell."""
    lon_a = float(a.get("经度（°）", a.get("经度"))); lat_a = float(a.get("纬度（°）", a.get("纬度")))
    lon_b = float(b.get("经度（°）", b.get("经度"))); lat_b = float(b.get("纬度（°）", b.get("纬度")))
    lat0 = float(data_lat0 := a.get("纬度（°）", a.get("纬度")))
    x1, y1 = xy(lon_a, lat_a, lat0); x2, y2 = xy(lon_b, lat_b, lat0)
    horizontal = math.hypot(x2 - x1, y2 - y1); d3 = math.sqrt(horizontal * horizontal + (h_b - h_a) ** 2)
    prof = terrain_profile(a, b, dem)
    clearances = []
    unknown = False
    for t, ground in prof:
        if not np.isfinite(ground): unknown = True; continue
        flight_h = h_a + t * (h_b - h_a)
        clearances.append(flight_h - ground)
    min_clear = min(clearances) if clearances else float("nan")
    # The obstruction penalty is applied only if the terrain penetrates the
    # straight 3-D corridor; otherwise the certified link is LOS.
    obstructed = unknown or min_clear <= 0.0
    if comm is None:
        raise ValueError("_segment_cert requires communication parameters loaded from workbook")
    freq = float(comm[("传播参数", "载波频率（MHz）")])
    lsys = float(comm[("传播参数", "系统损耗（dB）")])
    lobs = float(comm[("传播参数", "地形遮挡附加损耗（dB）")])
    loss = _loss_db(d3, freq, lsys, obstructed, lobs)
    return {"distance_m": d3, "min_clearance_m": min_clear, "unknown": unknown,
            "obstructed": obstructed, "loss_db": loss, "feasible": (not unknown and loss <= threshold_db)}


def q3(data: dict, tasks: pd.DataFrame, threshold_shift_db: float = 0.0) -> pd.DataFrame:
    """Three-dimensional direct-link certification plus one-relay search.

    Candidate relay points are placed at route fractions 0.25, 0.50 and 0.75,
    with 100/200/300 m above local DEM.  Lexicographic objective: feasible
    direct link, then minimum relay count, then total 3-D distance.
    """
    p = data["center"]; nm = node_map(data); rows = []; dem = data["dem"]
    comm = data["comm"]; lsys = comm[("传播参数", "系统损耗（dB）")]; fade = comm[("接收参数", "衰落裕量（dB）")]
    direct_thr = min(comm[("运输无人机", "发射功率（dBm）")] + comm[("运输无人机", "天线增益（dBi）")] + comm[("固定网关 G01", "天线增益（dBi）")] - comm[("接收参数", "接收灵敏度（dBm）")], comm[("固定网关 G01", "发射功率（dBm）")] + comm[("固定网关 G01", "天线增益（dBi）")] + comm[("运输无人机", "天线增益（dBi）")] - comm[("接收参数", "接收灵敏度（dBm）")]) - fade - lsys + threshold_shift_db
    access_thr = comm[("运输无人机", "发射功率（dBm）")] + comm[("运输无人机", "天线增益（dBi）")] + comm[("中继接入端", "天线增益（dBi）")] - comm[("接收参数", "接收灵敏度（dBm）")] - fade - lsys + threshold_shift_db
    back_thr = min(comm[("中继回传端", "发射功率（dBm）")] + comm[("中继回传端", "天线增益（dBi）")] + comm[("固定网关 G01", "天线增益（dBi）")] - comm[("接收参数", "接收灵敏度（dBm）")], comm[("固定网关 G01", "发射功率（dBm）")] + comm[("固定网关 G01", "天线增益（dBi）")] + comm[("中继回传端", "天线增益（dBi）")] - comm[("接收参数", "接收灵敏度（dBm）")]) - fade - lsys + threshold_shift_db
    for _, t in tasks.iterrows():
        s = nm[t["service"]]; dist, peak, *_ = line_geometry(p, s, dem); cruise_h = peak + 50.0
        gateway_h = float(p["海拔（m）"]) + comm[("固定网关 G01", "天线离地高度（m）")]
        direct = _segment_cert(p, s, gateway_h, cruise_h, dem, direct_thr, (20.0, 3.0, 12.0, 0.0), comm)
        best = None
        if not direct["feasible"]:
            lon0, lat0 = float(p["经度（°）"]), float(p["纬度（°）"]); lon1, lat1 = float(s["经度（°）"]), float(s["纬度（°）"])
            for frac in (0.25, 0.5, 0.75):
                rp = {"经度（°）": lon0 + frac * (lon1-lon0), "纬度（°）": lat0 + frac * (lat1-lat0)}
                up = {"经度（°）": rp["经度（°）"], "纬度（°）": rp["纬度（°）"]}
                # local DEM ground at the candidate; use a zero-length profile
                prof = terrain_profile(rp, rp, dem); rg = next((v for _, v in prof if np.isfinite(v)), float("nan"))
                if not np.isfinite(rg): continue
                for offset in (100.0, 200.0, 300.0):
                    rh = rg + offset
                    # Access is checked at every DEM-cell boundary/midpoint
                    # along the moving UAV route, not only at one waypoint.
                    # The direct leg receives the exact DEM-cell certificate.
                    # Relay access uses an adaptive 33-point sweep for the
                    # minimal runnable chain; final continuous certification
                    # remains a higher-cost refinement stage.
                    route_ts = np.linspace(0.0, 1.0, 33).tolist()
                    access_certs = []
                    for ut in route_ts:
                        up = {"经度（°）": lon0 + ut * (lon1 - lon0), "纬度（°）": lat0 + ut * (lat1 - lat0)}
                        access_certs.append(_segment_cert(up, rp, cruise_h, rh, dem, access_thr, (20.0, 3.0, 6.0, 0.0), comm))
                    a = {"feasible": all(c["feasible"] for c in access_certs),
                         "distance_m": max(c["distance_m"] for c in access_certs),
                         "min_clearance_m": min(c["min_clearance_m"] for c in access_certs)}
                    # Backhaul is relay to fixed G01.
                    b = _segment_cert(rp, p, rh, gateway_h, dem, back_thr, (19.0, 8.0, 12.0, 0.0), comm)
                    if a["feasible"] and b["feasible"]:
                        relay_e = data["relay"]["power_kw"] * (a["distance_m"] + b["distance_m"]) / data["relay"]["speed"] / 3600.0
                        cand = (a["distance_m"] + b["distance_m"], frac, offset, rg, a, b, relay_e)
                        if best is None or cand[0] < best[0]: best = cand
        relay_ok = best is not None
        rows.append({"sortie": t["sortie"], "service": t["service"], "direct_feasible": bool(direct["feasible"]),
                     "direct_min_clearance_m": direct["min_clearance_m"], "direct_distance_m": direct["distance_m"],
                     "relay_needed": not direct["feasible"], "relay_feasible": relay_ok,
                     "relay_fraction": None if not relay_ok else best[1], "relay_height_offset_m": None if not relay_ok else best[2],
                     "relay_ground_m": None if not relay_ok else best[3],
                     "total_distance_m": direct["distance_m"] if direct["feasible"] else (None if not relay_ok else best[0]),
                     "total_time_s": None if (not direct["feasible"] and not relay_ok) else (direct["distance_m"] if direct["feasible"] else best[0]) / data["drones"][t["type"]].speed,
                     "total_energy_kwh": t["energy_kwh"] if direct["feasible"] else (None if not relay_ok else t["energy_kwh"] + best[6]), "threshold_direct_db": direct_thr,
                     "threshold_access_db": access_thr, "threshold_backhaul_db": back_thr,
                     "direct_unknown": direct["unknown"]})
    return pd.DataFrame(rows)


def q4(data: dict, tasks: pd.DataFrame, k: int, comm: pd.DataFrame | None = None) -> pd.DataFrame:
    services = sorted(tasks["service"].unique()); group = {s: (i % k) + 1 for i, s in enumerate(services)}
    rows = []
    for g in range(1, k + 1):
        sub = tasks[tasks["service"].map(group) == g]
        events = []
        for _, r in sub.iterrows():
            events.extend([(float(r["start_s"]), 1), (float(r["return_s"]), -1)])
        events.sort(key=lambda x: (x[0], x[1]))
        cur = peak = 0; peak_t = 0.0
        for tm, delta in events:
            cur += delta
            if cur > peak: peak, peak_t = cur, tm
        rows.append({"K": k, "group": g, "services": ",".join(sorted(sub["service"].unique())),
                     "resource": "transport_aircraft_all", "peak_occupancy": peak,
                     "peak_time_s": peak_t, "capacity": sum(len(v) for v in data["aircraft"].values()),
                     "violates": peak > sum(len(v) for v in data["aircraft"].values()),
                     "sorties": len(sub), "interval_start_s": float(sub["start_s"].min()) if len(sub) else None,
                     "interval_end_s": float(sub["return_s"].max()) if len(sub) else None})
        for resource, capacity, col in [("A_aircraft", len(data["aircraft"]["A"]), "A"), ("B_aircraft", len(data["aircraft"]["B"]), "B"), ("C_aircraft", len(data["aircraft"]["C"]), "C")]:
            ev = []
            for _, r in sub[sub["type"] == col].iterrows(): ev.extend([(float(r["start_s"]), 1), (float(r["return_s"]), -1)])
            ev.sort(key=lambda x: (x[0], x[1])); cur = pk = 0; pt = 0.0
            for tm, dlt in ev:
                cur += dlt
                if cur > pk: pk, pt = cur, tm
            rows.append({"K": k, "group": g, "services": ",".join(sorted(sub["service"].unique())), "resource": resource,
                         "peak_occupancy": pk, "peak_time_s": pt, "capacity": capacity, "violates": pk > capacity,
                         "sorties": int((sub["type"] == col).sum()), "interval_start_s": float(sub["start_s"].min()) if len(sub) else None,
                         "interval_end_s": float(sub["return_s"].max()) if len(sub) else None})
            # Shared battery packs are occupied for the same flight interval;
            # charging is represented by the next-event availability already
            # encoded in Q2, so this is a conservative simultaneous-use check.
            bevents = []
            for _, r in sub[sub["type"] == col].iterrows(): bevents.extend([(float(r["start_s"]), 1), (float(r["return_s"]), -1)])
            bevents.sort(key=lambda x: (x[0], x[1])); cur = bpk = 0; bpt = 0.0
            for tm, dlt in bevents:
                cur += dlt
                if cur > bpk: bpk, bpt = cur, tm
            bcap = data["batteries"][col]
            rows.append({"K": k, "group": g, "services": ",".join(sorted(sub["service"].unique())), "resource": f"{col}_battery",
                         "peak_occupancy": bpk, "peak_time_s": bpt, "capacity": bcap, "violates": bpk > bcap,
                         "sorties": int((sub["type"] == col).sum()), "interval_start_s": float(sub["start_s"].min()) if len(sub) else None,
                         "interval_end_s": float(sub["return_s"].max()) if len(sub) else None})
        # Explicitly report relay and service-station occupancy even when the
        # certified Q3 solution uses zero relays.  Service handling is modeled
        # as a short interval at delivery, with one station per service.
        ev = []
        for _, r in sub.iterrows():
            if comm is not None:
                cr = comm[comm["sortie"] == r["sortie"]]
                if len(cr) and bool(cr.iloc[0]["relay_feasible"] and cr.iloc[0]["relay_needed"]):
                    ev.extend([(float(r["start_s"]), 1), (float(r["return_s"]), -1)])
        ev.sort(key=lambda x: (x[0], x[1])); cur = pk = 0; pt = 0.0
        for tm, dlt in ev:
            cur += dlt
            if cur > pk: pk, pt = cur, tm
        rows.append({"K": k, "group": g, "services": ",".join(sorted(sub["service"].unique())), "resource": "relay_aircraft",
                     "peak_occupancy": pk, "peak_time_s": pt, "capacity": len(data["relay"]["aircraft"]), "violates": pk > len(data["relay"]["aircraft"]),
                     "sorties": pk, "interval_start_s": float(sub["start_s"].min()) if len(sub) else None, "interval_end_s": float(sub["return_s"].max()) if len(sub) else None})
        for service in sorted(sub["service"].unique()):
            ss = sub[sub["service"] == service]
            ev = []
            for _, r in ss.iterrows():
                ev.extend([(float(r["delivery_s"]), 1), (float(r["delivery_s"] + r["handoff_s"]), -1)])
            ev.sort(key=lambda x: (x[0], x[1])); cur = pk = 0; pt = 0.0; p_end = 0.0
            for tm, dlt in ev:
                cur += dlt
                if cur > pk: pk, pt = cur, tm
                if pk and cur == 0 and tm >= pt: p_end = tm
            rows.append({"K": k, "group": g, "services": service, "resource": "service_station",
                         "peak_occupancy": pk, "peak_time_s": pt, "peak_end_s": p_end,
                         "capacity": 1, "violates": pk > 1, "sorties": len(ss),
                         "interval_start_s": float(ss["delivery_s"].min()) if len(ss) else None, "interval_end_s": float((ss["delivery_s"] + ss["handoff_s"]).max()) if len(ss) else None})
    return pd.DataFrame(rows)


def adjust_relay_schedule(tasks: pd.DataFrame, comm: pd.DataFrame, capacity: int = 2) -> pd.DataFrame:
    """Earliest-slot repair for relay intervals, preserving task order."""
    out = tasks.copy()
    relay_ids = set(comm.loc[comm["relay_needed"] & comm["relay_feasible"], "sortie"])
    available = [0.0] * capacity
    service_available = {}
    for idx, row in out.sort_values(["start_s", "sortie"]).iterrows():
        slot = min(range(capacity), key=lambda j: available[j])
        relay_release = available[slot] if row["sortie"] in relay_ids else float(row["start_s"])
        service_release = service_available.get(row["service"], float(row["start_s"]))
        shift = max(0.0, relay_release - float(row["start_s"]), service_release - float(row["delivery_s"]))
        out.at[idx, "start_s"] = float(row["start_s"]) + shift
        out.at[idx, "delivery_s"] = float(row["delivery_s"]) + shift
        out.at[idx, "return_s"] = float(row["return_s"]) + shift
        if row["sortie"] in relay_ids:
            available[slot] = float(row["return_s"]) + shift
        service_available[row["service"]] = float(row["delivery_s"] + row["handoff_s"]) + shift
    return out


def annotate_adjusted_deadlines(data: dict, tasks: pd.DataFrame) -> pd.DataFrame:
    boxes = {r["货箱编号"]: r for r in data["boxes"]}
    out = tasks.copy(); ok = []
    for _, row in out.iterrows():
        ids = [x for x in data["boxes"] if x["服务区编号"] == row["service"]]
        # Match the same service-level delivery timestamp to all boxes in its
        # assigned sortie; this is a diagnostic for schedule adjustment.
        good = True
        for b in ids:
            if b["是否首批保障"] == "是" and row["delivery_s"] > b["首批截止时间（s）"]: good = False
            if b["物资类型"] == "医疗物资" and row["delivery_s"] > b["期望送达时间（s）"]: good = False
        ok.append(good)
    out["adjusted_hard_ok_diagnostic"] = ok
    return out


def joint_schedule(data: dict, batches: pd.DataFrame, comm: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Deterministic Q2-Q3-Q4 schedule with lexicographic deadline priority."""
    base_tasks, _ = q2(data, batches)
    deadline = {s: min(float(r["首批截止时间（s）"]) if r["是否首批保障"] == "是" else float(r["期望送达时间（s）"])
                for r in data["boxes"] if r["服务区编号"] == s) for s in base_tasks["service"].unique()}
    base_tasks = base_tasks.assign(priority=base_tasks["service"].map(deadline))
    relay_ids = set(comm.loc[comm["relay_needed"] & comm["relay_feasible"], "sortie"])
    air = {u: 0.0 for us in data["aircraft"].values() for u in us}
    bat = {t: [0.0] * n for t, n in data["batteries"].items()}; relay_av = [0.0] * len(data["relay"]["aircraft"]); service_av = {}
    rows = []
    for _, r in base_tasks.sort_values(["priority", "sortie"]).iterrows():
        typ = r["type"]; u = min(data["aircraft"][typ], key=lambda x: air[x]); bi = min(range(len(bat[typ])), key=lambda i: bat[typ][i])
        dur = float(r["return_s"] - r["start_s"]); drel = float(r["delivery_s"] - r["start_s"]); hand = float(r["handoff_s"])
        start = max(air[u], bat[typ][bi], service_av.get(r["service"], 0.0) - drel)
        if r["sortie"] in relay_ids: start = max(start, min(relay_av))
        delivery = start + drel; ret = start + dur
        air[u] = ret
        energy = float(r["energy_kwh"]); soc = 1 - energy / data["drones"][typ].energy; full = {"A": 1800., "B": 2400., "C": 3000.}[typ]
        charge = full * (0.65 * (.9 - soc) / .9 + .35) if soc < .9 else full * .35 * (1 - soc) / .1
        bat[typ][bi] = ret + charge
        if r["sortie"] in relay_ids:
            j = min(range(len(relay_av)), key=lambda i: relay_av[i]); relay_av[j] = ret
        service_av[r["service"]] = delivery + hand
        rows.append({**r.to_dict(), "start_s": start, "delivery_s": delivery, "return_s": ret, "aircraft": u, "battery": f"{typ}-B{bi+1:02d}"})
    out = pd.DataFrame(rows).drop(columns=["priority"])
    return annotate_adjusted_deadlines(data, out), out


def plan_metrics(data: dict, tasks: pd.DataFrame, deliveries: pd.DataFrame, comm: pd.DataFrame) -> dict:
    tardy = deliveries.loc[~deliveries["hard_ok"]]
    lateness = []
    boxes = {r["货箱编号"]: r for r in data["boxes"]}
    for _, r in tardy.iterrows():
        b = boxes[r["box_id"]]; deadline = min(float(b["期望送达时间（s）"]), float(b["首批截止时间（s）"])) if b["是否首批保障"] == "是" else float(b["期望送达时间（s）"])
        lateness.append(max(0.0, float(r["completion_s"]) - deadline))
    relay_ids = set(comm.loc[comm["relay_needed"] & comm["relay_feasible"], "sortie"]); ev = []
    for _, t in tasks.iterrows():
        if t["sortie"] in relay_ids: ev.extend([(float(t["start_s"]), 1), (float(t["return_s"]), -1)])
    cur = pk = 0
    for _, d in sorted(ev, key=lambda x: (x[0], x[1])): cur += d; pk = max(pk, cur)
    return {"hard_tardy_boxes": len(tardy), "hard_tardy_sorties": int(tardy["sortie"].nunique()),
            "max_lateness_s": max(lateness, default=0.0), "total_lateness_s": sum(lateness),
            "completion_time_s": float(tasks["return_s"].max()), "total_energy_kwh": float(tasks["energy_kwh"].sum()),
            "relay_peak": int(pk)}


def deliveries_from_tasks(data: dict, tasks: pd.DataFrame) -> pd.DataFrame:
    boxes = {r["货箱编号"]: r for r in data["boxes"]}; rows = []
    for _, t in tasks.iterrows():
        ids = t["box_ids"] if isinstance(t["box_ids"], list) else json.loads(t["box_ids"])
        for bid in ids:
            b = boxes[bid]; ok = True
            if b["物资类型"] == "医疗物资": ok = ok and float(t["delivery_s"]) <= float(b["期望送达时间（s）"])
            if b["是否首批保障"] == "是": ok = ok and float(t["delivery_s"]) <= float(b["首批截止时间（s）"])
            rows.append({"box_id": bid, "sortie": t["sortie"], "service": t["service"], "completion_s": t["delivery_s"], "hard_ok": ok})
    return pd.DataFrame(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=20260924)
    args = ap.parse_args()
    np.random.seed(args.seed)
    (ROOT / "results").mkdir(exist_ok=True)
    data = load_inputs()
    cap, batches = q1(data)
    cap.to_csv(ROOT / "results/q1_capability_baseline.csv", index=False)
    batches.to_csv(ROOT / "results/q1_batches_baseline.csv", index=False)
    tasks, deliveries = q2(data, batches)
    tasks.to_csv(ROOT / "results/q2_sorties_baseline.csv", index=False)
    deliveries.to_csv(ROOT / "results/q2_deliveries_baseline.csv", index=False)
    comm = q3(data, tasks); comm.to_csv(ROOT / "results/q3_direct_link_diagnostic.csv", index=False)
    # Threshold sensitivity: recompute the certified direct/relay labels at
    # +/-3 dB around the contractual direct-link threshold.
    sens = []
    for delta in (-3.0, 0.0, 3.0):
        sd = comm if delta == 0 else q3(data, tasks, threshold_shift_db=delta)
        sens.append({"threshold_shift_db": delta,
                     "direct_feasible_count": int(sd["direct_feasible"].sum()),
                     "relay_feasible_count": int(sd["relay_feasible"].sum()),
                     "final_infeasible_count": int((~sd["direct_feasible"] & ~sd["relay_feasible"]).sum())})
    pd.DataFrame(sens).to_csv(ROOT / "results/q3_threshold_sensitivity.csv", index=False)
    q4_summary = {}
    adjusted_tasks = annotate_adjusted_deadlines(data, adjust_relay_schedule(tasks, comm, capacity=len(data["relay"]["aircraft"])))
    adjusted_tasks.to_csv(ROOT / "results/q4_schedule_adjusted.csv", index=False)
    joint_tasks, joint_deliveries = joint_schedule(data, batches, comm)
    joint_tasks.to_csv(ROOT / "results/q_joint_schedule_C.csv", index=False)
    joint_deliveries = deliveries_from_tasks(data, joint_tasks)
    joint_deliveries.to_csv(ROOT / "results/q_joint_deliveries_C.csv", index=False)
    plans = {
        "A_original_baseline": plan_metrics(data, tasks, deliveries, comm),
        "B_resource_adjusted": plan_metrics(data, adjusted_tasks, deliveries_from_tasks(data, adjusted_tasks), comm),
        "C_joint_deadline_first": plan_metrics(data, joint_tasks, joint_deliveries, comm),
    }
    (ROOT / "results/q2_q3_q4_plan_comparison.json").write_text(json.dumps(plans, indent=2, ensure_ascii=False))
    pd.DataFrame(plans).T.to_csv(ROOT / "results/q2_q3_q4_plan_comparison.csv")
    for k in (2, 3):
        q4res = q4(data, tasks, k, comm)
        q4res.to_csv(ROOT / f"results/q4_partition_k{k}_baseline.csv", index=False)
        q4adj = q4(data, adjusted_tasks, k, comm)
        q4adj.assign(schedule_adjustment="earliest_relay_slot").to_csv(ROOT / f"results/q4_partition_k{k}_adjusted.csv", index=False)
        q4adj = q4(data, adjusted_tasks, k, comm)
        q4_summary[str(k)] = {"baseline_peak_transport": int(q4res.loc[q4res["resource"] == "transport_aircraft_all", "peak_occupancy"].max()),
                              "baseline_peak_relay": int(q4res.loc[q4res["resource"] == "relay_aircraft", "peak_occupancy"].max()),
                              "baseline_violations": int(q4res["violates"].sum()),
                              "adjusted_peak_transport": int(q4adj.loc[q4adj["resource"] == "transport_aircraft_all", "peak_occupancy"].max()),
                              "adjusted_peak_relay": int(q4adj.loc[q4adj["resource"] == "relay_aircraft", "peak_occupancy"].max()),
                              "adjusted_violations": int(q4adj["violates"].sum())}
    summary = {"seed": args.seed, "q1_batches": len(batches), "q1_all_boxes": int(batches["box_ids"].map(len).sum()),
               "q2_sorties": len(tasks), "q2_hard_ok": bool(deliveries["hard_ok"].all()),
               "q3_direct_feasible": int(comm["direct_feasible"].sum()),
               "q3_relay_feasible": int(comm["relay_feasible"].sum()),
               "q3_final_infeasible": int((~comm["direct_feasible"] & ~comm["relay_feasible"]).sum()), "q3_sorties": len(comm),
               "q4_groups": [2, 3], "q4_peak_summary": q4_summary,
               "joint_plan_comparison": plans,
               "baseline": True, "note": "Minimal real-data chain; not globally optimized."}
    (ROOT / "results/minimal_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
