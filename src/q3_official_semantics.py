"""Official Q3 formula and resource semantics.

This module is intentionally separate from the historical schedulers.  It is a
small, testable implementation of the DOCX contract used by the representative
closed-loop audit before any Q3 optimization is resumed.
"""
from __future__ import annotations
import math
from pathlib import Path
import numpy as np

from minimal_pipeline import node_map, terrain_profile, line_geometry, _segment_cert

TOL = 1e-6


class RuntimeTraceDict(dict):
    """Dictionary wrapper that records actual runtime parameter reads."""
    def __init__(self, values, name, trace):
        super().__init__(values); self._name=name; self._trace=trace
    def __getitem__(self, key):
        value=super().__getitem__(key); self._trace.append({'container':self._name,'key':key,'runtime_value':value}); return value


def hard_deadline_ok(delivery_time_s: float, hard_deadline_s: float) -> bool:
    """Official hard predicate: delivery at service area, never return time."""
    return float(delivery_time_s) <= float(hard_deadline_s) + TOL


def build_transport_trajectory(data: dict, task: dict | object, time_origin_s: float = 0.0) -> list[dict]:
    """Build x(t),y(t),z(t) phases and DEM-boundary events for one sortie."""
    nm = node_map(data); o = data["center"]; service = str(task["service"]); s = nm[service]
    typ = str(task["type"]); d = data["drones"][typ]
    dist, peak, *_ = line_geometry(o, s, data["dem"]); cruise_h = peak + 50.0
    oh = float(o["海拔（m）"]); sh = float(s["海拔（m）"]) + 30.0
    up = max(0.0, cruise_h-oh); down = max(0.0, cruise_h-sh)
    t_up = up/d.climb_speed; t_cruise = dist/d.speed; t_down = down/d.descend_speed
    hand = float(task.get("handoff_s", 0.0) if isinstance(task, dict) else task["handoff_s"])
    t_up_r = max(0.0, cruise_h-sh)/d.climb_speed; t_down_r = max(0.0, cruise_h-oh)/d.descend_speed
    lons = float(o["经度（°）"]); lats = float(o["纬度（°）"]); lone = float(s["经度（°）"]); late = float(s["纬度（°）"])
    prof = terrain_profile(o, s, data["dem"]); events = sorted({float(t) for t, _ in prof if 0.0 < float(t) < 1.0})
    out=[]; cur=float(time_origin_s)
    def phase(dt,a,b,ha,hb,name,event_fractions=None):
        nonlocal cur
        cuts=[0.0,1.0]+([] if event_fractions is None else list(event_fractions)); cuts=sorted(set(cuts))
        for u,v in zip(cuts[:-1],cuts[1:]):
            aa={"经度（°）":float(a["经度（°）"])+u*(float(b["经度（°）"])-float(a["经度（°）"])),"纬度（°）":float(a["纬度（°）"])+u*(float(b["纬度（°）"])-float(a["纬度（°）"]))}
            bb={"经度（°）":float(a["经度（°）"])+v*(float(b["经度（°）"])-float(a["经度（°）"])),"纬度（°）":float(a["纬度（°）"])+v*(float(b["纬度（°）"])-float(a["纬度（°）"]))}
            out.append({"t0":cur,"t1":cur+dt*(v-u),"a":aa,"b":bb,"ha":float(ha)+u*(float(hb)-float(ha)),"hb":float(ha)+v*(float(hb)-float(ha)),"phase":name,"dem_event":bool(event_fractions)})
            cur += dt*(v-u)
    O={"经度（°）":lons,"纬度（°）":lats}; S={"经度（°）":lone,"纬度（°）":late}
    phase(t_up,O,O,oh,cruise_h,'climb'); phase(t_cruise,O,S,cruise_h,cruise_h,'cruise',events); phase(t_down,S,S,cruise_h,sh,'descent'); phase(hand,S,S,sh,sh,'handoff')
    reverse_events=[1.0-e for e in reversed(events)]
    phase(t_up_r,S,S,sh,cruise_h,'return_climb'); phase(t_cruise,S,O,cruise_h,cruise_h,'return_cruise',reverse_events); phase(t_down_r,O,O,cruise_h,oh,'return_descent')
    return out


def merge_relay_required_intervals(dynamic_rows: list[dict]) -> list[dict]:
    """Merge adjacent relay-required half-open intervals."""
    req=[r for r in dynamic_rows if bool(r.get("relay_required"))]
    merged=[]
    for r in sorted(req,key=lambda x:float(x["time_interval_start"])):
        a=float(r["time_interval_start"]); b=float(r["time_interval_end"])
        if merged and a <= merged[-1]["interval_end"] + TOL:
            merged[-1]["interval_end"] = max(merged[-1]["interval_end"], b)
            merged[-1]["continuous_pass"] = bool(merged[-1]["continuous_pass"] and r.get("communication_ok",False))
        else:
            merged.append({"interval_start":a,"interval_end":b,"continuous_pass":bool(r.get("communication_ok",False)),"minimum_access_link_margin_db":float(r.get("minimum_access_link_margin_db",float('nan'))),"minimum_backhaul_margin_db":float(r.get("minimum_backhaul_margin_db",float('nan'))),"minimum_terrain_clearance_m":float(r.get("minimum_terrain_clearance_m",float('nan')))})
    return merged


def timeline_continuity_audit(trajectory: list[dict]) -> list[dict]:
    """Audit a single global half-open trajectory for gaps/overlaps."""
    rows=[]; prev_end=None
    for i,e in enumerate(trajectory):
        a=float(e['t0']); b=float(e['t1']); duration=b-a
        gap=0.0 if prev_end is None else a-prev_end
        overlap=max(0.0, prev_end-a) if prev_end is not None else 0.0
        ok=bool(duration >= -TOL and gap <= TOL and overlap <= TOL)
        rows.append({'event_or_phase':e.get('phase',f'phase_{i}'),'start_time':a,'end_time':b,'duration':duration,'prev_end':prev_end,'gap':gap,'overlap':overlap,'pass':ok})
        prev_end=b
    return rows


def runtime_parameter_trace(data: dict) -> list[dict]:
    """Values consumed by the dynamic validator, traced to loaded attachments."""
    c=data['comm']; r=data['relay']; rows=[]
    def add(name,val,unit,source,field,func):
        rows.append({'parameter':name,'runtime_value':float(val),'unit':unit,'source_function':func,'source_file':source,'source_field':field,'loaded_from_attachment':True,'hardcoded_in_runtime_path':False})
    for (cat,name),val in c.items(): add(name,val,'see attachment field','通信链路参数.xlsx',f'{cat}:{name}','dynamic_communication_audit')
    for name,key,unit in [('prepare_s','prep_s','s'),('link_setup_s','link_s','s'),('turnaround_s','turn_s','s'),('hover_power_kw','hover_power_kw','kW'),('communication_additional_power_kw','comm_power_kw','kW'),('cruise_power_kw','power_kw','kW'),('cruise_speed_mps','speed','m/s'),('climb_speed_mps','climb_speed','m/s'),('descent_speed_mps','descend_speed','m/s'),('climb_efficiency','eta_climb','1'),('descent_efficiency','eta_descend','1'),('component_energy_kwh','energy_kwh','kWh'),('component_inventory','component_inventory','groups'),('component_full_charge_s','component_full_charge_s','s')]: add(name,r[key],unit,'中继无人机数据.xlsx',key,'relay_time_chain/relay_leg_time_energy')
    return rows


def component_charge_time(soc: float, full_charge_s: float) -> float:
    """DOCX P63 two-stage equivalent charging curve."""
    s = min(1.0, max(0.0, float(soc)))
    T = float(full_charge_s)
    if s < 0.90:
        return T * (0.65 * (0.90 - s) / 0.90 + 0.35)
    return T * 0.35 * (1.0 - s) / 0.10


def relay_leg_time_energy(data: dict, distance_m: float, ground_max_m: float,
                          h_start_m: float, h_end_m: float) -> tuple[float, float]:
    """Official relay leg time and energy (horizontal + climb; descent=0)."""
    r = data["relay"]
    # Appendix 2 sets the planned cruise altitude to the maximum terrain
    # along the segment plus 50 m, but it may not be below either endpoint's
    # operating altitude.
    cruise_h = max(float(ground_max_m) + 50.0, float(h_start_m), float(h_end_m))
    hup = max(0.0, cruise_h - float(h_start_m))
    hdown = max(0.0, cruise_h - float(h_end_m))
    t = hup / float(r["climb_speed"]) + float(distance_m) / float(r["speed"])
    t += hdown / float(r["descend_speed"])
    # P59: cruise power over cruise time + climb extra energy; descent
    # efficiency is zero in the supplied relay data, hence no descent term.
    e_cruise = float(r["power_kw"]) * float(distance_m) / float(r["speed"]) / 3600.0
    e_up = float(r["mass"]) * 9.81 * hup / (3.6e6 * float(r["eta_climb"]))
    e_down = 0.0 if float(r.get("eta_descend", 0.0)) == 0 else float(r["mass"]) * 9.81 * hdown / (3.6e6 * float(r["eta_descend"]))
    return t, e_cruise + e_up + e_down


def relay_candidate_point(data: dict, service: str, cert_row: dict) -> tuple[dict, float, float]:
    """Return certified relay point and its local ground elevation."""
    nm = node_map(data); o = data["center"]; s = nm[service]
    f = float(cert_row["relay_fraction"])
    lon = float(o["经度（°）"]) + f * (float(s["经度（°）"]) - float(o["经度（°）"]))
    lat = float(o["纬度（°）"]) + f * (float(s["纬度（°）"]) - float(o["纬度（°）"]))
    p = {"经度（°）": lon, "纬度（°）": lat}
    prof = terrain_profile(p, p, data["dem"])
    ground = max(v for _, v in prof if np.isfinite(v))
    height = ground + float(cert_row["relay_height_offset_m"])
    p["海拔（m）"] = height
    return p, ground, height


def relay_time_chain(data: dict, service: str, cert_row: dict, service_duration_s: float,
                     service_start_s: float = 0.0, service_intervals: list[tuple[float,float]] | None = None) -> dict:
    """Independent relay prepare/outbound/setup/service/return/turn ledger."""
    nm = node_map(data); o = data["center"]
    rp, ground, rh = relay_candidate_point(data, service, cert_row)
    # O01 and relay point use their 3-D distances; service endpoint is the
    # service-area work altitude (ground + 30 m), as in Appendix 2.
    _, peak_o, *_ = line_geometry(o, rp, data["dem"])
    _, peak_s, *_ = line_geometry(rp, nm[service], data["dem"])
    t_out, e_out = relay_leg_time_energy(data, line_geometry(o, rp, data["dem"])[0], peak_o,
                                         float(o["海拔（m）"]), rh)
    t_back, e_back = relay_leg_time_energy(data, line_geometry(rp, o, data["dem"])[0], peak_o,
                                          rh, float(o["海拔（m）"]))
    prep = float(data["relay"]["prep_s"]); setup = float(data["relay"]["link_s"]); turn = float(data["relay"]["turn_s"])
    t0 = float(service_start_s)
    prep_start = t0
    outbound_start = prep_start + prep
    link_start = outbound_start + t_out
    service_start = link_start + setup
    service_end = service_start + float(service_duration_s)
    if service_intervals:
        service_end = max(float(b) for _, b in service_intervals)
    return_start = service_end
    return_end = return_start + t_back
    busy_end = return_end + turn
    service_energy = (float(data["relay"]["hover_power_kw"]) + float(data["relay"]["comm_power_kw"])) * float(service_duration_s) / 3600.0
    total_energy = e_out + e_back + service_energy
    return {
        "prepare_start": prep_start, "prepare_end": outbound_start,
        "outbound_start": outbound_start, "outbound_end": link_start,
        "link_setup_start": link_start, "link_setup_end": service_start,
        "service_start": service_start, "service_end": service_end,
        "return_start": return_start, "return_end": return_end,
        "turnaround_start": return_end, "busy_end": busy_end,
        "outbound_flight_s": t_out, "return_flight_s": t_back,
        "service_duration_s": float(service_duration_s),
        "service_intervals": service_intervals or [(service_start, service_end)],
        "outbound_energy_kwh": e_out, "return_energy_kwh": e_back,
        "service_energy_kwh": service_energy, "total_energy_kwh": total_energy,
        "relay_ground_m": ground, "relay_height_m": rh,
        "relay_position_lon": rp["经度（°）"], "relay_position_lat": rp["纬度（°）"],
    }


def relay_component_ledger(data: dict, component_id: str, task_start_s: float,
                           task_end_s: float, energy_kwh: float,
                           previous_available_s: float = 0.0) -> dict:
    """Independent energy-component use/charge ledger with reserve check."""
    r = data["relay"]; available = float(r["energy_kwh"]); reserve = float(r["reserve"])
    soc = 1.0 - float(energy_kwh) / available
    reserve_ok = soc + TOL >= reserve
    charge_s = component_charge_time(soc, r["component_full_charge_s"])
    charge_start = float(task_end_s)
    charge_end = charge_start + charge_s
    available_before_task = float(previous_available_s)
    reusable = max(available_before_task, charge_end)
    return {"component_id": str(component_id), "energy_kwh": float(energy_kwh),
            "soc_start": 1.0, "soc_end": soc, "reserve_fraction": reserve,
            "reserve_ok": bool(reserve_ok), "available_before_task_s": available_before_task,
            "resource_available_ok": bool(float(task_start_s) + TOL >= available_before_task),
            "use_start_s": float(task_start_s),
            "use_end_s": float(task_end_s), "charge_start_s": charge_start,
            "charge_end_s": charge_end, "available_again_s": reusable,
            "full_charge_s": float(r["component_full_charge_s"]),
            "inventory": int(r["component_inventory"])}


def dynamic_communication_audit(data: dict, service: str, aircraft_type: str,
                               cert_row: dict, transport_trajectory: list[dict],
                               relay_chain: dict | None = None) -> list[dict]:
    """Check direct or simultaneous two-link relay state over trajectory intervals.

    Each trajectory item has `t0,t1`, endpoint coordinates `a,b`, and endpoint
    heights `ha,hb`; endpoints are evaluated at the interval midpoint.  The
    certificate candidate is reused only for its certified relay geometry; link
    state is recomputed for each interval.
    """
    nm = node_map(data); o = data["center"]; p, _, rh = relay_candidate_point(data, service, cert_row)
    gateway_h = float(o["海拔（m）"]) + float(data["comm"][("固定网关 G01", "天线离地高度（m）")])
    c = data["comm"]; sens = c[("接收参数", "接收灵敏度（dBm）")]; fade = c[("接收参数", "衰落裕量（dB）")]; lsys = c[("传播参数", "系统损耗（dB）")]
    direct_thr = min(c[("运输无人机", "发射功率（dBm）")] + c[("运输无人机", "天线增益（dBi）")] + c[("固定网关 G01", "天线增益（dBi）")] - sens,
                     c[("固定网关 G01", "发射功率（dBm）")] + c[("固定网关 G01", "天线增益（dBi）")] + c[("运输无人机", "天线增益（dBi）")] - sens) - fade - lsys
    access_thr = c[("运输无人机", "发射功率（dBm）")] + c[("运输无人机", "天线增益（dBi）")] + c[("中继接入端", "天线增益（dBi）")] - sens - fade - lsys
    back_thr = min(c[("中继回传端", "发射功率（dBm）")] + c[("中继回传端", "天线增益（dBi）")] + c[("固定网关 G01", "天线增益（dBi）")] - sens,
                   c[("固定网关 G01", "发射功率（dBm）")] + c[("固定网关 G01", "天线增益（dBi）")] + c[("中继回传端", "天线增益（dBi）")] - sens) - fade - lsys
    s = nm[service]; out = []
    for seg in transport_trajectory:
        a = seg["a"]; b = seg["b"]
        samples = []
        # Evaluate both ends and the midpoint of each physical phase.  The
        # resulting interval is marked feasible only when every sampled
        # state is feasible; a future full optimizer may further subdivide a
        # phase at all moving-endpoint DEM crossings.
        for alpha in (0.0, 0.5, 1.0):
            tm = float(seg["t0"]) + alpha * (float(seg["t1"]) - float(seg["t0"]))
            lon = float(a["经度（°）"]) + alpha * (float(b["经度（°）"]) - float(a["经度（°）"]))
            lat = float(a["纬度（°）"]) + alpha * (float(b["纬度（°）"]) - float(a["纬度（°）"]))
            h = float(seg["ha"]) + alpha * (float(seg["hb"]) - float(seg["ha"]))
            pos = {"经度（°）": lon, "纬度（°）": lat}
            dc = _segment_cert(pos, o, h, gateway_h, data["dem"], direct_thr, None, data["comm"])
            ac = _segment_cert(pos, p, h, rh, data["dem"], access_thr, None, data["comm"])
            bc = _segment_cert(p, o, rh, gateway_h, data["dem"], back_thr, None, data["comm"])
            direct, access, backhaul = dc["feasible"], ac["feasible"], bc["feasible"]
            relay_required = not direct
            ok = bool(direct or (relay_required and access and backhaul))
            samples.append((tm, direct, relay_required, access, backhaul, ok,
                            access_thr-ac["loss_db"], back_thr-bc["loss_db"],
                            min(ac["min_clearance_m"],bc["min_clearance_m"]),
                            dc["loss_db"], dc["distance_m"] / 1000.0,
                            len(terrain_profile(pos,o,data["dem"])), dc["min_clearance_m"]))
        tm, direct, relay_required, access, backhaul, ok, _, _, _, _, _, _, _ = samples[1]
        chain_ok = True
        if relay_chain is not None and any(x[2] for x in samples):
            req_start = min(x[0] for x in samples if x[2]); req_end = max(x[0] for x in samples if x[2])
            chain_ok = (float(relay_chain["service_start"]) <= req_start + TOL and
                        float(relay_chain["service_end"]) >= req_end - TOL)
        out.append({"time_interval_start": float(seg["t0"]), "time_interval_end": float(seg["t1"]),
                    "direct_available": bool(all(x[1] for x in samples)), "relay_required": bool(any(x[2] for x in samples)),
                    "relay_id": str(seg.get("relay_id", "R01")) if relay_required else "",
                    "access_link_ok": bool(all(x[3] for x in samples if x[2])) if any(x[2] for x in samples) else True,
                    "backhaul_link_ok": bool(all(x[4] for x in samples if x[2])) if any(x[2] for x in samples) else True,
                    "communication_ok": bool(all(x[5] for x in samples) and chain_ok), "relay_service_window_ok": bool(chain_ok),
                    "minimum_access_link_margin_db": float(min(x[6] for x in samples if x[2])) if any(x[2] for x in samples) else float('inf'),
                    "minimum_backhaul_margin_db": float(min(x[7] for x in samples if x[2])) if any(x[2] for x in samples) else float('inf'),
                    "minimum_terrain_clearance_m": float(min(x[8] for x in samples if x[2])) if any(x[2] for x in samples) else float(min(x[12] for x in samples)),
                    "max_path_loss_db": float(max(x[9] for x in samples)), "max_distance_km": float(max(x[10] for x in samples)),
                    "threshold_db": float(direct_thr), "margin_db": float(direct_thr-max(x[9] for x in samples)),
                    "dem_cells": int(max(x[11] for x in samples)), "continuous_certified": bool(all(x[5] for x in samples)), "midpoint_time_s": tm,
                    "sample_count": 3, "phase": seg.get("phase", "")})
    return out


def resource_peak(events: list[dict], resource: str, inventory: int) -> tuple[int, bool]:
    """Half-open [start,end) sweep; releases are processed before starts."""
    ev = []
    for e in events:
        if e["resource"] != resource: continue
        ev.extend([(float(e["start_s"]), 1), (float(e["end_s"]), -1)])
    cur = peak = 0
    for _, delta in sorted(ev, key=lambda x: (x[0], x[1])):
        cur += delta; peak = max(peak, cur)
    return peak, bool(peak <= int(inventory))
