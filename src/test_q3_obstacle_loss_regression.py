"""Regression tests for conditional DEM obstacle loss.

This test intentionally uses the audited S008 binding geometry and recomputes
the link budget from the loaded workbook/DEM values.  It does not use a route
optimizer or the old communication helper.
"""
from __future__ import annotations
import json, math
from pathlib import Path
import numpy as np
from minimal_pipeline import load_inputs, xy, terrain_profile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "q3_obstacle_loss_regression_tests.json"
R = 6371008.8

def xyz(data, lon, lat, h):
    return np.array([*xy(lon, lat, float(data["center"]["纬度（°）"])), float(h)], dtype=float)

def obstructed(data, a, b, ha, hb):
    for t, ground in terrain_profile(a, b, data["dem"]):
        if np.isfinite(ground) and float(ha) + float(t) * (float(hb) - float(ha)) - float(ground) <= 0:
            return True
    return False

def main():
    data = load_inputs()
    t_lon, t_lat, t_h = 109.21373972722884, 23.07561883018323, 372.9093322753906
    r_lon, r_lat, r_h = 109.22252777777778, 23.06485555555556, 347.8144226074219
    g_lon, g_lat, g_h = 109.2308517, 23.0085095, 147.7
    f = float(data["comm"][("传播参数", "载波频率（MHz）")])
    obs_loss = float(data["comm"][("传播参数", "地形遮挡附加损耗（dB）")])
    sens = float(data["comm"][("接收参数", "接收灵敏度（dBm）")])
    fade = float(data["comm"][("接收参数", "衰落裕量（dB）")])
    sys_loss = float(data["comm"][("传播参数", "系统损耗（dB）")])
    access_thr = 20 + 3 + 6 - sens - fade - sys_loss
    back_thr = 19 + 8 + 12 - sens - fade - sys_loss
    def budget(a, b, ha, hb, threshold):
        d = float(np.linalg.norm(xyz(data, *a, ha) - xyz(data, *b, hb)))
        fspl = 32.45 + 20 * math.log10(f) + 20 * math.log10(d / 1000.0)
        blocked = obstructed(data, {"经度（°）": a[0], "纬度（°）": a[1]}, {"经度（°）": b[0], "纬度（°）": b[1]}, ha, hb)
        applied = obs_loss if blocked else 0.0
        return {"obstructed": bool(blocked), "obstacle_loss_applied_db": applied,
                "fspl_db": fspl, "path_loss_db": fspl + applied,
                "threshold_db": threshold, "margin_db": threshold - fspl - applied}
    access = budget((t_lon, t_lat), (r_lon, r_lat), t_h, r_h, access_thr)
    backhaul = budget((r_lon, r_lat), (g_lon, g_lat), r_h, g_h, back_thr)
    result = {
        "test_unobstructed_access": {**access, "pass": (not access["obstructed"] and access["obstacle_loss_applied_db"] == 0 and abs(access["margin_db"] - 12.439370626081981) < 1e-6)},
        "test_obstructed_backhaul": {**backhaul, "pass": (backhaul["obstructed"] and abs(backhaul["obstacle_loss_applied_db"] - 10) < 1e-9 and abs(backhaul["margin_db"] + 0.07712024304419174) < 1e-6)},
    }
    result["all_pass"] = bool(result["test_unobstructed_access"]["pass"] and result["test_obstructed_backhaul"]["pass"])
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["all_pass"] else 1)

if __name__ == "__main__":
    main()
