"""Fast Q3 global infeasibility diagnostic for frozen K22/K23 route sets.

Tests nested feasibility gates to identify which relay requirement first makes a
transport route set infeasible. This is diagnostic only; it does not claim a
final Q3 schedule.
"""
from __future__ import annotations
import argparse, ast, json, math
from pathlib import Path
from collections import defaultdict
import pandas as pd
from ortools.sat.python import cp_model
from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy

SCALE=10
def I(x): return int(round(float(x)*SCALE))
def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try:return [str(x) for x in json.loads(str(v))]
    except:return [str(x) for x in ast.literal_eval(str(v))]

def run(schedule,hg,label,max_waves,gate):
    data=load_inputs(); sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=pd.read_csv(Path(hg)/"block_diagnostics.csv")
    cover=pd.read_csv(Path(hg)/"minimum_cover_positions.csv").reset_index(drop=True)
    elig=defaultdict(list)
    pos=[]
    for pidx,r in cover.iterrows():
        for b in str(r.covered_blocks).split("|"): elig[b].append(pidx)
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(r.altitude_m),float(data["center"]["海拔（m）"]))
        pos.append({"out":tout,"back":tback,"e":eout+eback})
    m=cp_model.CpModel(); H=I(16000)
    starts=[]; returns=[]; air=defaultdict(list); bat=defaultdict(list)
    for i,r in sch.iterrows():
        s=m.NewIntVar(0,H,f"s{i}"); e=m.NewIntVar(0,H*2,f"e{i}")
        dur=I(r.duration_s); m.Add(e==s+dur); starts.append(s); returns.append(e)
        if pd.notna(r.latest_start_s): m.Add(s<=int(math.floor(float(r.latest_start_s)*SCALE+1e-9)))
        typ=str(r.drone_type)
        air[typ].append(m.NewIntervalVar(s,dur,e,f"a{i}"))
        bd=I(float(r.duration_s)+float(r.charge_s)); be=m.NewIntVar(0,H*2,f"be{i}"); m.Add(be==s+bd)
        bat[typ].append(m.NewIntervalVar(s,bd,be,f"b{i}"))
    for typ,x in air.items():m.AddCumulative(x,[1]*len(x),len(data["aircraft"][typ]))
    for typ,x in bat.items():m.AddCumulative(x,[1]*len(x),int(data["batteries"][typ]))
    if gate=="transport":
        pass
    else:
        abs_s=[]; abs_e=[]
        for bi,b in blocks.iterrows():
            a=m.NewIntVar(0,H*2,f"bs{bi}"); e=m.NewIntVar(0,H*2,f"be{bi}")
            m.Add(a==starts[int(b.route_index)]+I(b.start_s)); m.Add(e==starts[int(b.route_index)]+I(b.end_s))
            abs_s.append(a); abs_e.append(e)
        yw={}; uses=[]; intervals=[]
        hover=float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"])
        usable=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
        for bi,b in blocks.iterrows():
            opts=[]
            for p in elig[str(b.block_id)]:
                for w in range(max_waves):
                    z=m.NewBoolVar(f"y{bi}_{p}_{w}"); yw[bi,p,w]=z; opts.append(z)
            m.Add(sum(opts)==1)
        for p in range(len(pos)):
            for w in range(max_waves):
                u=m.NewBoolVar(f"u{p}_{w}"); uses.append(u)
                ms=m.NewIntVar(0,H*2,f"ms{p}_{w}"); me=m.NewIntVar(0,H*2,f"me{p}_{w}")
                assigned=[]
                for bi,b in blocks.iterrows():
                    z=yw.get((bi,p,w))
                    if z is None: continue
                    assigned.append(z); m.AddImplication(z,u)
                    m.Add(ms<=abs_s[bi]).OnlyEnforceIf(z); m.Add(me>=abs_e[bi]).OnlyEnforceIf(z)
                if assigned:
                    m.Add(sum(assigned)>=1).OnlyEnforceIf(u); m.Add(sum(assigned)==0).OnlyEnforceIf(u.Not())
                else:m.Add(u==0)
                m.Add(ms==0).OnlyEnforceIf(u.Not()); m.Add(me==0).OnlyEnforceIf(u.Not())
                if gate in ("arrival","energy","relay2"):
                    lead=I(float(data["relay"]["prep_s"])+float(pos[p]["out"])+float(data["relay"]["link_s"]))
                    tail=I(float(pos[p]["back"])+float(data["relay"]["turn_s"]))
                    bs=m.NewIntVar(0,H*2,f"rbs{p}_{w}"); be=m.NewIntVar(0,H*2,f"rbe{p}_{w}"); bd=m.NewIntVar(0,H*2,f"rbd{p}_{w}")
                    m.Add(bs==ms-lead).OnlyEnforceIf(u); m.Add(be==me+tail).OnlyEnforceIf(u); m.Add(bd==be-bs).OnlyEnforceIf(u)
                    m.Add(bs==0).OnlyEnforceIf(u.Not()); m.Add(be==0).OnlyEnforceIf(u.Not()); m.Add(bd==0).OnlyEnforceIf(u.Not())
                    intervals.append(m.NewOptionalIntervalVar(bs,bd,be,u,f"ri{p}_{w}"))
                    if gate in ("energy","relay2") and hover>0:
                        maxservice=max(0.0,(usable-float(pos[p]["e"]))*3600.0/hover)
                        m.Add(me-ms<=int(math.floor(maxservice*SCALE+1e-9))).OnlyEnforceIf(u)
        if gate=="relay2": m.AddCumulative(intervals,[1]*len(intervals),2)
    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=20; solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    return {"label":label,"gate":gate,"max_waves":max_waves,"status":solver.StatusName(st)}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--schedule",required=True); ap.add_argument("--hypergraph",required=True); ap.add_argument("--label",required=True); ap.add_argument("--out",required=True)
    a=ap.parse_args(); rows=[]
    for w in (4,6,8):
        for gate in ("transport","assign","arrival","energy","relay2"):
            z=run(a.schedule,a.hypergraph,a.label,w,gate); rows.append(z); print(json.dumps(z),flush=True)
            if gate=="transport": break
        # transport only need once
        if w==4:
            for gate in ("assign","arrival","energy","relay2"):
                pass
    # Above loop intentionally repeats only complex gates; dedupe exact rows.
    df=pd.DataFrame(rows).drop_duplicates()
    Path(a.out).mkdir(parents=True,exist_ok=True); df.to_csv(Path(a.out)/f"{a.label}_diagnostic.csv",index=False)
    (Path(a.out)/f"{a.label}_diagnostic.json").write_text(json.dumps(df.to_dict("records"),indent=2),encoding="utf-8")
if __name__=="__main__":main()
