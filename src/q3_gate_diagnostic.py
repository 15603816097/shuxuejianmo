from __future__ import annotations
import argparse, ast, json, math
from pathlib import Path
from collections import defaultdict
import pandas as pd
from ortools.sat.python import cp_model
from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy

S=10
def I(x): return int(round(float(x)*S))
def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return [str(x) for x in json.loads(str(v))]
    except Exception: return [str(x) for x in ast.literal_eval(str(v))]

def solve_case(schedule,hg,label,mode,max_waves=6):
    data=load_inputs()
    sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=pd.read_csv(Path(hg)/"block_diagnostics.csv")
    cover=pd.read_csv(Path(hg)/"minimum_cover_positions.csv").reset_index(drop=True)

    elig=defaultdict(list)
    pos=[]
    for pidx,r in cover.iterrows():
        for b in str(r.covered_blocks).split("|"):
            elig[b].append(pidx)
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(r.altitude_m),float(data["center"]["海拔（m）"]))
        pos.append({"tout":tout,"tback":tback,"base_e":eout+eback})

    m=cp_model.CpModel(); H=I(16000)
    starts=[]; air=defaultdict(list); bat=defaultdict(list)
    for i,r in sch.iterrows():
        sv=m.NewIntVar(0,H,f"s{i}")
        dur=I(r.duration_s); ev=m.NewIntVar(0,H*2,f"e{i}"); m.Add(ev==sv+dur)
        starts.append(sv)
        if pd.notna(r.latest_start_s):
            m.Add(sv<=int(math.floor(float(r.latest_start_s)*S+1e-9)))
        typ=str(r.drone_type)
        air[typ].append(m.NewIntervalVar(sv,dur,ev,f"air{i}"))
        bd=I(float(r.duration_s)+float(r.charge_s))
        bev=m.NewIntVar(0,H*2,f"be{i}"); m.Add(bev==sv+bd)
        bat[typ].append(m.NewIntervalVar(sv,bd,bev,f"bat{i}"))
    for typ,x in air.items(): m.AddCumulative(x,[1]*len(x),len(data["aircraft"][typ]))
    for typ,x in bat.items(): m.AddCumulative(x,[1]*len(x),int(data["batteries"][typ]))

    if mode=="transport":
        solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=20; solver.parameters.num_search_workers=8
        st=solver.Solve(m); return solver.StatusName(st)

    abs_s=[]; abs_e=[]
    for bi,b in blocks.iterrows():
        a=m.NewIntVar(0,H*2,f"bs{bi}"); e=m.NewIntVar(0,H*2,f"be{bi}")
        m.Add(a==starts[int(b.route_index)]+I(b.start_s))
        m.Add(e==starts[int(b.route_index)]+I(b.end_s))
        abs_s.append(a); abs_e.append(e)

    hover=float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"])
    usable=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
    intervals=[]; uses=[]; mission_info=[]
    y={}
    for bi,b in blocks.iterrows():
        opts=[]
        for p in elig[str(b.block_id)]:
            for w in range(max_waves):
                z=m.NewBoolVar(f"y_{bi}_{p}_{w}"); y[bi,p,w]=z; opts.append(z)
        m.Add(sum(opts)==1)

    for p in range(len(pos)):
        for w in range(max_waves):
            u=m.NewBoolVar(f"u_{p}_{w}"); uses.append(u)
            ms=m.NewIntVar(0,H*2,f"ms{p}_{w}"); me=m.NewIntVar(0,H*2,f"me{p}_{w}")
            assigned=[]
            for bi,b in blocks.iterrows():
                z=y.get((bi,p,w))
                if z is None: continue
                assigned.append(z); m.AddImplication(z,u)
                m.Add(ms<=abs_s[bi]).OnlyEnforceIf(z)
                m.Add(me>=abs_e[bi]).OnlyEnforceIf(z)
            if assigned:
                m.Add(sum(assigned)>=1).OnlyEnforceIf(u)
                m.Add(sum(assigned)==0).OnlyEnforceIf(u.Not())
            else:
                m.Add(u==0)
            m.Add(ms==0).OnlyEnforceIf(u.Not()); m.Add(me==0).OnlyEnforceIf(u.Not())

            if mode in ("lead","relay2","mission_energy","component"):
                lead=I(float(data["relay"]["prep_s"])+float(pos[p]["tout"])+float(data["relay"]["link_s"]))
                tail=I(float(pos[p]["tback"])+float(data["relay"]["turn_s"]))
                bs=m.NewIntVar(0,H*2,f"rbs{p}_{w}"); be=m.NewIntVar(0,H*2,f"rbe{p}_{w}"); bd=m.NewIntVar(0,H*2,f"rbd{p}_{w}")
                m.Add(bs==ms-lead).OnlyEnforceIf(u); m.Add(be==me+tail).OnlyEnforceIf(u); m.Add(bd==be-bs).OnlyEnforceIf(u)
                m.Add(bs==0).OnlyEnforceIf(u.Not()); m.Add(be==0).OnlyEnforceIf(u.Not()); m.Add(bd==0).OnlyEnforceIf(u.Not())
                iv=m.NewOptionalIntervalVar(bs,bd,be,u,f"ri{p}_{w}")
                intervals.append(iv)
                mission_info.append((p,w,u,ms,me,bs,be,iv))
                if mode in ("mission_energy","component"):
                    max_service=max(0.0,(usable-float(pos[p]["base_e"]))*3600.0/hover)
                    m.Add(me-ms<=int(math.floor(max_service*S+1e-9))).OnlyEnforceIf(u)

    if mode in ("relay2","mission_energy","component"):
        m.AddCumulative(intervals,[1]*len(intervals),2)

    if mode=="component":
        inv=int(data["relay"]["component_inventory"])
        # Conservative component occupancy: mission busy interval plus full recharge time.
        comp_intervals=[]
        charge=I(float(data["relay"]["component_full_charge_s"]))
        for p,w,u,ms,me,bs,be,iv in mission_info:
            ce=m.NewIntVar(0,H*3,f"ce{p}_{w}")
            cd=m.NewIntVar(0,H*3,f"cd{p}_{w}")
            m.Add(ce==be+charge).OnlyEnforceIf(u)
            m.Add(cd==ce-bs).OnlyEnforceIf(u)
            m.Add(ce==0).OnlyEnforceIf(u.Not()); m.Add(cd==0).OnlyEnforceIf(u.Not())
            comp_intervals.append(m.NewOptionalIntervalVar(bs,cd,ce,u,f"comp{p}_{w}"))
        m.AddCumulative(comp_intervals,[1]*len(comp_intervals),inv)

    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=30; solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    return solver.StatusName(st)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True); ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--label",required=True); ap.add_argument("--out",required=True)
    a=ap.parse_args()
    rows=[]
    for mode in ["transport","assign","lead","relay2","mission_energy","component"]:
        st=solve_case(a.schedule,a.hypergraph,a.label,mode,6)
        z={"label":a.label,"mode":mode,"status":st}; rows.append(z); print(json.dumps(z),flush=True)
        if st=="INFEASIBLE": break
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    pd.DataFrame(rows).to_csv(out/f"{a.label}_gate_diagnostic.csv",index=False)
    (out/f"{a.label}_gate_diagnostic.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")
if __name__=="__main__":main()
