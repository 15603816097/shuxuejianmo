"""Screen friend-style Q3 baselines with explicit relay time waves.

This stage keeps each baseline's single-service transport batches, but jointly
re-optimizes route start times and assigns every relay-required block to one of
the four certified hover positions and a relay time wave. It first tests exactly
4 relay sorties; if infeasible, it increases the relay-sortie cap until a
feasible schedule is found. This is a search-space result, not global-route
optimality.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
from collections import defaultdict
import pandas as pd
from ortools.sat.python import cp_model
from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy

S=10
def I(x): return int(round(float(x)*S))

def peak(ints):
    pts=[]
    for a,b in ints:
        if b>a: pts += [(a,1),(b,-1)]
    cur=best=0
    for _,d in sorted(pts,key=lambda z:(z[0],z[1])):
        cur+=d; best=max(best,cur)
    return best

def build_and_solve(schedule,hg,label,relay_cap):
    data=load_inputs()
    sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=pd.read_csv(Path(hg)/"block_diagnostics.csv").reset_index(drop=True)
    cover=pd.read_csv(Path(hg)/"minimum_cover_positions.csv").reset_index(drop=True)
    H=I(18000); m=cp_model.CpModel()

    # Map strict block coverage to the four selected geographic positions.
    elig=defaultdict(list)
    pos=[]
    for pidx,r in cover.iterrows():
        for bid in str(r.covered_blocks).split("|"):
            elig[bid].append(pidx)
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(r.altitude_m),float(data["center"]["海拔（m）"]))
        pos.append({"tout":float(tout),"tback":float(tback),"base_e":float(eout+eback)})

    starts=[]; returns=[]; air=defaultdict(list); bat=defaultdict(list)
    for i,r in sch.iterrows():
        sv=m.NewIntVar(0,H,f"s{i}"); dur=I(r.duration_s)
        ev=m.NewIntVar(0,H*2,f"e{i}"); m.Add(ev==sv+dur)
        starts.append(sv); returns.append(ev)
        if pd.notna(r.latest_start_s):
            m.Add(sv<=int(math.floor(float(r.latest_start_s)*S+1e-9)))
        typ=str(r.drone_type)
        air[typ].append(m.NewIntervalVar(sv,dur,ev,f"air{i}"))
        bd=I(float(r.duration_s)+float(r.charge_s))
        bev=m.NewIntVar(0,H*2,f"be{i}"); m.Add(bev==sv+bd)
        bat[typ].append(m.NewIntervalVar(sv,bd,bev,f"bat{i}"))
    for typ,ints in air.items():
        m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat.items():
        m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    abs_s=[]; abs_e=[]
    for bi,b in blocks.iterrows():
        a=m.NewIntVar(0,H*2,f"bs{bi}"); e=m.NewIntVar(0,H*2,f"be{bi}")
        m.Add(a==starts[int(b.route_index)]+I(b.start_s))
        m.Add(e==starts[int(b.route_index)]+I(b.end_s))
        abs_s.append(a); abs_e.append(e)

    # Up to relay_cap optional missions, each chooses one of the four positions.
    # A mission may cover multiple blocks only when that position certifies all
    # assigned blocks. R01/R02 concurrency and component occupancy are enforced.
    M=relay_cap
    use=[m.NewBoolVar(f"use{j}") for j in range(M)]
    choose={(j,p):m.NewBoolVar(f"choose{j}_{p}") for j in range(M) for p in range(len(pos))}
    ms=[m.NewIntVar(0,H*2,f"ms{j}") for j in range(M)]
    me=[m.NewIntVar(0,H*2,f"me{j}") for j in range(M)]
    busy_intervals=[]; comp_intervals=[]
    assign={}
    for bi,b in blocks.iterrows():
        opts=[]
        bid=str(b.block_id)
        for j in range(M):
            for p in elig[bid]:
                z=m.NewBoolVar(f"a{bi}_{j}_{p}"); assign[bi,j,p]=z; opts.append(z)
                m.AddImplication(z,use[j]); m.AddImplication(z,choose[j,p])
                m.Add(ms[j]<=abs_s[bi]).OnlyEnforceIf(z)
                m.Add(me[j]>=abs_e[bi]).OnlyEnforceIf(z)
        m.Add(sum(opts)==1)

    hover=float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"])
    usable=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
    fullchg=I(float(data["relay"]["component_full_charge_s"]))
    for j in range(M):
        m.Add(sum(choose[j,p] for p in range(len(pos)))==use[j])
        # used missions must serve at least one block
        az=[z for (bi,jj,p),z in assign.items() if jj==j]
        if az:
            m.Add(sum(az)>=1).OnlyEnforceIf(use[j])
            m.Add(sum(az)==0).OnlyEnforceIf(use[j].Not())
        m.Add(ms[j]==0).OnlyEnforceIf(use[j].Not()); m.Add(me[j]==0).OnlyEnforceIf(use[j].Not())

        # Position-dependent lead/tail and energy are linearized by enforcement.
        bs=m.NewIntVar(0,H*2,f"rbs{j}"); be=m.NewIntVar(0,H*2,f"rbe{j}"); bd=m.NewIntVar(0,H*2,f"rbd{j}")
        m.Add(bs==0).OnlyEnforceIf(use[j].Not()); m.Add(be==0).OnlyEnforceIf(use[j].Not()); m.Add(bd==0).OnlyEnforceIf(use[j].Not())
        for p in range(len(pos)):
            lead=I(float(data["relay"]["prep_s"])+pos[p]["tout"]+float(data["relay"]["link_s"]))
            tail=I(pos[p]["tback"]+float(data["relay"]["turn_s"]))
            m.Add(bs==ms[j]-lead).OnlyEnforceIf(choose[j,p])
            m.Add(be==me[j]+tail).OnlyEnforceIf(choose[j,p])
            maxsvc=max(0.0,(usable-pos[p]["base_e"])*3600.0/hover)
            m.Add(me[j]-ms[j] <= int(math.floor(maxsvc*S+1e-9))).OnlyEnforceIf(choose[j,p])
        m.Add(bd==be-bs).OnlyEnforceIf(use[j])
        iv=m.NewOptionalIntervalVar(bs,bd,be,use[j],f"relay{j}")
        busy_intervals.append(iv)

        # Conservative component occupancy through full recharge.
        ce=m.NewIntVar(0,H*3,f"ce{j}"); cd=m.NewIntVar(0,H*3,f"cd{j}")
        m.Add(ce==be+fullchg).OnlyEnforceIf(use[j])
        m.Add(cd==ce-bs).OnlyEnforceIf(use[j])
        m.Add(ce==0).OnlyEnforceIf(use[j].Not()); m.Add(cd==0).OnlyEnforceIf(use[j].Not())
        comp_intervals.append(m.NewOptionalIntervalVar(bs,cd,ce,use[j],f"comp{j}"))

    m.AddCumulative(busy_intervals,[1]*M,2)
    m.AddCumulative(comp_intervals,[1]*M,int(data["relay"]["component_inventory"]))

    # Symmetry: earlier mission ids used first.
    for j in range(M-1): m.Add(use[j]>=use[j+1])

    # Prefer fewer used missions within this cap, then earlier joint completion.
    joint=m.NewIntVar(0,H*3,"joint")
    for e in returns: m.Add(joint>=e)
    for j in range(M):
        m.Add(joint>=me[j]).OnlyEnforceIf(use[j])
    m.Minimize(sum(use)*H*4 + joint)

    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=120; solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    rec={"label":label,"relay_cap":relay_cap,"status":solver.StatusName(st),
         "routes":len(sch),"blocks":len(blocks)}
    if st in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        used=[j for j in range(M) if solver.Value(use[j])]
        rec.update({"relay_sorties":len(used),"joint_completion_s":solver.Value(joint)/S,
                    "four_sorties_feasible":len(used)<=4})
    return rec

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--schedule",required=True); ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--label",required=True); ap.add_argument("--out",default="results/q3_friend_wave_screen")
    a=ap.parse_args(); rows=[]
    for cap in (4,5,6,7,8):
        rec=build_and_solve(a.schedule,a.hypergraph,a.label,cap); rows.append(rec); print(json.dumps(rec,ensure_ascii=False),flush=True)
        if rec["status"] in ("OPTIMAL","FEASIBLE"): break
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    pd.DataFrame(rows).to_csv(out/f"{a.label}.csv",index=False)
    (out/f"{a.label}.json").write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding="utf-8")
if __name__=="__main__": main()
