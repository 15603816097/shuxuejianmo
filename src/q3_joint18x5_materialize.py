"""Materialize and audit the JOINT-18 + 5-relay candidate.

This takes the selected 18-route joint schedule and strict hypergraph, solves the
5-relay mission assignment, materializes mission/transport timings, colors the
two physical relay aircraft, assigns exact energy components with the official
two-stage recharge curve, and performs a strict block-level coverage replay.
It does not claim recursive 0.1 s continuous certification yet.
"""
from __future__ import annotations
import json, math
from pathlib import Path
from collections import defaultdict
import pandas as pd
from ortools.sat.python import cp_model

from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy, component_charge_time

S=10
def I(x): return int(round(float(x)*S))

def solve(schedule,hg,outdir):
    data=load_inputs()
    sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=pd.read_csv(Path(hg)/"block_diagnostics.csv").reset_index(drop=True)
    cover=pd.read_csv(Path(hg)/"minimum_cover_positions.csv").reset_index(drop=True)

    elig=defaultdict(list); pos=[]
    for pidx,r in cover.iterrows():
        for bid in str(r.covered_blocks).split("|"): elig[bid].append(pidx)
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(r.altitude_m),float(data["center"]["海拔（m）"]))
        pos.append({"lon":float(r.lon),"lat":float(r.lat),"agl_m":float(r.agl_m),"altitude_m":float(r.altitude_m),
                    "tout":float(tout),"tback":float(tback),"base_e":float(eout+eback),
                    "covered_blocks":set(str(r.covered_blocks).split("|"))})
    H=I(18000); m=cp_model.CpModel()
    starts=[]; returns=[]; air=defaultdict(list); bat=defaultdict(list)
    for i,r in sch.iterrows():
        s=m.NewIntVar(0,H,f"s{i}"); d=I(r.duration_s); e=m.NewIntVar(0,H*2,f"e{i}"); m.Add(e==s+d)
        starts.append(s); returns.append(e)
        if pd.notna(r.latest_start_s): m.Add(s<=int(math.floor(float(r.latest_start_s)*S+1e-9)))
        typ=str(r.drone_type)
        air[typ].append(m.NewIntervalVar(s,d,e,f"air{i}"))
        bd=I(float(r.duration_s)+float(r.charge_s)); be=m.NewIntVar(0,H*2,f"be{i}"); m.Add(be==s+bd)
        bat[typ].append(m.NewIntervalVar(s,bd,be,f"bat{i}"))
    for typ,ints in air.items(): m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat.items(): m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    abs_s=[]; abs_e=[]
    for bi,b in blocks.iterrows():
        a=m.NewIntVar(0,H*2,f"bs{bi}"); e=m.NewIntVar(0,H*2,f"be{bi}")
        m.Add(a==starts[int(b.route_index)]+I(b.start_s)); m.Add(e==starts[int(b.route_index)]+I(b.end_s))
        abs_s.append(a); abs_e.append(e)

    M=5
    choose={(j,p):m.NewBoolVar(f"c{j}_{p}") for j in range(M) for p in range(len(pos))}
    ms=[m.NewIntVar(0,H*2,f"ms{j}") for j in range(M)]
    me=[m.NewIntVar(0,H*2,f"me{j}") for j in range(M)]
    assign={}
    for j in range(M): m.Add(sum(choose[j,p] for p in range(len(pos)))==1)
    for bi,b in blocks.iterrows():
        opts=[]; bid=str(b.block_id)
        for j in range(M):
            for p in elig[bid]:
                z=m.NewBoolVar(f"a{bi}_{j}_{p}"); assign[bi,j,p]=z; opts.append(z)
                m.AddImplication(z,choose[j,p])
                m.Add(ms[j]<=abs_s[bi]).OnlyEnforceIf(z); m.Add(me[j]>=abs_e[bi]).OnlyEnforceIf(z)
        m.Add(sum(opts)==1)

    hover=float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"])
    usable=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
    busy=[]; bsvars=[]; bevars=[]
    for j in range(M):
        az=[z for (bi,jj,p),z in assign.items() if jj==j]
        m.Add(sum(az)>=1)
        bs=m.NewIntVar(0,H*2,f"rbs{j}"); be=m.NewIntVar(0,H*2,f"rbe{j}"); bd=m.NewIntVar(0,H*2,f"rbd{j}")
        bsvars.append(bs); bevars.append(be)
        for p in range(len(pos)):
            lead=I(float(data["relay"]["prep_s"])+pos[p]["tout"]+float(data["relay"]["link_s"]))
            tail=I(pos[p]["tback"]+float(data["relay"]["turn_s"]))
            m.Add(bs==ms[j]-lead).OnlyEnforceIf(choose[j,p]); m.Add(be==me[j]+tail).OnlyEnforceIf(choose[j,p])
            maxsvc=max(0.0,(usable-pos[p]["base_e"])*3600.0/hover)
            m.Add(me[j]-ms[j] <= int(math.floor(maxsvc*S+1e-9))).OnlyEnforceIf(choose[j,p])
        m.Add(bd==be-bs); busy.append(m.NewIntervalVar(bs,bd,be,f"relay{j}"))
    m.AddCumulative(busy,[1]*M,2)

    joint=m.NewIntVar(0,H*3,"joint")
    for e in returns: m.Add(joint>=e)
    for e in bevars: m.Add(joint>=e)
    m.Minimize(joint)

    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=180; solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        raise RuntimeError(f"5-relay materialization failed: {solver.StatusName(st)}")

    tr=[]; missions=[]; assignment_rows=[]
    for i,r in sch.iterrows():
        nr=r.to_dict(); nr["start_s"]=solver.Value(starts[i])/S; nr["return_s"]=solver.Value(returns[i])/S; tr.append(nr)
    for j in range(M):
        p=next(p for p in range(len(pos)) if solver.Value(choose[j,p]))
        s0=solver.Value(ms[j])/S; e0=solver.Value(me[j])/S
        b0=solver.Value(bsvars[j])/S; b1=solver.Value(bevars[j])/S
        service_dur=e0-s0
        energy=pos[p]["base_e"]+hover*service_dur/3600.0
        missions.append({"mission_id":f"RLY-{j+1:02d}","position_index":p,**pos[p],
                         "service_start_s":s0,"service_end_s":e0,
                         "busy_start_s":b0,"busy_end_s":b1,
                         "service_duration_s":service_dur,"energy_kwh":energy,
                         "soc_end":1-energy/float(data["relay"]["energy_kwh"])})
        for bi,b in blocks.iterrows():
            z=assign.get((bi,j,p))
            if z is not None and solver.Value(z):
                assignment_rows.append({"block_id":str(b.block_id),"route_index":int(b.route_index),
                                        "mission_id":f"RLY-{j+1:02d}","position_index":p,
                                        "block_start_s":solver.Value(abs_s[bi])/S,
                                        "block_end_s":solver.Value(abs_e[bi])/S})

    # Color 5 relay missions onto R01/R02.
    rel_av={"R01":0.0,"R02":0.0}
    for q in sorted(missions,key=lambda x:x["busy_start_s"]):
        rid=min(rel_av,key=lambda k:(rel_av[k],k))
        if rel_av[rid] > q["busy_start_s"]+1e-6:
            raise RuntimeError("two-relay coloring failed")
        q["relay_id"]=rid; rel_av[rid]=q["busy_end_s"]

    # Exact official two-stage component recharge. Inventory currently comes from workbook.
    comp_n=int(data["relay"]["component_inventory"])
    comp_av={f"RE-{i+1:02d}":0.0 for i in range(comp_n)}
    for q in sorted(missions,key=lambda x:x["busy_start_s"]):
        cid=min(comp_av,key=lambda k:(comp_av[k],k))
        if comp_av[cid] > q["busy_start_s"]+1e-6:
            raise RuntimeError("component inventory exact-charge assignment failed")
        charge=component_charge_time(q["soc_end"],float(data["relay"]["component_full_charge_s"]))
        q["component_id"]=cid; q["component_charge_s"]=charge
        q["component_available_again_s"]=q["busy_end_s"]+charge
        comp_av[cid]=q["component_available_again_s"]

    amap={x["block_id"]:x for x in assignment_rows}
    strict_ok=True
    for _,b in blocks.iterrows():
        a=amap[str(b.block_id)]
        q=next(x for x in missions if x["mission_id"]==a["mission_id"])
        ok=(q["service_start_s"]<=a["block_start_s"]+1e-6 and q["service_end_s"]>=a["block_end_s"]-1e-6 and
            str(b.block_id) in pos[q["position_index"]]["covered_blocks"])
        a["strict_block_covered"]=bool(ok); strict_ok &= bool(ok)

    summary={"status":"MATERIALIZED_5_RELAY_CANDIDATE","solver_status":solver.StatusName(st),
             "transport_routes":len(tr),"relay_sorties":len(missions),"relay_aircraft_peak_bound":2,
             "component_inventory":comp_n,"strict_block_coverage_pass":bool(strict_ok),
             "relay_energy_kwh":sum(x["energy_kwh"] for x in missions),
             "transport_energy_kwh":float(sum(float(x["transport_energy_kwh"]) for x in tr)),
             "joint_energy_kwh":float(sum(float(x["transport_energy_kwh"]) for x in tr)+sum(x["energy_kwh"] for x in missions)),
             "joint_completion_s":max(max(x["return_s"] for x in tr),max(x["busy_end_s"] for x in missions)),
             "scope_note":"Block-level strict hypergraph replay plus exact component recharge; recursive 0.1 s continuous interval certificate remains pending."}
    out=Path(outdir); out.mkdir(parents=True,exist_ok=True)
    pd.DataFrame(tr).to_csv(out/"transport_schedule.csv",index=False,encoding="utf-8-sig")
    pd.DataFrame(missions).to_csv(out/"relay_missions.csv",index=False,encoding="utf-8-sig")
    pd.DataFrame(assignment_rows).to_csv(out/"block_assignments.csv",index=False,encoding="utf-8-sig")
    (out/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument("--schedule",required=True); ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--out",default="results/q3_joint18x5_materialized")
    a=ap.parse_args(); solve(a.schedule,a.hypergraph,a.out)
