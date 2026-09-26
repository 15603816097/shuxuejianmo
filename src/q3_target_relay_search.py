"""Search an exact-K Q3 schedule with exactly M relay sorties under a hard
joint-completion target.

The transport grouping is fixed by an input selected_schedule, but all
transport start times, strict hover signatures, block-to-relay assignments,
and relay service windows are re-optimized together.  The strict hover
signatures come from the 1596-point continuous-communication hypergraph.
"""
from __future__ import annotations
import argparse, ast, json, math
from collections import defaultdict
from pathlib import Path
import pandas as pd
from ortools.sat.python import cp_model

from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy
from q3_semantics_core import build_authoritative_timeline, deadline_ledger

S=1000
H=18000*S

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return [str(x) for x in json.loads(str(v))]
    except Exception: return [str(x) for x in ast.literal_eval(str(v))]

def load_signatures(data,hg):
    c=pd.read_csv(Path(hg)/"candidate_coverage.csv")
    sigs={}
    for _,r in c.iterrows():
        blocks=tuple(sorted(x for x in str(r.covered_blocks).split("|") if x and x!="nan"))
        if not blocks: continue
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(
            data,dist,terrain,float(data["center"]["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(
            data,dist,terrain,float(r.altitude_m),float(data["center"]["海拔（m）"]))
        rec={"blocks":set(blocks),"lon":float(r.lon),"lat":float(r.lat),
             "agl_m":float(r.agl_m),"altitude_m":float(r.altitude_m),
             "tout":float(tout),"tback":float(tback),
             "base_e":float(eout+eback),"source":str(r.get("source",""))}
        key=(rec["tout"]+rec["tback"],rec["base_e"])
        if blocks not in sigs or key<sigs[blocks][0]:
            sigs[blocks]=(key,rec)
    return [v[1] for v in sigs.values()]

def main(schedule,hg,relay_sorties,target_s,outdir,time_limit):
    data=load_inputs()
    sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=pd.read_csv(Path(hg)/"block_diagnostics.csv").reset_index(drop=True)
    sigs=load_signatures(data,hg)
    out=Path(outdir); out.mkdir(parents=True,exist_ok=True)

    m=cp_model.CpModel()
    starts=[]; ends=[]; air=defaultdict(list); bat=defaultdict(list)
    timelines={}
    late_flags=[]; late_amounts=[]

    for i,r in sch.iterrows():
        ids=parse_ids(r.box_ids)
        tl=build_authoritative_timeline(data,(str(r.service),),str(r.drone_type),ids)
        timelines[i]=tl
        s=m.NewIntVar(0,H,f"s{i}")
        dur=int(math.ceil(float(r.duration_s)*S-1e-12))
        e=m.NewIntVar(0,H*2,f"e{i}"); m.Add(e==s+dur)
        starts.append(s); ends.append(e)
        typ=str(r.drone_type)
        air[typ].append(m.NewIntervalVar(s,dur,e,f"air{i}"))
        bd=int(math.ceil((float(r.duration_s)+float(r.charge_s))*S-1e-12))
        be=m.NewIntVar(0,H*2,f"be{i}"); m.Add(be==s+bd)
        bat[typ].append(m.NewIntervalVar(s,bd,be,f"bat{i}"))

        for j,d in enumerate(deadline_ledger(data,tl)):
            arr=s+int(math.ceil(float(d["delivery_time_s"])*S-1e-12))
            due=int(math.floor(float(d["deadline_s"])*S+1e-12))
            if bool(d["hard"]):
                m.Add(arr<=due)
            else:
                lv=m.NewIntVar(0,H*2,f"late_{i}_{j}")
                fl=m.NewBoolVar(f"lateflag_{i}_{j}")
                m.Add(lv>=arr-due)
                m.Add(lv==0).OnlyEnforceIf(fl.Not())
                m.Add(lv>=1).OnlyEnforceIf(fl)
                m.Add(lv<=H*2*fl)
                late_flags.append(fl); late_amounts.append(lv)

    for typ,ints in air.items():
        m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat.items():
        m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    abs_s=[]; abs_e=[]
    for bi,b in blocks.iterrows():
        a=m.NewIntVar(0,H*2,f"block_s{bi}")
        e=m.NewIntVar(0,H*2,f"block_e{bi}")
        m.Add(a==starts[int(b.route_index)]+int(math.floor(float(b.start_s)*S+1e-12)))
        m.Add(e==starts[int(b.route_index)]+int(math.ceil(float(b.end_s)*S-1e-12)))
        abs_s.append(a); abs_e.append(e)

    M=int(relay_sorties)
    elig=defaultdict(list)
    for p,sig in enumerate(sigs):
        for bid in sig["blocks"]: elig[bid].append(p)
    uncovered=[str(b.block_id) for _,b in blocks.iterrows() if not elig[str(b.block_id)]]
    if uncovered:
        raise RuntimeError(f"strict hypergraph has uncovered blocks: {uncovered[:10]}")

    choose={(j,p):m.NewBoolVar(f"choose_{j}_{p}") for j in range(M) for p in range(len(sigs))}
    ms=[m.NewIntVar(0,H*2,f"ms{j}") for j in range(M)]
    me=[m.NewIntVar(0,H*2,f"me{j}") for j in range(M)]
    for j in range(M):
        m.Add(sum(choose[j,p] for p in range(len(sigs)))==1)

    assign={}
    for bi,b in blocks.iterrows():
        bid=str(b.block_id); opts=[]
        for j in range(M):
            for p in elig[bid]:
                z=m.NewBoolVar(f"a_{bi}_{j}_{p}")
                assign[bi,j,p]=z; opts.append(z)
                m.AddImplication(z,choose[j,p])
                m.Add(ms[j]<=abs_s[bi]).OnlyEnforceIf(z)
                m.Add(me[j]>=abs_e[bi]).OnlyEnforceIf(z)
        m.Add(sum(opts)==1)

    hover=float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"])
    usable=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
    fullchg=int(math.ceil(float(data["relay"]["component_full_charge_s"])*S-1e-12))
    busy=[]; comps=[]; bsvars=[]; bevars=[]
    for j in range(M):
        az=[z for (bi,jj,p),z in assign.items() if jj==j]
        m.Add(sum(az)>=1)
        bs=m.NewIntVar(0,H*2,f"rbs{j}")
        be=m.NewIntVar(0,H*2,f"rbe{j}")
        bd=m.NewIntVar(0,H*2,f"rbd{j}")
        bsvars.append(bs); bevars.append(be)
        for p,sig in enumerate(sigs):
            lead=int(math.ceil((float(data["relay"]["prep_s"])+sig["tout"]+float(data["relay"]["link_s"]))*S-1e-12))
            tail=int(math.ceil((sig["tback"]+float(data["relay"]["turn_s"]))*S-1e-12))
            m.Add(bs==ms[j]-lead).OnlyEnforceIf(choose[j,p])
            m.Add(be==me[j]+tail).OnlyEnforceIf(choose[j,p])
            maxsvc=max(0.0,(usable-sig["base_e"])*3600.0/hover)
            m.Add(me[j]-ms[j] <= int(math.floor(maxsvc*S+1e-9))).OnlyEnforceIf(choose[j,p])
        m.Add(bd==be-bs)
        busy.append(m.NewIntervalVar(bs,bd,be,f"relay{j}"))
        ce=m.NewIntVar(0,H*3,f"ce{j}"); cd=m.NewIntVar(0,H*3,f"cd{j}")
        m.Add(ce==be+fullchg); m.Add(cd==ce-bs)
        comps.append(m.NewIntervalVar(bs,cd,ce,f"comp{j}"))

    m.AddCumulative(busy,[1]*M,2)
    m.AddCumulative(comps,[1]*M,int(data["relay"]["component_inventory"]))

    joint=m.NewIntVar(0,H*3,"joint")
    for e in ends: m.Add(joint>=e)
    for e in bevars: m.Add(joint>=e)
    m.Add(joint<=int(math.floor(float(target_s)*S+1e-9)))

    late_count=sum(late_flags) if late_flags else 0
    total_late=sum(late_amounts) if late_amounts else 0
    total_span=sum(me[j]-ms[j] for j in range(M))
    # Strong lexicographic surrogate: late boxes -> total lateness -> completion -> relay span.
    m.Minimize(late_count*2_000_000_000_000 + total_late*1000 + joint*10 + total_span)

    solver=cp_model.CpSolver()
    solver.parameters.max_time_in_seconds=float(time_limit)
    solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    rec={"status":solver.StatusName(st),"transport_routes":len(sch),"relay_sorties":M,
         "target_s":float(target_s),"signature_count":len(sigs),"blocks":len(blocks)}
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        (out/"summary.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
        print(json.dumps(rec,ensure_ascii=False,indent=2))
        raise SystemExit(2)

    tr=[]
    for i,r in sch.iterrows():
        nr=r.to_dict()
        nr["start_s"]=solver.Value(starts[i])/S
        nr["return_s"]=float(nr["start_s"])+float(r.duration_s)
        tr.append(nr)

    missions=[]; assignment_rows=[]
    for j in range(M):
        p=next(p for p in range(len(sigs)) if solver.Value(choose[j,p]))
        sig=sigs[p]
        ss=solver.Value(ms[j])/S; ee=solver.Value(me[j])/S
        bs=solver.Value(bsvars[j])/S; be=solver.Value(bevars[j])/S
        energy=sig["base_e"]+hover*(ee-ss)/3600.0
        missions.append({"mission_id":f"RLY-{j+1:02d}","position_index":p,
                         "lon":sig["lon"],"lat":sig["lat"],"agl_m":sig["agl_m"],
                         "altitude_m":sig["altitude_m"],"service_start_s":ss,
                         "service_end_s":ee,"busy_start_s":bs,"busy_end_s":be,
                         "service_duration_s":ee-ss,"energy_kwh":energy,
                         "soc_end":1-energy/float(data["relay"]["energy_kwh"])})
        for bi,b in blocks.iterrows():
            z=assign.get((bi,j,p))
            if z is not None and solver.Value(z):
                assignment_rows.append({"block_id":str(b.block_id),
                                        "route_index":int(b.route_index),
                                        "mission_id":f"RLY-{j+1:02d}",
                                        "position_index":p,
                                        "block_start_s":solver.Value(abs_s[bi])/S,
                                        "block_end_s":solver.Value(abs_e[bi])/S})

    rec.update({"soft_violation_boxes":int(solver.Value(late_count)) if late_flags else 0,
                "total_soft_lateness_s":solver.Value(total_late)/S if late_amounts else 0.0,
                "joint_completion_s":solver.Value(joint)/S,
                "transport_energy_kwh":float(sum(float(x["transport_energy_kwh"]) for x in tr)),
                "relay_energy_kwh":float(sum(x["energy_kwh"] for x in missions))})
    rec["joint_energy_kwh"]=rec["transport_energy_kwh"]+rec["relay_energy_kwh"]

    pd.DataFrame(tr).to_csv(out/"transport_schedule.csv",index=False,encoding="utf-8-sig")
    pd.DataFrame(missions).to_csv(out/"relay_missions.csv",index=False,encoding="utf-8-sig")
    pd.DataFrame(assignment_rows).to_csv(out/"block_assignments.csv",index=False,encoding="utf-8-sig")
    (out/"summary.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(rec,ensure_ascii=False,indent=2))

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True)
    ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--relay-sorties",type=int,required=True)
    ap.add_argument("--target-s",type=float,default=7000.0)
    ap.add_argument("--time-limit",type=float,default=300.0)
    ap.add_argument("--out",required=True)
    a=ap.parse_args()
    main(a.schedule,a.hypergraph,a.relay_sorties,a.target_s,a.out,a.time_limit)
