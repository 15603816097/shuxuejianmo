"""Q3 flexible-position four-sortie screen.

Unlike the minimum-cover screen, this model does not freeze the four geographic
positions chosen by set cover. It compresses all strict certified hover
candidates by identical block-coverage signatures and lets each of exactly four
relay sorties choose its own signature/representative position while jointly
re-optimizing transport starts. This tests whether four relay sorties are
possible even when more than four geographic candidate positions are available.
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

def load_signatures(data,hg):
    c=pd.read_csv(Path(hg)/"candidate_coverage.csv")
    sigs={}
    for _,r in c.iterrows():
        blocks=tuple(sorted(str(r.covered_blocks).split("|")))
        if not blocks: continue
        q={"经度（°）":float(r.lon),"纬度（°）":float(r.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(r.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(r.altitude_m),float(data["center"]["海拔（m）"]))
        rec={"blocks":set(blocks),"lon":float(r.lon),"lat":float(r.lat),"agl_m":float(r.agl_m),
             "altitude_m":float(r.altitude_m),"tout":float(tout),"tback":float(tback),
             "base_e":float(eout+eback),"source":str(r.source)}
        # Keep the representative with smallest travel-time lead; tie-break by energy.
        key=(rec["tout"]+rec["tback"],rec["base_e"])
        if blocks not in sigs or key < sigs[blocks][0]:
            sigs[blocks]=(key,rec)
    return [x[1] for x in sigs.values()]

def solve(schedule,hg,label):
    data=load_inputs(); sch=pd.read_csv(schedule).reset_index(drop=True)
    blocks=pd.read_csv(Path(hg)/"block_diagnostics.csv").reset_index(drop=True)
    sigs=load_signatures(data,hg)
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

    M=4
    choose={(j,p):m.NewBoolVar(f"c{j}_{p}") for j in range(M) for p in range(len(sigs))}
    for j in range(M): m.Add(sum(choose[j,p] for p in range(len(sigs)))==1)

    elig=defaultdict(list)
    for p,sig in enumerate(sigs):
        for bid in sig["blocks"]: elig[bid].append(p)

    assign={}
    for bi,b in blocks.iterrows():
        opts=[]
        bid=str(b.block_id)
        for j in range(M):
            for p in elig[bid]:
                z=m.NewBoolVar(f"a{bi}_{j}_{p}"); assign[bi,j,p]=z; opts.append(z)
                m.AddImplication(z,choose[j,p])
        m.Add(sum(opts)==1)

    busy=[]; comps=[]; ms=[]; me=[]
    hover=float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"])
    usable=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
    fullchg=I(float(data["relay"]["component_full_charge_s"]))
    for j in range(M):
        s0=m.NewIntVar(0,H*2,f"ms{j}"); e0=m.NewIntVar(0,H*2,f"me{j}"); ms.append(s0); me.append(e0)
        for bi,b in blocks.iterrows():
            for p in elig[str(b.block_id)]:
                z=assign.get((bi,j,p))
                if z is None: continue
                m.Add(s0<=abs_s[bi]).OnlyEnforceIf(z); m.Add(e0>=abs_e[bi]).OnlyEnforceIf(z)
        bs=m.NewIntVar(0,H*2,f"rbs{j}"); be=m.NewIntVar(0,H*2,f"rbe{j}"); bd=m.NewIntVar(0,H*2,f"rbd{j}")
        for p,sig in enumerate(sigs):
            lead=I(float(data["relay"]["prep_s"])+sig["tout"]+float(data["relay"]["link_s"]))
            tail=I(sig["tback"]+float(data["relay"]["turn_s"]))
            m.Add(bs==s0-lead).OnlyEnforceIf(choose[j,p]); m.Add(be==e0+tail).OnlyEnforceIf(choose[j,p])
            maxsvc=max(0.0,(usable-sig["base_e"])*3600.0/hover)
            m.Add(e0-s0<=int(math.floor(maxsvc*S+1e-9))).OnlyEnforceIf(choose[j,p])
        m.Add(bd==be-bs)
        busy.append(m.NewIntervalVar(bs,bd,be,f"relay{j}"))
        ce=m.NewIntVar(0,H*3,f"ce{j}"); cd=m.NewIntVar(0,H*3,f"cd{j}")
        m.Add(ce==be+fullchg); m.Add(cd==ce-bs)
        comps.append(m.NewIntervalVar(bs,cd,ce,f"comp{j}"))

    m.AddCumulative(busy,[1]*M,2)
    m.AddCumulative(comps,[1]*M,int(data["relay"]["component_inventory"]))

    joint=m.NewIntVar(0,H*3,"joint")
    for e in returns: m.Add(joint>=e)
    for e in me: m.Add(joint>=e)
    m.Minimize(joint)

    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=180; solver.parameters.num_search_workers=8
    st=solver.Solve(m)
    rec={"label":label,"routes":len(sch),"blocks":len(blocks),"signature_count":len(sigs),
         "relay_sorties":4,"status":solver.StatusName(st)}
    if st in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        chosen=[]
        for j in range(M):
            p=next(p for p in range(len(sigs)) if solver.Value(choose[j,p]))
            chosen.append({"mission":j+1,"signature":p,"lon":sigs[p]["lon"],"lat":sigs[p]["lat"],
                           "agl_m":sigs[p]["agl_m"],"covered_blocks":len(sigs[p]["blocks"])})
        rec["joint_completion_s"]=solver.Value(joint)/S; rec["chosen"]=chosen
    return rec

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--schedule",required=True); ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--label",required=True); ap.add_argument("--out",default="results/q3_flexible_four")
    a=ap.parse_args(); rec=solve(a.schedule,a.hypergraph,a.label); print(json.dumps(rec,ensure_ascii=False,indent=2))
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    (out/f"{a.label}.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
if __name__=="__main__": main()
