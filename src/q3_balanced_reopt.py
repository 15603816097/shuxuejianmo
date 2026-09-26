"""Fine-time repair for JOINT-18x5 with fixed five relay missions/positions.

Re-optimizes the 18 transport start times on a 1 ms grid while keeping the
chosen five relay mission positions and block-to-mission assignments fixed.
Exact relay service envelopes are derived from authoritative floating-point
block offsets, so the repaired schedule respects the final continuous-time
semantics instead of the earlier 0.1 s CP-SAT rounding.
"""
from __future__ import annotations
import ast, json, math
from pathlib import Path
from collections import defaultdict
import pandas as pd
from ortools.sat.python import cp_model
from minimal_pipeline import load_inputs, line_geometry
from q3_semantics_core import build_authoritative_timeline, direct_and_relay_blocks, deadline_ledger
from q3_official_semantics import relay_leg_time_energy, component_charge_time

S=1000
H=18000*S
def I(x): return int(round(float(x)*S))
def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try:return [str(x) for x in json.loads(str(v))]
    except Exception:return [str(x) for x in ast.literal_eval(str(v))]

def main(base,outdir):
    base=Path(base); out=Path(outdir); out.mkdir(parents=True,exist_ok=True)
    data=load_inputs()
    tr=pd.read_csv(base/"transport_schedule.csv").reset_index(drop=True)
    missions=pd.read_csv(base/"relay_missions.csv").copy()
    ba=pd.read_csv(base/"block_assignments.csv").copy()

    # authoritative exact block offsets by route, keyed to existing block ids by order
    block_offsets={}; route_deadlines={}
    for ri,r in tr.iterrows():
        tl=build_authoritative_timeline(data,(str(r.service),),str(r.drone_type),parse_ids(r.box_ids))
        route_deadlines[ri]=deadline_ledger(data,tl)
        bl=direct_and_relay_blocks(data,tl)
        rows=ba[ba.route_index.astype(int)==ri].sort_values("block_start_s").reset_index(drop=True)
        if len(rows)!=len(bl): raise RuntimeError(f"route {ri} block mismatch")
        for j,b in enumerate(bl):
            block_offsets[str(rows.iloc[j].block_id)]=(ri,float(b["start_s"]),float(b["end_s"]))

    m=cp_model.CpModel()
    starts=[]; ends=[]; air=defaultdict(list); bat=defaultdict(list)
    for i,r in tr.iterrows():
        s=m.NewIntVar(0,H,f"s{i}"); e=m.NewIntVar(0,H*2,f"e{i}")
        dur=int(math.ceil(float(r.duration_s)*S-1e-12)); m.Add(e==s+dur)
        if pd.notna(r.latest_start_s): m.Add(s<=int(math.floor(float(r.latest_start_s)*S+1e-9)))
        starts.append(s); ends.append(e)
        typ=str(r.drone_type)
        air[typ].append(m.NewIntervalVar(s,dur,e,f"air{i}"))
        bd=int(math.ceil((float(r.duration_s)+float(r.charge_s))*S-1e-12)); be=m.NewIntVar(0,H*2,f"be{i}"); m.Add(be==s+bd)
        bat[typ].append(m.NewIntervalVar(s,bd,be,f"bat{i}"))
    for typ,ints in air.items(): m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat.items(): m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

    # Lexicographic delivery-timeliness objective on the frozen 18-route plan.
    # Hard deadlines remain constraints. For ordinary boxes, first minimize the
    # number of late boxes, then total lateness.
    late_flags=[]; late_amounts=[]
    for ri, rows in route_deadlines.items():
        for k,d in enumerate(rows):
            delivery_off=int(math.ceil(float(d["delivery_time_s"])*S-1e-12))
            deadline=int(math.floor(float(d["deadline_s"])*S+1e-12))
            if bool(d["hard"]):
                m.Add(starts[ri]+delivery_off <= deadline)
            else:
                late=m.NewIntVar(0,H*2,f"late_{ri}_{k}")
                flag=m.NewBoolVar(f"lateflag_{ri}_{k}")
                m.Add(late >= starts[ri]+delivery_off-deadline)
                m.Add(late == 0).OnlyEnforceIf(flag.Not())
                m.Add(late >= 1).OnlyEnforceIf(flag)
                m.Add(late <= H*2*flag)
                late_amounts.append(late); late_flags.append(flag)

    # five fixed missions, fixed positions; service window is exact envelope of assigned block offsets
    rdat=data["relay"]; hover=float(rdat["hover_power_kw"])+float(rdat["comm_power_kw"])
    relay_intervals=[]; msvars={}; mevars={}; bsvars={}; bevars={}
    for _,mr in missions.iterrows():
        mid=str(mr.mission_id)
        ass=ba[ba.mission_id.astype(str)==mid]
        if ass.empty: raise RuntimeError(f"{mid} has no blocks")
        ms=m.NewIntVar(0,H*2,f"ms_{mid}"); me=m.NewIntVar(0,H*2,f"me_{mid}")
        msvars[mid]=ms; mevars[mid]=me
        for _,a in ass.iterrows():
            ri,off0,off1=block_offsets[str(a.block_id)]
            m.Add(ms <= starts[ri]+int(math.floor(off0*S+1e-12)))
            m.Add(me >= starts[ri]+int(math.ceil(off1*S-1e-12)))
        # force tight envelope by objective later, but exact containment is enough.
        q={"经度（°）":float(mr.lon),"纬度（°）":float(mr.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(mr.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(mr.altitude_m),float(data["center"]["海拔（m）"]))
        lead=I(float(rdat["prep_s"])+tout+float(rdat["link_s"]))
        tail=I(tback+float(rdat["turn_s"]))
        bs=m.NewIntVar(0,H*2,f"bs_{mid}"); be=m.NewIntVar(0,H*2,f"be_{mid}"); bd=m.NewIntVar(0,H*2,f"bd_{mid}")
        m.Add(bs==ms-lead); m.Add(be==me+tail); m.Add(bd==be-bs)
        bsvars[mid]=bs; bevars[mid]=be
        relay_intervals.append(m.NewIntervalVar(bs,bd,be,f"relay_{mid}"))
        usable=(1-float(rdat["reserve"]))*float(rdat["energy_kwh"])
        base_e=float(eout+eback)
        maxsvc=max(0.0,(usable-base_e)*3600.0/hover)
        m.Add(me-ms<=int(math.floor(maxsvc*S+1e-9)))
    m.AddCumulative(relay_intervals,[1]*len(relay_intervals),2)

    # Conservative component occupancy through full recharge to guarantee feasibility.
    comp_ints=[]; fullchg=I(float(rdat["component_full_charge_s"]))
    for _,mr in missions.iterrows():
        mid=str(mr.mission_id); ce=m.NewIntVar(0,H*3,f"ce_{mid}"); cd=m.NewIntVar(0,H*3,f"cd_{mid}")
        m.Add(ce==bevars[mid]+fullchg); m.Add(cd==ce-bsvars[mid])
        comp_ints.append(m.NewIntervalVar(bsvars[mid],cd,ce,f"comp_{mid}"))
    m.AddCumulative(comp_ints,[1]*len(comp_ints),int(rdat["component_inventory"]))

    joint=m.NewIntVar(0,H*3,"joint")
    for e in ends: m.Add(joint>=e)
    for be in bevars.values(): m.Add(joint>=be)
    total_span=sum(mevars[mid]-msvars[mid] for mid in msvars)
    late_count=sum(late_flags)
    total_late=sum(late_amounts)

    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=240; solver.parameters.num_search_workers=8

    # Keep the already-certified fast plan envelope: do not trade one fewer
    # late box for a much later disaster-response completion.
    m.Add(joint <= 9000*S)

    m.Minimize(late_count)
    st=solver.Solve(m)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        raise RuntimeError(f"lateness stage1 infeasible: {solver.StatusName(st)}")
    best_count=int(round(solver.ObjectiveValue())); m.Add(late_count==best_count)
    print(json.dumps({"stage":"late_box_count","status":solver.StatusName(st),"value":best_count},ensure_ascii=False))

    m.Minimize(total_late)
    st=solver.Solve(m)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        raise RuntimeError(f"lateness stage2 infeasible: {solver.StatusName(st)}")
    best_late=int(round(solver.ObjectiveValue())); m.Add(total_late==best_late)
    print(json.dumps({"stage":"total_soft_lateness_ms","status":solver.StatusName(st),"value":best_late},ensure_ascii=False))

    m.Minimize(joint)
    st=solver.Solve(m)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        raise RuntimeError(f"lateness stage3 infeasible: {solver.StatusName(st)}")
    best_joint=int(round(solver.ObjectiveValue())); m.Add(joint==best_joint)
    print(json.dumps({"stage":"joint_completion_ms","status":solver.StatusName(st),"value":best_joint},ensure_ascii=False))

    m.Minimize(total_span)
    st=solver.Solve(m)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
        raise RuntimeError(f"lateness stage4 infeasible: {solver.StatusName(st)}")
    print(json.dumps({"stage":"relay_service_span_ms","status":solver.StatusName(st),"value":int(round(solver.ObjectiveValue()))},ensure_ascii=False))

    for i in range(len(tr)):
        tr.at[i,"start_s"]=solver.Value(starts[i])/S
        tr.at[i,"return_s"]=float(tr.at[i,"start_s"])+float(tr.at[i,"duration_s"])

    # materialize exact mission timing/energy and exact component assignment
    cav={f"RE-{i+1:02d}":0.0 for i in range(int(rdat["component_inventory"]))}
    rav={"R01":0.0,"R02":0.0}
    for idx in missions.sort_values("busy_start_s").index:
        mid=str(missions.at[idx,"mission_id"])
        ass=ba[ba.mission_id.astype(str)==mid]
        exact_starts=[]; exact_ends=[]
        for _,a in ass.iterrows():
            ri,off0,off1=block_offsets[str(a.block_id)]
            exact_starts.append(solver.Value(starts[ri])/S + off0)
            exact_ends.append(solver.Value(starts[ri])/S + off1)
        ss=min(exact_starts); ee=max(exact_ends)
        q={"经度（°）":float(missions.at[idx,"lon"]),"纬度（°）":float(missions.at[idx,"lat"])}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(missions.at[idx,"altitude_m"]))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(missions.at[idx,"altitude_m"]),float(data["center"]["海拔（m）"]))
        bs=ss-float(rdat["prep_s"])-tout-float(rdat["link_s"])
        be=ee+tback+float(rdat["turn_s"])
        energy=eout+eback+hover*(ee-ss)/3600.0; soc=1-energy/float(rdat["energy_kwh"])
        rid=min(rav,key=lambda k:(rav[k],k))
        if rav[rid]>bs+1e-9: raise RuntimeError("R=2 coloring failed")
        rav[rid]=be
        cid=min(cav,key=lambda k:(cav[k],k))
        if cav[cid]>bs+1e-9: raise RuntimeError("component assignment failed")
        ch=component_charge_time(soc,float(rdat["component_full_charge_s"])); cav[cid]=be+ch
        vals=[ss,ee,bs,be,ee-ss,energy,soc,rid,cid,ch,be+ch]
        cols=["service_start_s","service_end_s","busy_start_s","busy_end_s","service_duration_s","energy_kwh","soc_end","relay_id","component_id","component_charge_s","component_available_again_s"]
        for c,v in zip(cols,vals): missions.at[idx,c]=v

    # refresh absolute block assignment times
    for i,a in ba.iterrows():
        ri,off0,off1=block_offsets[str(a.block_id)]
        ba.at[i,"block_start_s"]=float(tr.at[ri,"start_s"])+off0
        ba.at[i,"block_end_s"]=float(tr.at[ri,"start_s"])+off1

    rec={"solver_status":solver.StatusName(st),"joint_completion_s":solver.Value(joint)/S,
         "transport_routes":len(tr),"relay_sorties":len(missions),
         "soft_violation_boxes_optimized":best_count,
         "total_soft_lateness_s_optimized":best_late/S,
         "joint_completion_cap_s":9000.0}
    tr.to_csv(out/"transport_schedule.csv",index=False,encoding="utf-8-sig")
    missions.to_csv(out/"relay_missions.csv",index=False,encoding="utf-8-sig")
    ba.to_csv(out/"block_assignments.csv",index=False,encoding="utf-8-sig")
    (out/"fine_repair_summary.json").write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(rec,ensure_ascii=False,indent=2))

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument("--base",required=True); ap.add_argument("--out",required=True)
    a=ap.parse_args(); main(a.base,a.out)
