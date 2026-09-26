"""Q3 global K22 schedule: fixed transport routes, 4 certified hover positions.

This stage keeps the frozen Q2 K=22 route/box/type/aircraft/battery structure,
but re-optimizes route start times jointly with the four minimum-cover relay
hover missions.  Each relay-required block is assigned to one of the four
strictly certified hover positions that covers it.

Certified here:
- 80/80 exact transport boxes, no duplicates;
- hard-deadline latest-start constraints;
- concrete transport-aircraft and battery no-overlap calendars;
- every relay-required block assigned to a certified hover point;
- 4 relay missions, at most 2 concurrent R aircraft;
- relay prepare/outbound/link/service/return/turn timing;
- relay energy reserve for the whole mission service window.

A separate final continuous-interval replay remains required before Q3 is
frozen for Q4.
"""
from __future__ import annotations
import argparse, ast, json, math
from pathlib import Path
from collections import defaultdict
import numpy as np
import pandas as pd
from ortools.sat.python import cp_model

from minimal_pipeline import load_inputs, line_geometry
from q3_official_semantics import relay_leg_time_energy

SCALE=10
TOL=1e-6

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return [str(x) for x in json.loads(str(v))]
    except Exception: return [str(x) for x in ast.literal_eval(str(v))]

def parse_map(v):
    if isinstance(v,dict): return {str(k):float(x) for k,x in v.items()}
    try: return {str(k):float(x) for k,x in json.loads(str(v)).items()}
    except Exception: return {str(k):float(x) for k,x in ast.literal_eval(str(v)).items()}

def I(x): return int(round(float(x)*SCALE))
def F(x): return float(x)/SCALE

def peak(intervals):
    pts=[]
    for a,b in intervals:
        if b>a+TOL: pts += [(float(a),1),(float(b),-1)]
    cur=pk=0
    for _,d in sorted(pts,key=lambda z:(z[0],z[1])):
        cur+=d; pk=max(pk,cur)
    return pk

def no_overlap(df,resource_col,start_col,end_col):
    for _,g in df.groupby(resource_col):
        z=sorted([(float(a),float(b)) for a,b in zip(g[start_col],g[end_col])])
        for (_,b),(c,_) in zip(z,z[1:]):
            if c < b-TOL: return False
    return True

def solve_phase(model,obj,limit_s=120):
    model.Minimize(obj)
    s=cp_model.CpSolver()
    s.parameters.max_time_in_seconds=float(limit_s)
    s.parameters.num_search_workers=8
    s.parameters.log_search_progress=True
    st=s.Solve(model)
    return s,st

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True)
    ap.add_argument("--hypergraph",required=True)
    ap.add_argument("--out",default="results/q3_global")
    args=ap.parse_args()
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    data=load_inputs()
    sch=pd.read_csv(args.schedule).reset_index(drop=True)
    hg=Path(args.hypergraph)
    blocks=pd.read_csv(hg/"block_diagnostics.csv")
    cover=pd.read_csv(hg/"minimum_cover_positions.csv")
    if len(sch)!=22 or len(cover)!=4:
        raise RuntimeError(f"expected K22 and 4 hover positions, got routes={len(sch)}, hover={len(cover)}")

    # authoritative 80-box check
    allids=[b for z in sch.box_ids for b in parse_ids(z)]
    official={str(x["货箱编号"]) for x in data["boxes"]}
    if len(allids)!=80 or set(allids)!=official or len(set(allids))!=80:
        raise RuntimeError("K22 schedule is not an exact 80-box cover")

    # map block -> eligible minimum-cover positions
    pos_rows=[]
    elig=defaultdict(list)
    for pidx,r in cover.reset_index(drop=True).iterrows():
        pid=f"P{pidx+1}"
        cov=set(str(r.covered_blocks).split("|"))
        for b in cov: elig[b].append(pidx)
        pos_rows.append({
            "pidx":pidx,"position_id":pid,"lon":float(r.lon),"lat":float(r.lat),
            "agl_m":float(r.agl_m),"altitude_m":float(r.altitude_m),
            "source":str(r.source),"covered_block_count":int(r.covered_block_count)
        })
    missing=[str(b) for b in blocks.block_id if str(b) not in elig]
    if missing: raise RuntimeError(f"minimum cover does not cover blocks: {missing[:10]}")

    # precompute relay travel/energy constants
    c=data["center"]; rdata=data["relay"]
    for p in pos_rows:
        q={"经度（°）":p["lon"],"纬度（°）":p["lat"]}
        dist,terrain,*_=line_geometry(c,q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(c["海拔（m）"]),p["altitude_m"])
        tback,eback=relay_leg_time_energy(data,dist,terrain,p["altitude_m"],float(c["海拔（m）"]))
        p.update({"outbound_s":float(tout),"return_s":float(tback),
                  "base_energy_kwh":float(eout+eback)})

    m=cp_model.CpModel()
    H=I(14000.0)
    starts=[]; returns=[]; air_int=defaultdict(list); bat_int=defaultdict(list)
    for i,row in sch.iterrows():
        sv=m.NewIntVar(0,H,f"route_start_{i}")
        dur=I(row.duration_s)
        ev=m.NewIntVar(0,H+dur,f"route_return_{i}")
        m.Add(ev==sv+dur)
        starts.append(sv); returns.append(ev)
        if pd.notna(row.latest_start_s):
            m.Add(sv<=int(math.floor(float(row.latest_start_s)*SCALE+1e-9)))
        air_int[str(row.aircraft)].append(m.NewIntervalVar(sv,dur,ev,f"air_{i}"))
        bdur=I(float(row.battery_end_s)-float(row.start_s))
        bev=m.NewIntVar(0,H+bdur,f"bat_end_{i}")
        m.Add(bev==sv+bdur)
        bat_int[str(row.battery)].append(m.NewIntervalVar(sv,bdur,bev,f"bat_{i}"))
    for ints in air_int.values(): m.AddNoOverlap(ints)
    for ints in bat_int.values(): m.AddNoOverlap(ints)

    # ordinary soft lateness objective; hard boxes are already represented by latest_start.
    bm={str(x["货箱编号"]):x for x in data["boxes"]}
    late_vars=[]
    for i,row in sch.iterrows():
        offs=parse_map(row.delivery_offsets)
        for bid in parse_ids(row.box_ids):
            b=bm[bid]
            if str(b.get("物资类型",""))=="医疗物资": continue
            due=I(float(b["期望送达时间（s）"]))
            lv=m.NewIntVar(0,H*2,f"late_{i}_{bid}")
            m.Add(lv>=starts[i]+I(offs[bid])-due)
            late_vars.append(lv)
    total_late=m.NewIntVar(0,H*max(1,len(late_vars))*2,"total_late")
    m.Add(total_late==sum(late_vars) if late_vars else 0)

    # block assignments and absolute intervals
    y={}
    block_abs_start=[]; block_abs_end=[]
    for bi,b in blocks.reset_index(drop=True).iterrows():
        ri=int(b.route_index)
        aoff=I(b.start_s); boff=I(b.end_s)
        av=m.NewIntVar(0,H+boff,f"block_start_{bi}")
        bv=m.NewIntVar(0,H+boff,f"block_end_{bi}")
        m.Add(av==starts[ri]+aoff); m.Add(bv==starts[ri]+boff)
        block_abs_start.append(av); block_abs_end.append(bv)
        opts=[]
        for pidx in elig[str(b.block_id)]:
            z=m.NewBoolVar(f"assign_b{bi}_p{pidx}")
            y[bi,pidx]=z; opts.append(z)
        m.Add(sum(opts)==1)

    service_start=[]; service_end=[]; service_dur=[]; relay_busy=[]
    hover_power=float(rdata["hover_power_kw"])+float(rdata["comm_power_kw"])
    usable=(1-float(rdata["reserve"]))*float(rdata["energy_kwh"])
    for pidx,p in enumerate(pos_rows):
        starts_aux=[]; ends_aux=[]; assigned=[]
        for bi,b in blocks.reset_index(drop=True).iterrows():
            if (bi,pidx) not in y: continue
            z=y[bi,pidx]; assigned.append(z)
            aa=m.NewIntVar(0,H*2,f"p{pidx}_b{bi}_minaux")
            ee=m.NewIntVar(0,H*2,f"p{pidx}_b{bi}_maxaux")
            m.Add(aa==block_abs_start[bi]).OnlyEnforceIf(z)
            m.Add(aa==H*2).OnlyEnforceIf(z.Not())
            m.Add(ee==block_abs_end[bi]).OnlyEnforceIf(z)
            m.Add(ee==0).OnlyEnforceIf(z.Not())
            starts_aux.append(aa); ends_aux.append(ee)
        # all four positions belong to the proven minimum cover, require use
        m.Add(sum(assigned)>=1)
        ss=m.NewIntVar(0,H*2,f"relay_service_start_{pidx}")
        se=m.NewIntVar(0,H*2,f"relay_service_end_{pidx}")
        m.AddMinEquality(ss,starts_aux); m.AddMaxEquality(se,ends_aux)
        sd=m.NewIntVar(0,H*2,f"relay_service_dur_{pidx}"); m.Add(sd==se-ss)
        service_start.append(ss); service_end.append(se); service_dur.append(sd)

        lead=I(float(rdata["prep_s"])+float(p["outbound_s"])+float(rdata.get("link_s",0.0)))
        tail=I(float(p["return_s"])+float(rdata["turn_s"]))
        rs=m.NewIntVar(0,H*2,f"relay_busy_start_{pidx}")
        re=m.NewIntVar(0,H*2,f"relay_busy_end_{pidx}")
        m.Add(rs==ss-lead); m.Add(re==se+tail)
        rd=m.NewIntVar(0,H*2,f"relay_busy_dur_{pidx}"); m.Add(rd==re-rs)
        relay_busy.append(m.NewIntervalVar(rs,rd,re,f"relay_mission_{pidx}"))
        p["lead_s"]=F(lead); p["tail_s"]=F(tail)
        # reserve bound converted to a maximum hover/service duration
        if hover_power>0:
            max_service=max(0.0,(usable-float(p["base_energy_kwh"]))*3600.0/hover_power)
            m.Add(sd<=int(math.floor(max_service*SCALE+1e-9)))
            p["max_service_s"]=max_service
        else: p["max_service_s"]=1e9
    m.AddCumulative(relay_busy,[1]*len(relay_busy),2)

    tr_cmax=m.NewIntVar(0,H*2,"transport_cmax"); m.AddMaxEquality(tr_cmax,returns)
    relay_ends=[]
    # recover interval end vars via relation service_end + tail
    for pidx,p in enumerate(pos_rows):
        z=m.NewIntVar(0,H*2,f"relay_end_obj_{pidx}")
        m.Add(z==service_end[pidx]+I(p["tail_s"])); relay_ends.append(z)
    joint_cmax=m.NewIntVar(0,H*2,"joint_cmax")
    m.AddMaxEquality(joint_cmax,returns+relay_ends)
    total_service=m.NewIntVar(0,H*8,"total_relay_service"); m.Add(total_service==sum(service_dur))

    # phase 1: minimize soft lateness
    sol,st=solve_phase(m,total_late,150)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase1 infeasible")
    best_late=sol.Value(total_late); m.Add(total_late==best_late)
    # phase 2: joint completion time
    sol,st=solve_phase(m,joint_cmax,150)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase2 infeasible")
    best_joint=sol.Value(joint_cmax); m.Add(joint_cmax==best_joint)
    # phase 3: relay service span / energy
    sol,st=solve_phase(m,total_service,150)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase3 infeasible")

    # reconstruct transport
    sr=[]
    for i,row in sch.iterrows():
        z=row.to_dict(); z["q2_original_start_s"]=float(row.start_s)
        z["start_s"]=F(sol.Value(starts[i])); z["return_s"]=F(sol.Value(returns[i]))
        z["start_shift_s"]=z["start_s"]-z["q2_original_start_s"]
        sr.append(z)
    sdf=pd.DataFrame(sr)
    sdf.to_csv(out/"transport_schedule.csv",index=False,encoding="utf-8-sig")

    # block assignments
    br=[]
    for bi,b in blocks.reset_index(drop=True).iterrows():
        pidx=next(p for p in elig[str(b.block_id)] if sol.Value(y[bi,p])==1)
        br.append({**b.to_dict(),"assigned_position_id":pos_rows[pidx]["position_id"],
                   "absolute_start_s":F(sol.Value(block_abs_start[bi])),
                   "absolute_end_s":F(sol.Value(block_abs_end[bi]))})
    bdf=pd.DataFrame(br); bdf.to_csv(out/"communication_block_assignments.csv",index=False,encoding="utf-8-sig")

    # relay missions, greedy color 2
    missions=[]
    for pidx,p in enumerate(pos_rows):
        ss=F(sol.Value(service_start[pidx])); se=F(sol.Value(service_end[pidx]))
        busy_start=ss-p["lead_s"]; busy_end=se+p["tail_s"]
        dur=se-ss
        energy=float(p["base_energy_kwh"])+hover_power*dur/3600.0
        missions.append({**p,"service_start_s":ss,"service_end_s":se,"service_duration_s":dur,
                         "busy_start_s":busy_start,"busy_end_s":busy_end,
                         "relay_energy_kwh":energy,"usable_energy_kwh":usable,
                         "reserve_ok":energy<=usable+TOL})
    missions.sort(key=lambda z:z["busy_start_s"])
    avail={"R01":0.0,"R02":0.0}
    for z in missions:
        rid=min(avail,key=lambda k:avail[k])
        if z["busy_start_s"] < avail[rid]-TOL:
            # try other relay
            other="R02" if rid=="R01" else "R01"
            if z["busy_start_s"] >= avail[other]-TOL: rid=other
        z["relay_id"]=rid; avail[rid]=z["busy_end_s"]
    mdf=pd.DataFrame(missions); mdf.to_csv(out/"relay_missions.csv",index=False,encoding="utf-8-sig")

    # delivery audit
    drows=[]
    for i,row in sdf.iterrows():
        offs=parse_map(row.delivery_offsets)
        for bid in parse_ids(row.box_ids):
            b=bm[bid]; t=float(row.start_s)+offs[bid]
            hard=[]
            if str(b.get("物资类型",""))=="医疗物资": hard.append(float(b["期望送达时间（s）"]))
            if str(b.get("是否首批保障",""))=="是": hard.append(float(b["首批截止时间（s）"]))
            hard_bad=any(t>x+TOL for x in hard)
            due=float(b["期望送达时间（s）"])
            late=max(0.0,t-due) if str(b.get("物资类型",""))!="医疗物资" else 0.0
            drows.append({"box_id":bid,"route_index":i,"delivery_s":t,"hard_violation":hard_bad,
                          "soft_lateness_s":late})
    ddf=pd.DataFrame(drows); ddf.to_csv(out/"deliveries.csv",index=False,encoding="utf-8-sig")

    relay_peak=peak([(x["busy_start_s"],x["busy_end_s"]) for x in missions])
    summary={
      "status":"GLOBAL_BLOCK_LEVEL_CERTIFIED_NEEDS_FINAL_CONTINUOUS_REPLAY",
      "q3_global_certified":False,
      "transport_routes":22,"boxes":len(ddf),"unique_boxes":int(ddf.box_id.nunique()),
      "hard_violation_boxes":int(ddf.hard_violation.sum()),
      "soft_late_boxes":int((ddf.soft_lateness_s>TOL).sum()),
      "soft_total_lateness_s":float(ddf.soft_lateness_s.sum()),
      "transport_makespan_s":float(sdf.return_s.max()),
      "relay_missions":4,"relay_peak":int(relay_peak),
      "relay_energy_all_reserve_ok":bool(mdf.reserve_ok.all()),
      "communication_blocks":int(len(bdf)),
      "communication_blocks_assigned":int(len(bdf)),
      "all_assignments_from_certified_cover":True,
      "aircraft_no_overlap":bool(no_overlap(sdf,"aircraft","start_s","return_s")),
      "battery_no_overlap":bool(no_overlap(sdf,"battery","start_s","battery_end_s")) if False else True,
      "joint_completion_s":F(sol.Value(joint_cmax)),
      "phase1_total_lateness_ds":int(best_late),
      "phase2_joint_cmax_ds":int(best_joint),
      "phase3_total_relay_service_ds":int(sol.Value(total_service)),
      "final_continuous_interval_replay_pass":False,
      "scope_note":"All route blocks are assigned to strict hypergraph cover positions and resources are globally scheduled. Final recursive continuous-interval communication replay is the remaining Q3 gate."
    }
    # Battery end after shifted starts is recomputed from the fixed relative occupancy.
    sdf2=pd.read_csv(out/"transport_schedule.csv")
    sdf2["battery_busy_end_s"]=[float(r.start_s)+(float(orig.battery_end_s)-float(orig.start_s)) for (_,r),(_,orig) in zip(sdf2.iterrows(),sch.iterrows())]
    summary["battery_no_overlap"]=bool(no_overlap(sdf2,"battery","start_s","battery_busy_end_s"))
    summary["block_level_global_pass"]=bool(
        summary["boxes"]==80 and summary["unique_boxes"]==80 and summary["hard_violation_boxes"]==0 and
        summary["relay_peak"]<=2 and summary["relay_energy_all_reserve_ok"] and
        summary["aircraft_no_overlap"] and summary["battery_no_overlap"]
    )
    (out/"q3_global_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
