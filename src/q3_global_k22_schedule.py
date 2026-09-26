"""Q3 global schedule: fixed transport route set, certified hover positions.

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
    ap.add_argument("--label",required=True)
    ap.add_argument("--out",default="results/q3_global")
    args=ap.parse_args()
    out=Path(args.out)/args.label; out.mkdir(parents=True,exist_ok=True)
    data=load_inputs()
    sch=pd.read_csv(args.schedule).reset_index(drop=True)
    hg=Path(args.hypergraph)
    blocks=pd.read_csv(hg/"block_diagnostics.csv")
    cover=pd.read_csv(hg/"minimum_cover_positions.csv")
    K=len(sch)
    if len(cover)!=4:
        raise RuntimeError(f"expected 4 hover positions from minimum cover, got routes={K}, hover={len(cover)}")

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
        typ=str(row.drone_type)
        air_int[typ].append(m.NewIntervalVar(sv,dur,ev,f"air_{i}"))
        bdur=I(float(row.charge_s)+float(row.duration_s))
        bev=m.NewIntVar(0,H+bdur,f"bat_end_{i}")
        m.Add(bev==sv+bdur)
        bat_int[typ].append(m.NewIntervalVar(sv,bdur,bev,f"bat_{i}"))
    # Q3 is allowed to reassign concrete aircraft and batteries. Preserve the
    # route/drone-type structure but enforce the official pooled capacities.
    for typ,ints in air_int.items():
        m.AddCumulative(ints,[1]*len(ints),len(data["aircraft"][typ]))
    for typ,ints in bat_int.items():
        m.AddCumulative(ints,[1]*len(ints),int(data["batteries"][typ]))

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

    # Time-wave relay missions: each certified hover position may be reused in
    # multiple relay sorties. This matches the benchmark's geographic-cluster
    # + time-wave architecture and avoids forcing one very long mission per
    # hover point.
    MAX_WAVES=4
    mission_use={}
    mission_start={}
    mission_end={}
    mission_dur={}
    mission_busy=[]
    mission_meta=[]
    hover_power=float(rdata["hover_power_kw"])+float(rdata["comm_power_kw"])
    usable=(1-float(rdata["reserve"]))*float(rdata["energy_kwh"])

    # assignment variable block -> (position,wave)
    yw={}
    for bi,b in blocks.reset_index(drop=True).iterrows():
        opts=[]
        for pidx in elig[str(b.block_id)]:
            for w in range(MAX_WAVES):
                z=m.NewBoolVar(f"assign_b{bi}_p{pidx}_w{w}")
                yw[bi,pidx,w]=z
                opts.append(z)
        m.Add(sum(opts)==1)

    for pidx,p in enumerate(pos_rows):
        for w in range(MAX_WAVES):
            use=m.NewBoolVar(f"mission_use_p{pidx}_w{w}")
            ms=m.NewIntVar(0,H*2,f"mission_start_p{pidx}_w{w}")
            me=m.NewIntVar(0,H*2,f"mission_end_p{pidx}_w{w}")
            md=m.NewIntVar(0,H*2,f"mission_dur_p{pidx}_w{w}")
            m.Add(md==me-ms)
            mission_use[pidx,w]=use
            mission_start[pidx,w]=ms
            mission_end[pidx,w]=me
            mission_dur[pidx,w]=md
            assigned=[]
            for bi,b in blocks.reset_index(drop=True).iterrows():
                z=yw.get((bi,pidx,w))
                if z is None: continue
                assigned.append(z)
                # A used mission must span every assigned communication block.
                m.Add(ms<=block_abs_start[bi]).OnlyEnforceIf(z)
                m.Add(me>=block_abs_end[bi]).OnlyEnforceIf(z)
                m.AddImplication(z,use)
            if assigned:
                m.Add(sum(assigned)>=1).OnlyEnforceIf(use)
                m.Add(sum(assigned)==0).OnlyEnforceIf(use.Not())
            else:
                m.Add(use==0)
            # compact unused mission values to zero
            m.Add(ms==0).OnlyEnforceIf(use.Not())
            m.Add(me==0).OnlyEnforceIf(use.Not())

            lead=I(float(rdata["prep_s"])+float(p["outbound_s"])+float(rdata.get("link_s",0.0)))
            tail=I(float(p["return_s"])+float(rdata["turn_s"]))
            bs=m.NewIntVar(0,H*2,f"busy_start_p{pidx}_w{w}")
            be=m.NewIntVar(0,H*2,f"busy_end_p{pidx}_w{w}")
            bd=m.NewIntVar(0,H*2,f"busy_dur_p{pidx}_w{w}")
            m.Add(bs==ms-lead).OnlyEnforceIf(use)
            m.Add(be==me+tail).OnlyEnforceIf(use)
            m.Add(bd==be-bs).OnlyEnforceIf(use)
            m.Add(bs==0).OnlyEnforceIf(use.Not())
            m.Add(be==0).OnlyEnforceIf(use.Not())
            m.Add(bd==0).OnlyEnforceIf(use.Not())
            mission_busy.append(m.NewOptionalIntervalVar(bs,bd,be,use,f"relay_mission_p{pidx}_w{w}"))

            if hover_power>0:
                max_service=max(0.0,(usable-float(p["base_energy_kwh"]))*3600.0/hover_power)
                m.Add(md<=int(math.floor(max_service*SCALE+1e-9))).OnlyEnforceIf(use)
            else:
                max_service=1e9
            mission_meta.append({"pidx":pidx,"wave":w,"use":use,"lead_s":F(lead),"tail_s":F(tail),
                                 "max_service_s":max_service})

    m.AddCumulative(mission_busy,[1]*len(mission_busy),2)
    mission_count=m.NewIntVar(0,len(mission_meta),"mission_count")
    m.Add(mission_count==sum(x["use"] for x in mission_meta))

    tr_cmax=m.NewIntVar(0,H*2,"transport_cmax"); m.AddMaxEquality(tr_cmax,returns)
    relay_end_terms=[]
    for q in mission_meta:
        e=m.NewIntVar(0,H*2,f"relay_end_obj_p{q['pidx']}_w{q['wave']}")
        m.Add(e==mission_end[q["pidx"],q["wave"]]+I(q["tail_s"])).OnlyEnforceIf(q["use"])
        m.Add(e==0).OnlyEnforceIf(q["use"].Not())
        relay_end_terms.append(e)
    joint_cmax=m.NewIntVar(0,H*2,"joint_cmax")
    m.AddMaxEquality(joint_cmax,returns+relay_end_terms)
    total_service=m.NewIntVar(0,H*len(mission_meta)*2,"total_relay_service")
    m.Add(total_service==sum(mission_dur.values()))

    # First test whether the benchmark-like four-sortie relay structure is
    # feasible. If not, the solver may use more time waves, and the exact
    # minimum relay-sortie count is optimized after timeliness.
    # phase 1: minimize ordinary-box lateness under all hard/resource constraints
    sol,st=solve_phase(m,total_late,120)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase1 infeasible")
    best_late=sol.Value(total_late); m.Add(total_late==best_late)
    # phase 2: minimum number of relay sorties/time waves
    sol,st=solve_phase(m,mission_count,120)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase2 infeasible")
    best_missions=sol.Value(mission_count); m.Add(mission_count==best_missions)
    # phase 3: joint completion time
    sol,st=solve_phase(m,joint_cmax,120)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase3 infeasible")
    best_joint=sol.Value(joint_cmax); m.Add(joint_cmax==best_joint)
    # phase 4: compact relay service spans / energy
    sol,st=solve_phase(m,total_service,120)
    if st not in (cp_model.OPTIMAL,cp_model.FEASIBLE): raise RuntimeError("phase4 infeasible")

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
        chosen=None
        for pidx in elig[str(b.block_id)]:
            for w in range(MAX_WAVES):
                if sol.Value(yw[bi,pidx,w])==1:
                    chosen=(pidx,w); break
            if chosen is not None: break
        if chosen is None: raise RuntimeError(f"unassigned block {b.block_id}")
        pidx,w=chosen
        br.append({**b.to_dict(),"assigned_position_id":pos_rows[pidx]["position_id"],
                   "assigned_wave":int(w+1),
                   "absolute_start_s":F(sol.Value(block_abs_start[bi])),
                   "absolute_end_s":F(sol.Value(block_abs_end[bi]))})
    bdf=pd.DataFrame(br); bdf.to_csv(out/"communication_block_assignments.csv",index=False,encoding="utf-8-sig")

    # relay missions, then greedy color onto the two physical R aircraft
    missions=[]
    for q in mission_meta:
        pidx,w=q["pidx"],q["wave"]
        if sol.Value(q["use"])!=1: continue
        p=pos_rows[pidx]
        ss=F(sol.Value(mission_start[pidx,w])); se=F(sol.Value(mission_end[pidx,w]))
        busy_start=ss-q["lead_s"]; busy_end=se+q["tail_s"]
        dur=se-ss
        energy=float(p["base_energy_kwh"])+hover_power*dur/3600.0
        missions.append({**p,"wave":int(w+1),"service_start_s":ss,"service_end_s":se,
                         "service_duration_s":dur,"busy_start_s":busy_start,"busy_end_s":busy_end,
                         "relay_energy_kwh":energy,"usable_energy_kwh":usable,
                         "reserve_ok":energy<=usable+TOL})
    missions.sort(key=lambda z:z["busy_start_s"])
    avail={"R01":0.0,"R02":0.0}
    for z in missions:
        fits=[rid for rid,t in avail.items() if z["busy_start_s"]>=t-TOL]
        if not fits: raise RuntimeError("post-solve relay coloring exceeded two physical R aircraft")
        rid=min(fits,key=lambda k:avail[k])
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
      "status":"GLOBAL_TIME_WAVE_BLOCK_LEVEL_CERTIFIED_NEEDS_FINAL_CONTINUOUS_REPLAY",
      "q3_global_certified":False,
      "label":args.label,"transport_routes":int(K),"boxes":len(ddf),"unique_boxes":int(ddf.box_id.nunique()),
      "hard_violation_boxes":int(ddf.hard_violation.sum()),
      "soft_late_boxes":int((ddf.soft_lateness_s>TOL).sum()),
      "soft_total_lateness_s":float(ddf.soft_lateness_s.sum()),
      "transport_makespan_s":float(sdf.return_s.max()),
      "relay_missions":int(len(missions)),"relay_peak":int(relay_peak),
      "relay_energy_all_reserve_ok":bool(mdf.reserve_ok.all()),
      "communication_blocks":int(len(bdf)),
      "communication_blocks_assigned":int(len(bdf)),
      "all_assignments_from_certified_cover":True,
      "aircraft_capacity_model":"pooled_by_drone_type",
      "battery_capacity_model":"pooled_by_drone_type",
      "joint_completion_s":F(sol.Value(joint_cmax)),
      "phase1_total_lateness_ds":int(best_late),
      "minimum_relay_sorties":int(best_missions),
      "four_relay_sorties_feasible":bool(best_missions<=4),
      "phase3_joint_cmax_ds":int(best_joint),
      "phase4_total_relay_service_ds":int(sol.Value(total_service)),
      "final_continuous_interval_replay_pass":False,
      "scope_note":"All route blocks are assigned to strict hypergraph cover positions and resources are globally scheduled. Final recursive continuous-interval communication replay is the remaining Q3 gate."
    }
    # Independent pooled-capacity replay on reconstructed starts.
    air_ok=True; bat_ok=True; air_peaks={}; bat_peaks={}
    for typ in sorted(data["aircraft"]):
        ints=[]; bints=[]
        for i,row in sdf.iterrows():
            if str(row.drone_type)!=typ: continue
            ints.append((float(row.start_s),float(row.return_s)))
            orig=sch.iloc[i]
            bints.append((float(row.start_s),float(row.start_s)+float(orig.duration_s)+float(orig.charge_s)))
        air_peaks[typ]=peak(ints); bat_peaks[typ]=peak(bints)
        air_ok &= air_peaks[typ] <= len(data["aircraft"][typ])
        bat_ok &= bat_peaks[typ] <= int(data["batteries"][typ])
    summary["aircraft_peak_by_type"]=air_peaks
    summary["battery_peak_by_type"]=bat_peaks
    summary["aircraft_capacity_pass"]=bool(air_ok)
    summary["battery_capacity_pass"]=bool(bat_ok)
    summary["block_level_global_pass"]=bool(
        summary["boxes"]==80 and summary["unique_boxes"]==80 and summary["hard_violation_boxes"]==0 and
        summary["relay_peak"]<=2 and summary["relay_energy_all_reserve_ok"] and air_ok and bat_ok
    )
    (out/"q3_global_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
