"""Final recursive continuous-communication certification for JOINT-18x5.

Rebuild each authoritative transport trajectory, apply direct-first communication,
and where direct is unavailable require the same assigned relay mission over the
whole strict relay block. A tri-state interval test recursively subdivides mixed
communication intervals to <=0.1 s, matching the friend-paper certification
logic. All hard deadlines, resource peaks, component ledgers, 80-box exact cover,
and final energy/makespan are re-audited before q3_global_certified can be true.
"""
from __future__ import annotations
import ast, json, math
from pathlib import Path
import numpy as np
import pandas as pd

from minimal_pipeline import load_inputs, terrain_profile
from q3_semantics_core import build_authoritative_timeline, direct_and_relay_blocks, deadline_ledger, xyz, link_obstructed
from q3_five_service_hover_refinement import thresholds

EPS=1e-9
MAX_DT=0.1

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return [str(x) for x in json.loads(str(v))]
    except Exception: return [str(x) for x in ast.literal_eval(str(v))]

def loss_margin(data,a,b,ha,hb,thr):
    freq=float(data["comm"][("传播参数","载波频率（MHz）")])
    obs=float(data["comm"][("传播参数","地形遮挡附加损耗（dB）")])
    d=float(np.linalg.norm(xyz(data,a,ha)-xyz(data,b,hb)))
    blocked=link_obstructed(data,a,b,ha,hb)
    loss=32.45+20*math.log10(freq)+20*math.log10(max(d/1000.0,1e-12))+(obs if blocked else 0.0)
    return thr-loss

def interp(seg,u):
    a,b=seg["a"],seg["b"]
    p={"经度（°）":float(a["经度（°）"])+u*(float(b["经度（°）"])-float(a["经度（°）"])),
       "纬度（°）":float(a["纬度（°）"])+u*(float(b["纬度（°）"])-float(a["纬度（°）"]))}
    h=float(seg["ha"])+u*(float(seg["hb"])-float(seg["ha"]))
    return p,h

def status_at(data,seg,u,relay_point=None,relay_h=None):
    td,ta,tb=thresholds(data)
    p,h=interp(seg,u); c=data["center"]
    gh=float(c["海拔（m）"])+float(data["comm"][("固定网关 G01","天线离地高度（m）")])
    dm=loss_margin(data,p,c,h,gh,td)
    if dm>=-1e-9:
        return True,"DIRECT",dm,float("inf"),float("inf")
    if relay_point is None:
        return False,"NO_RELAY",dm,float("-inf"),float("-inf")
    am=loss_margin(data,p,relay_point,h,relay_h,ta)
    bm=loss_margin(data,relay_point,c,relay_h,gh,tb)
    ok=am>=-1e-9 and bm>=-1e-9
    return ok,"RELAY" if ok else "RELAY_FAIL",dm,am,bm

def recurse_cert(data,seg,relay_point,relay_h,u0=0.0,u1=1.0,depth=0):
    t0=float(seg["start_s"])+(float(seg["end_s"])-float(seg["start_s"]))*u0
    t1=float(seg["start_s"])+(float(seg["end_s"])-float(seg["start_s"]))*u1
    um=(u0+u1)/2
    vals=[status_at(data,seg,u,relay_point,relay_h) for u in (u0,um,u1)]
    oks=[v[0] for v in vals]
    mins={"direct":min(v[2] for v in vals),"access":min(v[3] for v in vals),"backhaul":min(v[4] for v in vals)}
    modes=set(v[1] for v in vals)
    if all(oks) and len(modes)==1:
        return True,1,mins,modes
    if (t1-t0)<=MAX_DT+1e-12:
        return all(oks),1,mins,modes
    # Mixed or failing: recurse until 0.1s.
    ok1,n1,m1,md1=recurse_cert(data,seg,relay_point,relay_h,u0,um,depth+1)
    ok2,n2,m2,md2=recurse_cert(data,seg,relay_point,relay_h,um,u1,depth+1)
    mm={k:min(m1[k],m2[k]) for k in mins}
    return ok1 and ok2,n1+n2,mm,md1|md2

def peak(intervals):
    ev=[]
    for a,b in intervals:
        ev.append((float(a),1)); ev.append((float(b),-1))
    cur=best=0
    for _,d in sorted(ev,key=lambda x:(x[0],x[1])):
        cur+=d; best=max(best,cur)
    return best

def main(base,outdir):
    base=Path(base); out=Path(outdir); out.mkdir(parents=True,exist_ok=True)
    data=load_inputs()
    tr=pd.read_csv(base/"transport_schedule.csv").reset_index(drop=True)
    missions=pd.read_csv(base/"relay_missions.csv")
    ba=pd.read_csv(base/"block_assignments.csv")
    hg=Path(base).parent/"q3_joint_grouping_hg"/"JOINT"
    # workflow may stage hypergraph separately; allow explicit sibling copied into base
    if not (hg/"block_diagnostics.csv").exists():
        hg=base/"hypergraph"
    bd=pd.read_csv(hg/"block_diagnostics.csv").reset_index(drop=True)

    mby={str(r.mission_id):r for _,r in missions.iterrows()}
    aby={str(r.block_id):r for _,r in ba.iterrows()}
    cert_rows=[]; deadline_rows=[]; all_box=[]; route_intervals={k:[] for k in data["aircraft"]}
    battery_intervals={k:[] for k in data["batteries"]}

    for ri,r in tr.iterrows():
        ids=parse_ids(r.box_ids); all_box+=ids
        tl=build_authoritative_timeline(data,(str(r.service),),str(r.drone_type),ids)
        start=float(r.start_s)
        for d in deadline_ledger(data,tl):
            q=dict(d); q["delivery_time_s"]=float(q["delivery_time_s"])+start
            q["pass"]=bool(q["delivery_time_s"]<=float(q["deadline_s"])+1e-9) if q["hard"] else True
            q["route_index"]=ri; deadline_rows.append(q)
        route_intervals[str(r.drone_type)].append((start,start+float(r.duration_s)))
        battery_intervals[str(r.drone_type)].append((start,start+float(r.duration_s)+float(r.charge_s)))

        blocks=direct_and_relay_blocks(data,tl)
        rows=bd[bd.route_index.astype(int)==ri].sort_values("start_s").reset_index(drop=True)
        if len(rows)!=len(blocks):
            raise RuntimeError(f"route {ri}: block count mismatch {len(rows)} != {len(blocks)}")
        block_map={}
        for j,b in enumerate(blocks):
            bid=str(rows.iloc[j].block_id); ar=aby[bid]; mr=mby[str(ar.mission_id)]
            rp={"经度（°）":float(mr.lon),"纬度（°）":float(mr.lat)}
            block_map[j]=(b,bid,mr,rp)

        # audit every flight phase; if it belongs to a relay-required block use the same relay for entire phase/block
        for ev in [x for x in tl["events"] if x.get("event")=="flight"]:
            seg=dict(ev); seg["start_s"]=float(seg["start_s"])+start; seg["end_s"]=float(seg["end_s"])+start
            match=None
            for j,(b,bid,mr,rp) in block_map.items():
                # q3 blocks currently correspond to phase events; match relative endpoints.
                if abs(float(ev["start_s"])-float(b["start_s"]))<1e-6 and abs(float(ev["end_s"])-float(b["end_s"]))<1e-6:
                    match=(bid,mr,rp); break
            if match is None:
                ok,n,mins,modes=recurse_cert(data,seg,None,None)
                cert_rows.append({"route_index":ri,"block_id":"","phase":ev["phase"],"start_s":seg["start_s"],"end_s":seg["end_s"],
                                  "relay_id":"","mission_id":"","pass":ok,"leaf_intervals":n,**{f"min_{k}_margin_db":v for k,v in mins.items()},
                                  "modes":"|".join(sorted(modes))})
            else:
                bid,mr,rp=match
                # same relay mission must cover entire absolute block
                chain=(float(mr.service_start_s)<=seg["start_s"]+1e-6 and float(mr.service_end_s)>=seg["end_s"]-1e-6)
                ok,n,mins,modes=recurse_cert(data,seg,rp,float(mr.altitude_m))
                ok=bool(ok and chain)
                cert_rows.append({"route_index":ri,"block_id":bid,"phase":ev["phase"],"start_s":seg["start_s"],"end_s":seg["end_s"],
                                  "relay_id":str(mr.relay_id),"mission_id":str(mr.mission_id),"pass":ok,"leaf_intervals":n,**{f"min_{k}_margin_db":v for k,v in mins.items()},
                                  "modes":"|".join(sorted(modes))})

    cert=pd.DataFrame(cert_rows); dl=pd.DataFrame(deadline_rows)
    transport_peak={typ:peak(v) for typ,v in route_intervals.items()}
    battery_peak={typ:peak(v) for typ,v in battery_intervals.items()}
    transport_resource_ok=all(transport_peak[t]<=len(data["aircraft"][t]) for t in transport_peak)
    battery_resource_ok=all(battery_peak[t]<=int(data["batteries"][t]) for t in battery_peak)
    relay_peak=peak([(r.busy_start_s,r.busy_end_s) for _,r in missions.iterrows()])
    relay_resource_ok=relay_peak<=len(data["relay"]["aircraft"])
    comp_peak=peak([(r.busy_start_s,r.component_available_again_s) for _,r in missions.iterrows()])
    comp_ok=comp_peak<=int(data["relay"]["component_inventory"]) and bool((missions.soc_end>=float(data["relay"]["reserve"])-1e-9).all())
    hard_ok=bool(dl[dl.hard==True]["pass"].all())
    exact80=(len(all_box)==80 and len(set(all_box))==80)
    comm_ok=bool(cert["pass"].all())
    min_margin=float(min(cert["min_direct_margin_db"].min(),
                         cert["min_access_margin_db"].replace([np.inf,-np.inf],np.nan).min(skipna=True),
                         cert["min_backhaul_margin_db"].replace([np.inf,-np.inf],np.nan).min(skipna=True)))
    soft=dl[dl.hard==False].copy()
    soft["lateness_s"]=(soft["delivery_time_s"]-soft["deadline_s"]).clip(lower=0)
    soft_violation_boxes=int((soft["lateness_s"]>1e-9).sum())
    total_soft_late=float(soft["lateness_s"].sum())

    joint_energy=float(tr.transport_energy_kwh.sum()+missions.energy_kwh.sum())
    joint_completion=float(max(tr.return_s.max(),missions.busy_end_s.max()))
    final=bool(exact80 and hard_ok and transport_resource_ok and battery_resource_ok and relay_resource_ok and comp_ok and comm_ok)
    summary={
      "q3_global_certified":final,
      "scope":"current JOINT-18 candidate; not a proof of global route-space optimality",
      "transport_routes":int(len(tr)),"relay_sorties":int(len(missions)),"boxes":len(all_box),"unique_boxes":len(set(all_box)),
      "hard_deadline_pass":hard_ok,"soft_violation_boxes":soft_violation_boxes,"total_soft_lateness_s":total_soft_late,
      "continuous_communication_pass":comm_ok,"continuous_leaf_intervals":int(cert.leaf_intervals.sum()),
      "minimum_sampled_margin_db":min_margin,
      "transport_aircraft_peak":transport_peak,"transport_battery_peak":battery_peak,
      "relay_peak":relay_peak,"component_peak":comp_peak,
      "transport_resource_pass":transport_resource_ok,"battery_resource_pass":battery_resource_ok,
      "relay_resource_pass":relay_resource_ok,"component_pass":comp_ok,
      "transport_energy_kwh":float(tr.transport_energy_kwh.sum()),"relay_energy_kwh":float(missions.energy_kwh.sum()),
      "joint_energy_kwh":joint_energy,"joint_completion_s":joint_completion
    }
    cert.to_csv(out/"continuous_communication_certificate.csv",index=False,encoding="utf-8-sig")
    dl.to_csv(out/"deadline_ledger.csv",index=False,encoding="utf-8-sig")
    tr.to_csv(out/"transport_schedule.csv",index=False,encoding="utf-8-sig")
    missions.to_csv(out/"relay_missions.csv",index=False,encoding="utf-8-sig")
    ba.to_csv(out/"block_assignments.csv",index=False,encoding="utf-8-sig")
    (out/"q3_global_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument("--base",required=True); ap.add_argument("--out",default="results/q3_official")
    a=ap.parse_args(); main(a.base,a.out)
