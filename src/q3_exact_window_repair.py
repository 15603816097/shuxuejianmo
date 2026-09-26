"""Repair sub-0.1s relay-window rounding on JOINT-18x5 and re-certify.

The CP-SAT screens use 0.1 s integer time. Final continuous replay uses exact
floating-point trajectory times. Expand each relay mission service window to
the exact envelope of its assigned blocks, then recompute busy times, relay
energy, component recharge, and verify R=2/component resources before invoking
final continuous certification.
"""
from __future__ import annotations
import ast, json
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs, line_geometry
from q3_semantics_core import build_authoritative_timeline, direct_and_relay_blocks
from q3_official_semantics import relay_leg_time_energy, component_charge_time

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try:return [str(x) for x in json.loads(str(v))]
    except Exception:return [str(x) for x in ast.literal_eval(str(v))]

def peak(intervals):
    ev=[]
    for a,b in intervals: ev += [(float(a),1),(float(b),-1)]
    cur=best=0
    for _,d in sorted(ev,key=lambda x:(x[0],x[1])):
        cur+=d; best=max(best,cur)
    return best

def main(base,outdir):
    base=Path(base); out=Path(outdir); out.mkdir(parents=True,exist_ok=True)
    data=load_inputs()
    tr=pd.read_csv(base/"transport_schedule.csv").reset_index(drop=True)
    missions=pd.read_csv(base/"relay_missions.csv").copy()
    ba=pd.read_csv(base/"block_assignments.csv").copy()

    exact={}
    for ri,r in tr.iterrows():
        tl=build_authoritative_timeline(data,(str(r.service),),str(r.drone_type),parse_ids(r.box_ids))
        blocks=direct_and_relay_blocks(data,tl)
        # assignment rows preserve one row per strict block and route
        rows=ba[ba.route_index.astype(int)==ri].copy()
        # order by the original block start from materialized output
        rows=rows.sort_values("block_start_s").reset_index(drop=True)
        if len(rows)!=len(blocks):
            raise RuntimeError(f"route {ri} block mismatch {len(rows)} != {len(blocks)}")
        for j,b in enumerate(blocks):
            bid=str(rows.iloc[j].block_id)
            exact[bid]=(float(r.start_s)+float(b["start_s"]), float(r.start_s)+float(b["end_s"]))

    for i,a in ba.iterrows():
        s,e=exact[str(a.block_id)]
        ba.at[i,"block_start_s"]=s; ba.at[i,"block_end_s"]=e

    r=data["relay"]; hover=float(r["hover_power_kw"])+float(r["comm_power_kw"])
    for i,m in missions.iterrows():
        assigned=ba[ba.mission_id.astype(str)==str(m.mission_id)]
        ss=float(assigned.block_start_s.min()); ee=float(assigned.block_end_s.max())
        q={"经度（°）":float(m.lon),"纬度（°）":float(m.lat)}
        dist,terrain,*_=line_geometry(data["center"],q,data["dem"])
        tout,eout=relay_leg_time_energy(data,dist,terrain,float(data["center"]["海拔（m）"]),float(m.altitude_m))
        tback,eback=relay_leg_time_energy(data,dist,terrain,float(m.altitude_m),float(data["center"]["海拔（m）"]))
        lead=float(r["prep_s"])+tout+float(r["link_s"]); tail=tback+float(r["turn_s"])
        busy_s=ss-lead; busy_e=ee+tail
        if busy_s < -1e-9: raise RuntimeError("negative relay launch time after exact expansion")
        energy=eout+eback+hover*(ee-ss)/3600.0
        soc=1-energy/float(r["energy_kwh"])
        if soc < float(r["reserve"])-1e-9: raise RuntimeError("relay reserve violated after exact expansion")
        missions.loc[i,["service_start_s","service_end_s","busy_start_s","busy_end_s",
                        "service_duration_s","energy_kwh","soc_end"]] = [ss,ee,busy_s,busy_e,ee-ss,energy,soc]

    # Re-color R01/R02 after exact expansion.
    rav={"R01":0.0,"R02":0.0}
    for idx in missions.sort_values("busy_start_s").index:
        rid=min(rav,key=lambda k:(rav[k],k))
        if rav[rid] > float(missions.at[idx,"busy_start_s"])+1e-9:
            raise RuntimeError("R=2 violated after exact expansion")
        missions.at[idx,"relay_id"]=rid; rav[rid]=float(missions.at[idx,"busy_end_s"])

    # Exact component reassignment/recharge.
    cav={f"RE-{i+1:02d}":0.0 for i in range(int(r["component_inventory"]))}
    for idx in missions.sort_values("busy_start_s").index:
        cid=min(cav,key=lambda k:(cav[k],k))
        if cav[cid] > float(missions.at[idx,"busy_start_s"])+1e-9:
            raise RuntimeError("component inventory violated after exact expansion")
        ch=component_charge_time(float(missions.at[idx,"soc_end"]),float(r["component_full_charge_s"]))
        missions.at[idx,"component_id"]=cid
        missions.at[idx,"component_charge_s"]=ch
        missions.at[idx,"component_available_again_s"]=float(missions.at[idx,"busy_end_s"])+ch
        cav[cid]=float(missions.at[idx,"component_available_again_s"])

    summary={
      "relay_peak":peak([(x.busy_start_s,x.busy_end_s) for _,x in missions.iterrows()]),
      "component_peak":peak([(x.busy_start_s,x.component_available_again_s) for _,x in missions.iterrows()]),
      "relay_energy_kwh":float(missions.energy_kwh.sum()),
      "joint_energy_kwh":float(tr.transport_energy_kwh.sum()+missions.energy_kwh.sum()),
      "joint_completion_s":float(max(tr.return_s.max(),missions.busy_end_s.max())),
      "max_service_expansion_s":float(max(
          max(0.0,float(old.service_start_s)-float(new.service_start_s))+
          max(0.0,float(new.service_end_s)-float(old.service_end_s))
          for (_,old),(_,new) in zip(pd.read_csv(base/"relay_missions.csv").iterrows(),missions.iterrows())
      ))
    }
    tr.to_csv(out/"transport_schedule.csv",index=False,encoding="utf-8-sig")
    missions.to_csv(out/"relay_missions.csv",index=False,encoding="utf-8-sig")
    ba.to_csv(out/"block_assignments.csv",index=False,encoding="utf-8-sig")
    (out/"repair_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument("--base",required=True); ap.add_argument("--out",required=True)
    a=ap.parse_args(); main(a.base,a.out)
