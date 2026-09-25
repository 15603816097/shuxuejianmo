"""Unified Q2-Q3-Q4 final gate runner.

The scheduling layer is a deterministic MILP on a 60-second start grid.  Q3
uses exact DEM-cell traversal with the project's piecewise-constant DEM rule;
no fixed-point relay sampling is used for the final certificate.
"""
from __future__ import annotations
import json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp, Bounds, LinearConstraint
from scipy.sparse import lil_matrix
from minimal_pipeline import load_inputs, q1, q2, _segment_cert, node_map, terrain_profile

ROOT = Path(__file__).resolve().parents[1]

def strict_q3(data, tasks):
    p, nm, dem, comm, rows = data["center"], node_map(data), data["dem"], data["comm"], []
    fade, lsys = comm[("接收参数", "衰落裕量（dB）")], comm[("传播参数", "系统损耗（dB）")]
    direct_thr = min(20+3+12-(-98), 27+12+3-(-98))-fade-lsys
    access_thr = 20+3+6-(-98)-fade-lsys
    back_thr = min(19+8+12-(-98), 27+12+8-(-98))-fade-lsys
    for _, t in tasks.iterrows():
        s = nm[t["service"]]; p_h = float(p["海拔（m）"]) + 20; peak = max(v for _,v in terrain_profile(p,s,dem) if np.isfinite(v)); s_h = peak + 50
        direct = _segment_cert(p,s,p_h,s_h,dem,direct_thr,(20,3,12,0))
        best = None; lon0,lat0=float(p["经度（°）"]),float(p["纬度（°）"]); lon1,lat1=float(s["经度（°）"]),float(s["纬度（°）"])
        for frac in (.25,.5,.75):
            rp={"经度（°）":lon0+frac*(lon1-lon0),"纬度（°）":lat0+frac*(lat1-lat0)}
            rg=next(v for _,v in terrain_profile(rp,rp,dem) if np.isfinite(v))
            for off in (100.,200.,300.):
                rh=rg+off
                # Exact cell traversal on the two relay legs.  The first is
                # transport-to-relay access; the second is relay backhaul to G01.
                a=_segment_cert(p,rp,p_h,rh,dem,access_thr,(20,3,6,0))
                b=_segment_cert(rp,s,rh,s_h,dem,back_thr,(19,8,12,0))
                if a["feasible"] and b["feasible"]:
                    cand=(a["distance_m"]+b["distance_m"],frac,off,rg,a,b)
                    if best is None or cand[0]<best[0]: best=cand
        if best:
            margin=min(direct_thr-direct["loss_db"], 999) if direct["feasible"] else min(access_thr-best[4]["loss_db"], back_thr-best[5]["loss_db"])
            rows.append({"sortie":t["sortie"],"service":t["service"],"direct_feasible":bool(direct["feasible"]),"relay_feasible":True,"relay_needed":not direct["feasible"],"relay_fraction":best[1],"relay_height_offset_m":best[2],"min_clearance_m":min(best[4]["min_clearance_m"],best[5]["min_clearance_m"]),"link_margin_db":margin,"segment1_pixels":len(terrain_profile(p,{"经度（°）":lon0+best[1]*(lon1-lon0),"纬度（°）":lat0+best[1]*(lat1-lat0)},dem)),"segment2_pixels":len(terrain_profile({"经度（°）":lon0+best[1]*(lon1-lon0),"纬度（°）":lat0+best[1]*(lat1-lat0)},s,dem)),"total_distance_m":best[0]})
        else:
            rows.append({"sortie":t["sortie"],"service":t["service"],"direct_feasible":bool(direct["feasible"]),"relay_feasible":False,"relay_needed":not direct["feasible"],"relay_fraction":None,"relay_height_offset_m":None,"min_clearance_m":direct["min_clearance_m"],"link_margin_db":direct_thr-direct["loss_db"],"segment1_pixels":0,"segment2_pixels":0,"total_distance_m":None})
    return pd.DataFrame(rows)

def milp_schedule(data, batches, cert, relay_cap):
    base, _ = q2(data,batches); n=len(base); step=60.; horizon=24000.; slots=int(horizon/step)+1; N=n*slots
    boxes={r["货箱编号"]:r for r in data["boxes"]}; deadlines=[]; tardy=[]; late=[]
    for i,r in base.iterrows():
        ids=r["box_ids"] if isinstance(r["box_ids"],list) else json.loads(r["box_ids"]); ds=[]
        for bid in ids:
            b=boxes[bid]; ds.append(min(float(b["首批截止时间（s）"]),float(b["期望送达时间（s）"])) if b["是否首批保障"]=="是" else float(b["期望送达时间（s）"]))
        deadlines.append(ds); tardy.append([]); late.append([])
    c=np.zeros(N); rows=[]; lb=[]; ub=[]
    def ix(i,s): return i*slots+s
    for i,r in base.iterrows():
        for s in range(slots):
            st=s*step; delivery=st+(r["delivery_s"]-r["start_s"]); c[ix(i,s)]=sum(delivery>d for d in deadlines[i]); tardy[i].append(c[ix(i,s)]); late[i].append(sum(max(0,delivery-d) for d in deadlines[i]))
    # one start per task
    for i in range(n):
        row={ix(i,s):1 for s in range(slots)}; rows.append(row); lb.append(1); ub.append(1)
    # resources at every grid instant
    for t in range(slots):
        tm=t*step
        for typ,cap in [("A",len(data["aircraft"]["A"])),("B",len(data["aircraft"]["B"])),("C",len(data["aircraft"]["C"]))]:
            row={ix(i,s):1 for i,r in base.iterrows() if r["type"]==typ for s in range(max(0,int((tm-(r["return_s"]-r["start_s"]))//step)),min(slots,int(tm//step)+1))}; rows.append(row); lb.append(-np.inf); ub.append(cap)
        for typ,cap in data["batteries"].items():
            row={ix(i,s):1 for i,r in base.iterrows() if r["type"]==typ for s in range(max(0,int((tm-(r["return_s"]-r["start_s"])-3600)//step)),min(slots,int(tm//step)+1))}; rows.append(row); lb.append(-np.inf); ub.append(cap)
        row={ix(i,s):1 for i,r in base.iterrows() if bool(cert.iloc[i]["relay_needed"]) for s in range(max(0,int((tm-(r["return_s"]-r["start_s"])-data["relay"]["turn_s"])//step)),min(slots,int((tm+data["relay"]["prep_s"])//step)+1))}; rows.append(row); lb.append(-np.inf); ub.append(relay_cap)
    A=lil_matrix((len(rows),N))
    for ri,d in enumerate(rows):
        for j,v in d.items(): A[ri,j]=v
    cons=[LinearConstraint(A.tocsr(),np.array(lb),np.array(ub))]
    lo_bounds=np.zeros(N); hi_bounds=np.ones(N)
    for i in range(n):
        if bool(cert.iloc[i]["relay_needed"]):
            for s in range(int(data["relay"]["prep_s"]//step)): hi_bounds[ix(i,s)]=0.0
    res=milp(c,integrality=np.ones(N),bounds=Bounds(lo_bounds,hi_bounds),constraints=cons,options={"time_limit":60})
    if not res.success: return None,res
    out=base.copy(); starts=[]
    for i in range(n): starts.append(next(s*step for s in range(slots) if res.x[ix(i,s)]>.5))
    delivery_offset = base["delivery_s"].to_numpy() - base["start_s"].to_numpy()
    return_offset = base["return_s"].to_numpy() - base["start_s"].to_numpy()
    out["start_s"]=starts; out["delivery_s"]=out["start_s"].to_numpy()+delivery_offset; out["return_s"]=out["start_s"].to_numpy()+return_offset
    # Deterministically bind aggregate-capacity slots to physical IDs after
    # optimization, preventing an impossible overlap on one named aircraft.
    for typ in ("A", "B", "C"):
        avail={u:0.0 for u in data["aircraft"][typ]}
        for idx,r in out[out["type"]==typ].sort_values("start_s").iterrows():
            u=min(avail,key=avail.get); out.at[idx,"aircraft"]=u; avail[u]=float(r["return_s"])
    return out,res

def main():
    data=load_inputs(); _,batches=q1(data); base,_=q2(data,batches); cert=strict_q3(data,base); cert.to_csv(ROOT/"results/final_q3_continuous_certification.csv",index=False)
    base_relay_ids=set(cert.loc[cert["relay_needed"] & cert["relay_feasible"],"sortie"]); bev=[]
    for _,r in base.iterrows():
        if r["sortie"] in base_relay_ids: bev.extend([(float(r["start_s"]),1),(float(r["return_s"]),-1)])
    cur=bpk=0
    for _,d in sorted(bev,key=lambda z:(z[0],z[1])): cur+=d; bpk=max(bpk,cur)
    plans={}; chosen=None
    for R in (2,3,4):
        sol,res=milp_schedule(data,batches,cert,R); plans[R]={"solver_status":int(res.status),"success":bool(res.success),"mip_gap":getattr(res,"mip_gap",None),"message":str(res.message)}
        if sol is not None:
            # Evaluate hard deadlines for each resource-capacity run.
            bx={r["货箱编号"]:r for r in data["boxes"]}; bad=[]; bad_sorties=set()
            for _,rr in sol.iterrows():
                ids=rr["box_ids"] if isinstance(rr["box_ids"],list) else json.loads(rr["box_ids"])
                for bid in ids:
                    b=bx[bid]; ok=(b["物资类型"]!="医疗物资" or rr["delivery_s"]<=b["期望送达时间（s）"]) and (b["是否首批保障"]!="是" or rr["delivery_s"]<=b["首批截止时间（s）"])
                    if not ok: bad.append(bid); bad_sorties.add(rr["sortie"])
            plans[R].update({"start_max":float(sol["return_s"].max()),"tardy_boxes":len(bad),"tardy_sorties":len(bad_sorties)})
            if R==2: chosen=sol
            if R==2: sol.to_csv(ROOT/"results/final_joint_schedule.csv",index=False)
    if chosen is None: chosen=base
    # unified deliveries/events from the selected schedule
    boxes={r["货箱编号"]:r for r in data["boxes"]}; drows=[]; events=[]
    for _,r in chosen.iterrows():
        ids=r["box_ids"] if isinstance(r["box_ids"],list) else json.loads(r["box_ids"])
        for bid in ids:
            b=boxes[bid]; ok=float(r["delivery_s"])<=float(b["期望送达时间（s）"]) if b["物资类型"]=="医疗物资" else True
            ok=ok and (b["是否首批保障"]!="是" or float(r["delivery_s"])<=float(b["首批截止时间（s）"]))
            drows.append({"box_id":bid,"sortie":r["sortie"],"completion_s":r["delivery_s"],"hard_ok":ok})
        events.append({"sortie":r["sortie"],"start_s":r["start_s"],"end_s":r["return_s"],"resource":"transport_aircraft"})
        soc=1-float(r["energy_kwh"])/data["drones"][r["type"]].energy; full={"A":1800.,"B":2400.,"C":3000.}[r["type"]]
        charge=full*(0.65*(.9-soc)/.9+.35) if soc<.9 else full*.35*(1-soc)/.1
        events.append({"sortie":r["sortie"],"start_s":r["start_s"],"end_s":r["return_s"]+charge,"resource":f"battery_{r['type']}"})
        if bool(cert.loc[cert["sortie"]==r["sortie"],"relay_needed"].iloc[0]):
            events.append({"sortie":r["sortie"],"start_s":max(0,float(r["start_s"])-data["relay"]["prep_s"]),"end_s":float(r["return_s"])+data["relay"]["turn_s"],"resource":"relay_aircraft"})
    event_df=pd.DataFrame(events); event_df.to_csv(ROOT/"results/final_resource_events.csv",index=False)
    peaks={}
    for resource,g in event_df.groupby("resource"):
        ev=[]
        for _,e in g.iterrows(): ev.extend([(float(e.start_s),1),(float(e.end_s),-1)])
        cur=pk=0
        for _,d in sorted(ev,key=lambda z:(z[0],z[1])): cur+=d; pk=max(pk,cur)
        peaks[resource]=pk
    pd.DataFrame(drows).to_csv(ROOT/"results/final_joint_deliveries.csv",index=False)
    tardy=pd.DataFrame(drows).query("hard_ok == False")
    summary={"baseline_global_relay_peak":bpk,"plans":plans,"final_resource_peaks":peaks,"q2_tardy_boxes":len(tardy),"q3_direct":int(cert.direct_feasible.sum()),"q3_relay":int(cert.relay_feasible.sum()),"q3_infeasible":int((~cert.relay_feasible).sum()),"p1":"FAIL" if len(tardy) or not all(plans[r]["success"] for r in plans) else "PASS","note":"MILP uses 60-second start grid; continuous Q3 certificate uses complete DEM-cell traversal with piecewise-constant cell elevation."}
    (ROOT/"results/final_feasibility_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
