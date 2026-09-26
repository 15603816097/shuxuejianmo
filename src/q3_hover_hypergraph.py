"""Friend-paper-style Q3 hover-candidate hypergraph on a frozen Q2 schedule.

The base pool is exactly 1596 candidates = 19 longitude grid points * 14
latitude grid points * 6 AGL levels (50..300 m), matching the benchmark
candidate count. Historical dense-search points are used only as search seeds;
all pass/fail decisions are recomputed with the frozen strict Q3 physics.

For each relay-required continuous block we require one stationary relay point
to certify every physical primitive endpoint in the block, plus the backhaul
to G01 and relay energy reserve. If a base block is uncovered, a local adaptive
refinement is run around its best base candidates. A minimum set-cover MILP
then reports how many distinct hover positions are needed to cover all blocks.

This is a geographic/route-level gate, not yet the final global relay-time
resource schedule.
"""
from __future__ import annotations
import argparse, ast, hashlib, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix

import q3_final_pipeline as pipe
from q3_official_semantics import relay_leg_time_energy
from minimal_pipeline import line_geometry, terrain_profile

ROOT=Path(__file__).resolve().parents[1]
SHA="5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb"
TOL=1e-9

def parse_ids(v):
    if isinstance(v,list): return [str(x) for x in v]
    try: return [str(x) for x in json.loads(str(v))]
    except Exception: return [str(x) for x in ast.literal_eval(str(v))]

def ground(data,lon,lat):
    p={"经度（°）":float(lon),"纬度（°）":float(lat)}
    vals=[z for _,z in terrain_profile(p,p,data["dem"]) if np.isfinite(z)]
    return max(vals) if vals else float("nan")

def fspl(freq_mhz,dist_m):
    return 32.45+20*math.log10(freq_mhz)+20*math.log10(max(dist_m/1000.0,1e-12))

def candidate_record(data,lon,lat,agl,source):
    g=ground(data,lon,lat)
    if not np.isfinite(g) or agl<0 or agl>float(data["relay"]["max_height_m"])+TOL:
        return None
    p={"经度（°）":float(lon),"纬度（°）":float(lat)}
    h=float(g+agl)
    return {"lon":float(lon),"lat":float(lat),"agl_m":float(agl),"ground_m":float(g),
            "altitude_m":h,"source":source,"p":p}

def base_candidates(data):
    lons=np.asarray(data["dem"]["longitude"]).ravel()
    lats=np.asarray(data["dem"]["latitude"]).ravel()
    xs=np.linspace(float(lons.min()),float(lons.max()),19)
    ys=np.linspace(float(lats.min()),float(lats.max()),14)
    out=[]
    for x in xs:
      for y in ys:
        for agl in (50.,100.,150.,200.,250.,300.):
          z=candidate_record(data,x,y,agl,"base_1596")
          if z is not None: out.append(z)
    return out

def seed_candidates(data):
    out=[]; seen=set()
    files=[
      ROOT/"results"/"q3_five_service_hover_refinement.csv",
      ROOT/"results"/"q3_S008_hover_refinement_v2.csv",
      ROOT/"results"/"q3_S008_global_hover_optimization.csv",
    ]
    for p in files:
      if not p.exists(): continue
      df=pd.read_csv(p)
      for _,r in df.head(300).iterrows():
        lon=r.get("x",r.get("candidate_hover_x",np.nan))
        lat=r.get("y",r.get("candidate_hover_y",np.nan))
        alt=r.get("altitude",r.get("candidate_altitude",np.nan))
        if not (pd.notna(lon) and pd.notna(lat) and pd.notna(alt)): continue
        g=ground(data,float(lon),float(lat))
        if not np.isfinite(g): continue
        agl=float(alt)-g
        key=(round(float(lon),7),round(float(lat),7),round(agl,2))
        if key in seen or agl<0 or agl>float(data["relay"]["max_height_m"])+TOL: continue
        seen.add(key)
        z=candidate_record(data,float(lon),float(lat),agl,"historical_seed_recertified")
        if z is not None: out.append(z)
    return out

def _margin_fast(data,a,b,ha,hb,thr):
    """Exact pass/fail with cheap bounds; terrain is queried only in the ambiguous 0..obstacle-loss band."""
    freq=float(data["comm"][("传播参数","载波频率（MHz）")])
    obs=float(data["comm"][("传播参数","地形遮挡附加损耗（dB）")])
    A=pipe.xyz(data,a,ha); B=pipe.xyz(data,b,hb)
    dist=float(np.linalg.norm(A-B))
    clear=float(thr-fspl(freq,dist))
    if clear < -TOL:
        return clear
    if clear-obs >= -TOL:
        # It passes even if obstructed. Return the conservative obstructed margin
        # so ranking never overstates the certificate.
        return clear-obs
    blocked=pipe.link_obstructed(data,a,b,ha,hb)
    return float(clear-(obs if blocked else 0.0))

def candidate_static(data,cand,tb):
    """Candidate quantities independent of the served block; compute once."""
    c=data["center"]
    gh=float(c["海拔（m）"])+float(data["comm"][("固定网关 G01","天线离地高度（m）")])
    bm=_margin_fast(data,cand["p"],c,cand["altitude_m"],gh,tb)
    dist,peak,*_=line_geometry(c,cand["p"],data["dem"])
    _,eout=relay_leg_time_energy(data,dist,peak,float(c["海拔（m）"]),cand["altitude_m"])
    _,eback=relay_leg_time_energy(data,dist,peak,cand["altitude_m"],float(c["海拔（m）"]))
    return {"backhaul_margin":float(bm),"base_flight_energy_kwh":float(eout+eback)}

def block_score(data,block,cand,ta,static):
    margins=[]
    for q in block["items"]:
      for ep,eh in ((q["a"],q["ha"]),(q["b"],q["hb"])):
        margins.append(_margin_fast(data,ep,cand["p"],eh,cand["altitude_m"],ta))
    bm=float(static["backhaul_margin"])
    access=min(margins) if margins else float("inf")
    dur=float(block["end_s"])-float(block["start_s"])
    e=float(static["base_flight_energy_kwh"])+(float(data["relay"]["hover_power_kw"])+float(data["relay"]["comm_power_kw"]))*dur/3600.0
    emax=(1-float(data["relay"]["reserve"]))*float(data["relay"]["energy_kwh"])
    energy_ok=bool(e<=emax+TOL)
    return min(access,bm),access,bm,energy_ok,e

def local_refine(data,block,base_ranked,ta,tb):
    out=[]; seen=set()
    for _,c0 in base_ranked[:6]:
      for dx in np.linspace(-0.003,0.003,7):
        for dy in np.linspace(-0.003,0.003,7):
          for da in (-50.,-25.,0.,25.,50.):
            agl=min(float(data["relay"]["max_height_m"]),max(10.,float(c0["agl_m"])+da))
            key=(round(c0["lon"]+dx,7),round(c0["lat"]+dy,7),round(agl,2))
            if key in seen: continue
            seen.add(key)
            z=candidate_record(data,c0["lon"]+dx,c0["lat"]+dy,agl,"adaptive_refine")
            if z is not None: out.append(z)
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--schedule",required=True)
    ap.add_argument("--label",required=True)
    ap.add_argument("--out",default="results/q3_hypergraph")
    args=ap.parse_args()
    got=hashlib.sha256((ROOT/"src/q3_final_pipeline.py").read_bytes()).hexdigest()
    if got!=SHA: raise RuntimeError(f"frozen Q3 SHA mismatch: {got}")
    data=pipe.load_inputs(); sch=pd.read_csv(args.schedule)
    td,ta,tb=pipe.thresholds(data)
    blocks=[]; route_rows=[]
    for ri,r in sch.iterrows():
      ids=parse_ids(r.box_ids)
      order=tuple(str(r.visit_order).split(">"))
      tl=pipe.build_authoritative_timeline(data,order,str(r.drone_type),ids)
      bks=pipe.direct_and_relay_blocks(data,tl)
      for bi,b in enumerate(bks):
        blocks.append({"block_id":f"{args.label}-R{ri+1:02d}-B{bi+1:02d}",
                       "route_index":int(ri),"candidate_id":str(r.get("candidate_id","")),
                       "visit_order":str(r.visit_order),"drone_type":str(r.drone_type),**b})
      route_rows.append({"route_index":ri,"candidate_id":str(r.get("candidate_id","")),
                         "visit_order":str(r.visit_order),"drone_type":str(r.drone_type),
                         "box_count":len(ids),"relay_blocks":len(bks)})
    outdir=Path(args.out)/args.label; outdir.mkdir(parents=True,exist_ok=True)
    pd.DataFrame(route_rows).to_csv(outdir/"routes.csv",index=False,encoding="utf-8-sig")
    if not blocks:
      summary={"label":args.label,"routes":len(sch),"relay_blocks":0,"all_blocks_covered":True,
               "minimum_hover_positions":0,"scenario":"OFFICIAL_STRICT"}
      (outdir/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
      print(json.dumps(summary,ensure_ascii=False,indent=2)); return

    cands=base_candidates(data)
    base_count=len(cands)
    cands += seed_candidates(data)
    # dedupe candidates after seeding
    uniq=[]; seen=set()
    for c in cands:
      k=(round(c["lon"],7),round(c["lat"],7),round(c["agl_m"],2))
      if k not in seen: seen.add(k); uniq.append(c)
    cands=uniq

    print(f"{args.label}: relay_blocks={len(blocks)}, candidates={len(cands)}",flush=True)
    statics=[candidate_static(data,c,tb) for c in cands]
    cover=[set() for _ in cands]
    block_rows=[]; unresolved=[]
    for bi,b in enumerate(blocks):
      ranked=[]
      for ci,c in enumerate(cands):
        score,am,bm,eok,e=block_score(data,b,c,ta,statics[ci])
        ranked.append((score,c,ci,am,bm,eok,e))
        if score>=-TOL and eok: cover[ci].add(bi)
      ranked.sort(key=lambda x:x[0],reverse=True)
      feasible=sum(1 for z in ranked if z[0]>=-TOL and z[5])
      best=ranked[0]
      if feasible==0: unresolved.append((bi,b,[(z[0],z[1]) for z in ranked[:6]]))
      print(f"{args.label}: block {bi+1}/{len(blocks)} feasible_candidates={feasible} best_margin={best[0]:.4f}",flush=True)
      block_rows.append({"block_id":b["block_id"],"route_index":b["route_index"],
                         "visit_order":b["visit_order"],"start_s":b["start_s"],"end_s":b["end_s"],
                         "base_feasible_candidates":feasible,"best_joint_margin_db":best[0],
                         "best_access_margin_db":best[3],"best_backhaul_margin_db":best[4],
                         "best_energy_ok":best[5],"best_relay_energy_kwh":best[6]})

    # adaptive refinement only for uncovered blocks
    for ui,(bi,b,ranked) in enumerate(unresolved,1):
      print(f"{args.label}: adaptive refine {ui}/{len(unresolved)} for {b['block_id']}",flush=True)
      new=local_refine(data,b,ranked,ta,tb)
      for c in new:
        k=(round(c["lon"],7),round(c["lat"],7),round(c["agl_m"],2))
        if k in seen: continue
        seen.add(k); ci=len(cands); cands.append(c); st=candidate_static(data,c,tb); statics.append(st); cv=set()
        # First require the refinement to solve its target uncovered block.
        target_score,_,_,target_eok,_=block_score(data,b,c,ta,st)
        if target_score < -TOL or not target_eok:
          cover.append(cv); continue
        # Only successful refinements are tested against all blocks for sharing.
        for bj,bb in enumerate(blocks):
          score,am,bm,eok,e=block_score(data,bb,c,ta,st)
          if score>=-TOL and eok: cv.add(bj)
        cover.append(cv)

    covered_by=[[] for _ in blocks]
    for ci,s in enumerate(cover):
      for bi in s: covered_by[bi].append(ci)
    uncovered=[blocks[i]["block_id"] for i,z in enumerate(covered_by) if not z]
    chosen=[]
    if not uncovered:
      useful=[i for i,s in enumerate(cover) if s]
      A=lil_matrix((len(blocks),len(useful)),dtype=float)
      for j,ci in enumerate(useful):
        for bi in cover[ci]: A[bi,j]=1.0
      res=milp(np.ones(len(useful)),integrality=np.ones(len(useful)),
               bounds=Bounds(0,1),constraints=LinearConstraint(A.tocsr(),1,np.inf),
               options={"time_limit":120})
      if res.x is not None:
        chosen=[useful[j] for j in np.flatnonzero(res.x>.5)]

    cand_rows=[]
    for i,c in enumerate(cands):
      if cover[i]:
        cand_rows.append({"candidate_index":i,"lon":c["lon"],"lat":c["lat"],"agl_m":c["agl_m"],
                          "altitude_m":c["altitude_m"],"source":c["source"],
                          "covered_block_count":len(cover[i]),
                          "chosen_min_cover":i in chosen,
                          "covered_blocks":"|".join(blocks[j]["block_id"] for j in sorted(cover[i]))})
    pd.DataFrame(block_rows).to_csv(outdir/"block_diagnostics.csv",index=False,encoding="utf-8-sig")
    pd.DataFrame(cand_rows).to_csv(outdir/"candidate_coverage.csv",index=False,encoding="utf-8-sig")
    if chosen:
      pd.DataFrame([cand_rows[[x["candidate_index"] for x in cand_rows].index(i)] for i in chosen]).to_csv(
        outdir/"minimum_cover_positions.csv",index=False,encoding="utf-8-sig")
    summary={
      "label":args.label,"routes":int(len(sch)),"boxes":int(sum(len(parse_ids(x)) for x in sch.box_ids)),
      "unique_boxes":int(len({b for x in sch.box_ids for b in parse_ids(x)})),
      "relay_blocks":int(len(blocks)),"base_candidate_target":1596,"base_candidates_dem_valid":int(base_count),
      "total_candidates_after_seed_refine":int(len(cands)),
      "covered_blocks":int(len(blocks)-len(uncovered)),"uncovered_blocks":uncovered,
      "all_blocks_covered":bool(not uncovered),
      "minimum_hover_positions":None if uncovered or not chosen else int(len(chosen)),
      "scenario":"OFFICIAL_STRICT","delta_db":0.0,
      "frozen_pipeline_sha256":got,
      "scope_note":"Geographic/route-block hypergraph only; global relay concurrency, component reuse and final continuous-time schedule remain to be certified."
    }
    (outdir/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
