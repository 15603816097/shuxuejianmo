"""Freeze the best-known 11-box schedule and run finite R sensitivity scans."""
from __future__ import annotations
import ast, hashlib, json, shutil, subprocess, sys
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2
from parity_evaluators import eval_A, sweep
from continuous_joint import _solve, _schedule_from

ROOT=Path(__file__).resolve().parents[1]

def sha(path):
    h=hashlib.sha256(); h.update(Path(path).read_bytes()); return h.hexdigest()

def category_peak(events,prefix):
    q=events[events.resource.astype(str).str.startswith(prefix)]
    pts=[]
    for _,r in q.iterrows(): pts.extend([(round(float(r.start_s),6),1),(round(float(r.end_s),6),-1)])
    level=peak=0
    for _,d in sorted(pts,key=lambda x:(x[0],0 if x[1]<0 else 1)):
        level+=d; peak=max(peak,level)
    return peak

def pack_best(data):
    sched=pd.read_csv(ROOT/"results/continuous_cap11_schedule.csv")
    tasks, deliveries, events=eval_A(sched,data)
    if "relay" in sched.columns:
        for _,r in sched.iterrows():
            if str(r.get("relay","")):
                events.loc[(events.task_id==r["sortie"])&(events.resource=="relay_pool"),"resource"]=f"relay:{r['relay']}"
    cp=ROOT/"checkpoint_best_feasible_11"; cp.mkdir(exist_ok=True)
    sched.to_csv(cp/"schedule.csv",index=False); deliveries.to_csv(cp/"deliveries.csv",index=False); events.to_csv(cp/"resource_events.csv",index=False)
    shutil.copy2(ROOT/"results/final_q3_continuous_certification.csv",cp/"q3_continuous_certification.csv")
    for src in ["src/continuous_joint.py","src/continuous_cap_proof.py","src/continuous_cap10_strengthened.py","src/revalidate_cap11.py","src/finalize_current_results.py","src/plot_current_results.py","src/parity_evaluators.py","src/minimal_pipeline.py","src/final_joint.py","logs/continuous_final_milp.log","logs/continuous_cap13.log","logs/cap10_strengthened.log","logs/revalidate_cap11.log","results/复现清单.json"]:
        shutil.copy2(ROOT/src,cp/Path(src).name)
    peaks=sweep(events); transport=category_peak(events,"transport:"); relay=category_peak(events,"relay:"); battery=category_peak(events,"battery:"); service=category_peak(events,"service:")
    metrics={"status":"BEST_KNOWN_FEASIBLE_NOT_PROVEN_OPTIMAL","best_known_late_boxes":int((deliveries.lateness>1e-6).sum()),"best_known_late_sorties":int(tasks.late_flag.sum()),"proven_optimal":False,"transport_peak":transport,"relay_peak":relay,"battery_peak":battery,"service_peak":service,"total_lateness":float(deliveries.lateness.sum()),"max_lateness":float(deliveries.lateness.max()),"makespan":float(tasks.completion_time.max()),"energy":float(tasks.total_energy.sum()),"source":"results/continuous_cap11_schedule.csv","tolerance_s":1e-6}
    (cp/"metrics.json").write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding="utf-8")
    files={str(p.relative_to(cp)):sha(p) for p in cp.rglob("*") if p.is_file() and p.name!="checkpoint_manifest.json"}
    manifest={"status":"BEST_KNOWN_FEASIBLE_NOT_PROVEN_OPTIMAL","reproduction_command":"PYTHONPATH=.deps:src python src/finalize_current_results.py","files_sha256":files}
    (cp/"checkpoint_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    return metrics,events

def sensitivity(data):
    _,b=q1(data); tasks,_=q2(data,b); cert=pd.read_csv(ROOT/"results/final_q3_continuous_certification.csv"); rows=[]
    rows.append({"R":2,"tested_cap":11,"status":"KNOWN_FEASIBLE","late_boxes":11,"proven_optimal":False,"runtime_s":None})
    for R in (3,4,5):
        for cap in (0,10):
            res,names,rec=_solve(data,tasks,cert,f"R{R}_cap{cap}","cap",limit=20,violation_cap=cap,R=R)
            actual=None
            if rec["status"]==0:
                s=_schedule_from(tasks,res,names); _,d,_=eval_A(s,data); actual=int((d.lateness>1e-6).sum()); s.to_csv(ROOT/f"results/resource_R{R}_cap{cap}_schedule.csv",index=False)
            rows.append({"R":R,"tested_cap":cap,"status":rec["status"],"late_boxes":actual if actual is not None else cap if rec["status"]==0 else None,"proven_optimal":False,"runtime_s":rec["runtime_s"],"mip_gap":rec["mip_gap"]})
    out=pd.DataFrame(rows); out.to_csv(ROOT/"results/resource_sensitivity.csv",index=False); (ROOT/"logs/resource_sensitivity.log").write_text(out.to_json(orient="records",force_ascii=False,indent=2),encoding="utf-8"); return out

def main():
    data=load_inputs(); metrics,_=pack_best(data); sens=sensitivity(data); summary={"R2_metrics":metrics,"resource_sensitivity":sens.to_dict(orient="records"),"P1":"LIMITED / FAIL_OPTIMALITY_PROOF","unclosed":"cap<=10 strict feasibility/infeasibility proof","allowed_claim":"best-known feasible only"}; (ROOT/"results/current_result_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8"); print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=="__main__": main()
