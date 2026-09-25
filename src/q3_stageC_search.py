"""Stage-C Q3 search with independent Q2 task construction.

This search opens the box grouping and K (single-service route columns) instead
of inheriting the frozen Q2 K=19 batches.  It is a bounded deterministic search
for feasible evidence, not a global-optimality claim.
"""
from __future__ import annotations
import json, time
from pathlib import Path
import numpy as np
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2
from final_joint import strict_q3
from q2_sortie_count_sensitivity import split_to_k
from q3_reoptimize_from_q2 import _build_stage_a, _evaluate, _event_peak
from parity_evaluators import _ids

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/"results"; LOG=ROOT/"logs"

def main():
    data=load_inputs(); _,base=q1(data); base,_=q2(data,base)
    rows=[]; found=None
    for K in range(18,31):
        t0=time.time(); tasks=split_to_k(data,base,K); cert=strict_q3(data,tasks)
        sched=_build_stage_a(data,tasks,cert); d,e=_evaluate(data,sched,cert)
        hard=int(d.hard_violation.sum()); hard_sorties=int(d.loc[d.hard_violation,'sortie_id'].nunique())
        soft=d[d.soft_deadline_s.notna()]; relay_peak=_event_peak(e,lambda x:x.resource.astype(str).str.startswith('relay:'))
        tr_peak=_event_peak(e,lambda x:x.resource.astype(str).str.startswith('transport:')); bp=_event_peak(e,lambda x:x.resource.astype(str).str.startswith('battery:B-')); cp=_event_peak(e,lambda x:x.resource.astype(str).str.startswith('battery:C-')); sp=_event_peak(e,lambda x:x.resource.astype(str).str.startswith('service:'))
        comm=bool(int(cert.relay_feasible.sum())==K and int(cert.direct_feasible.sum())==0)
        resource=bool(relay_peak<=2 and tr_peak<=sum(map(len,data['aircraft'].values())) and bp<=data['batteries']['B'] and cp<=data['batteries']['C'] and sp<=15)
        rec={"K":K,"transport_sorties":K,"relay_sorties":int(cert.relay_feasible.sum()),"hard_violation_boxes":hard,"hard_violation_sorties":hard_sorties,"soft_late_boxes":int((soft.soft_lateness_s>1e-6).sum()),"soft_total_lateness_s":float(soft.soft_lateness_s.sum()),"soft_max_lateness_s":float(soft.soft_lateness_s.max()),"joint_makespan_s":float(sched.return_s.max()),"transport_energy_kwh":float(sched.energy_kwh.sum()),"relay_energy_kwh":float(cert.total_distance_m.sum()*data['relay']['power_kw']/data['relay']['speed']/3600.0),"relay_peak":relay_peak,"B_battery_peak":bp,"C_battery_peak":cp,"transport_peak":tr_peak,"service_peak":sp,"communication_feasible":comm,"resource_feasible":resource,"hard_feasible":hard==0,"complete_feasible":bool(hard==0 and comm and resource),"solver_status":"DETERMINISTIC_BOUNDED_SEARCH","proven_optimal":False,"runtime_s":time.time()-t0}
        rows.append(rec)
        if rec['complete_feasible'] and found is None:
            found=(rec,sched,d,e,cert)
            break
    pd.DataFrame(rows).to_csv(RES/'q3_stageC_search_summary.csv',index=False)
    summary={"C1_single_point_completed":True,"C1_first_feasible_K":None if found is None else found[0]['K'],"C2_multi_point_attempted":False,"C2_reason":"No validated multi-point DEM/link candidate generator exists in the current certified route interface; no multi-point result is claimed.","rows":rows}
    (RES/'q3_stageC_search_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    if found is not None:
        rec,s,d,e,c=found; s.to_csv(RES/'q3_final_schedule.csv',index=False); d.to_csv(RES/'q3_final_deliveries.csv',index=False); e.to_csv(RES/'q3_final_resource_events.csv',index=False); c.to_csv(RES/'q3_final_relay_schedule.csv',index=False); (RES/'q3_final_metrics.json').write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding='utf-8')
    else:
        (RES/'q3_final_metrics.json').write_text(json.dumps({"status":"NO_C1_FEASIBLE_SOLUTION","C2_not_attempted":True},ensure_ascii=False,indent=2),encoding='utf-8')
    (LOG/'q3_stageC_search.md').write_text("\n".join(["# Q3 Stage C 搜索","","C1重新组批、单点往返，K=18..30；不继承Q2 K=19固定批次。所有候选使用逐DEM栅格认证和R=2中继事件审计。",f"C1首个完整可行K：{summary['C1_first_feasible_K']}","C2未形成可验证的多点DEM/通信候选，因此不报告多点可行性。","结果状态：仅将真实计算得到的候选标记为可行；未找到的K不解释为全局不可行。","" ]),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
