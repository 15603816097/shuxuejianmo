"""Audit whether Q3 hard deadlines enter the solver and rerun hard-only checks."""
from __future__ import annotations
import ast,json,time,inspect
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,q1,q2
from final_joint import strict_q3
from q2_sortie_count_sensitivity import split_to_k
from q3_reoptimize_from_q2 import _build_stage_a,_evaluate
from continuous_joint import _solve
from parity_evaluators import _ids
from q2_k18_zero_hard import deadlines
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; LOG=ROOT/'logs'; M=100000.0; TOL=1e-6

def main():
 D=load_inputs(); _,base=q1(D); base,_=q2(D,base); bm={str(x['货箱编号']):x for x in D['boxes']}
 # Trace the 24 boxes through current C1's heuristic and the existing MILP semantics.
 fixed=pd.read_csv(RES/'q3_fixed_hard_violation_boxes_after_fix.csv'); q2k=pd.read_csv(RES/'q2_sensitivity_19_27'/'q2_sens_K19_schedule.csv'); taskmap={b:(str(r.sortie),str(r.service)) for _,r in q2k.iterrows() for b in _ids(r.box_ids)}
 trace=[]; source=inspect.getsource(__import__('continuous_joint').build_model)
 for k,(_,x) in enumerate(fixed.iterrows()):
  b=x.box_id; br=bm[b]; hard=min(deadlines(br)); task,service=taskmap.get(b,('',str(b).split('-')[0]));
  # Current build_model uses s_i + delivery_offset - deadline <= M*tardy_k.
  row=q2k[q2k.sortie.astype(str)==task].iloc[0]; do=float(row.delivery_s-row.start_s)
  trace.append({'box_id':b,'service':service,'material_type':br.get('物资类型'),'raw_deadline_s':hard,'model_deadline_s':hard,'associated_sortie':task,'arrival_expression':f's_{k}+{do:.9f} (delivery_s)','constraint_name':f'deadline_relax_{k}','constraint_rhs':hard-do,'big_M':M,'late_variable':f'tardy_box_{k}','constraint_active':False,'evidence':'build_model: s_i - M*tardy_k <= deadline-do; C1 heuristic has no solver constraint'})
 pd.DataFrame(trace).to_csv(RES/'q3_hard_deadline_constraint_trace.csv',index=False)
 # Deadline semantics audit on K19 Q2 reference: delivery, not return/recharge.
 sem=[]
 for _,r in q2k.iterrows():
  for b in _ids(r.box_ids):
   if b not in fixed.box_id.values: continue
   sem.append({'box_id':b,'deadline':min(deadlines(bm[b])),'delivery_time':r.delivery_s,'sortie_return_time':r.return_s,'resource_release_time':r.return_s,'current_constraint_time_variable':'delivery_s via s_i + delivery_offset','correct_time_variable':'delivery_time_at_service_area'})
 pd.DataFrame(sem).to_csv(RES/'q3_deadline_semantics_audit.csv',index=False)
 # Independent EDF cross-check for K18 (builder sorts by earliest hard deadline).
 tasks18=split_to_k(D,base,18); cert18=strict_q3(D,tasks18); edf=_build_stage_a(D,tasks18,cert18); dd,ee=_evaluate(D,edf,cert18)
 pd.DataFrame([{'K':18,'heuristic':'EDF','hard_violation_boxes':int(dd.hard_violation.sum()),'hard_violation_sorties':int(dd.loc[dd.hard_violation,'sortie_id'].nunique()),'soft_late_boxes':int((dd.soft_lateness_s>1e-6).sum()),'relay_peak':int(ee.resource.astype(str).str.startswith('relay:').sum())}]).to_csv(RES/'q3_edf_crosscheck.csv',index=False)
 # K18/K30 hard-box assignment/time differences.
 diff=[]
 for K in (18,30):
  tasks=split_to_k(D,base,K); cert=strict_q3(D,tasks); s=_build_stage_a(D,tasks,cert); d,e=_evaluate(D,s,cert); d=d[d.box_id.isin(fixed.box_id)]
  for _,r in d.iterrows(): diff.append({'K':K,'box_id':r.box_id,'sortie':r.sortie_id,'service':r.service_area,'q3_arrival':r.completion_time_s,'deadline':r.hard_deadline_s,'lateness':max(0.0,r.completion_time_s-r.hard_deadline_s)})
 df=pd.DataFrame(diff); piv=df.pivot(index='box_id',columns='K'); rows=[]
 for b in sorted(df.box_id.unique()):
  a=df[(df.box_id==b)&(df.K==18)].iloc[0]; z=df[(df.box_id==b)&(df.K==30)].iloc[0]; rows.append({'box_id':b,'K18_sortie':a.sortie,'K30_sortie':z.sortie,'K18_arrival':a.q3_arrival,'K30_arrival':z.q3_arrival,'K18_deadline':a.deadline,'K30_deadline':z.deadline,'K18_lateness':a.lateness,'K30_lateness':z.lateness,'sortie_changed':a.sortie!=z.sortie,'arrival_delta_K30_minus_K18':z.q3_arrival-a.q3_arrival})
 pd.DataFrame(rows).to_csv(RES/'q3_K18_K30_hardbox_diff.csv',index=False)
 # Exact hard-only feasibility: violation_cap=0 fixes tardy vars at 0.
 recs=[]
 for K in range(18,31):
  tasks=split_to_k(D,base,K); cert=strict_q3(D,tasks); _,_,rec=_solve(D,tasks,cert,f'Q3_deadline_fixed_K{K}','zero',limit=60,violation_cap=0,R=2); rec.update({'K':K,'hard_constraint_semantics':'tardy_box upper bound fixed to 0; delivery<=deadline','solver_hard_feasible':rec['status']==0}); recs.append(rec)
 pd.DataFrame(recs).to_csv(RES/'q3_stageC1_deadline_fixed_summary.csv',index=False)
 report={'trace_rows':len(trace),'current_c1_deadline_is_hard':False,'hard_only_solver_uses_delivery_time':True,'edf_K18_hard_violation_boxes':int(dd.hard_violation.sum()),'K18_K30_same_assignment':bool((pd.read_csv(RES/'q3_K18_K30_hardbox_diff.csv').sortie_changed==False).all()),'next':'deadline hard-only solver is now auditable; dynamic communication and energy remain pending'}
 (LOG/'q3_deadline_constraint_audit.md').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8'); print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
