from pathlib import Path
import json, pandas as pd
from minimal_pipeline import load_inputs,q1,q2
from continuous_joint import _solve,_schedule_from
from parity_evaluators import eval_A,sweep
ROOT=Path(__file__).resolve().parents[1]
def main():
 d=load_inputs(); _,b=q1(d); tasks,_=q2(d,b); cert=pd.read_csv(ROOT/'results/final_q3_continuous_certification.csv')
 res,names,rec=_solve(d,tasks,cert,'revalidate_cap11','cap',limit=90,violation_cap=11,R=2)
 out={'solver':rec,'model_fix':'corrected disjunctive Big-M and nonnegative relay preparation start','strictly_feasible':rec['status']==0}
 if rec['status']==0:
  s=_schedule_from(tasks,res,names)
  aircraft=[]; batteries=[]
  for i,r in s.iterrows():
   au=[u for u in d['aircraft'][r['type']] if f'air_{i}_{u}' in names and res.x[names.index(f'air_{i}_{u}')]>0.5]
   bu=[u for u in range(d['batteries'][r['type']]) if f'bat_{i}_{u}' in names and res.x[names.index(f'bat_{i}_{u}')]>0.5]
   aircraft.append(au[0] if au else r['aircraft']); batteries.append(f"{r['type']}-B{(bu[0]+1) if bu else 1:02d}")
  s['aircraft']=aircraft; s['battery']=batteries
  relay_ids=[]
  for i in range(len(s)):
   chosen=[u for u in range(2) if f'rel_{i}_{u}' in names and res.x[names.index(f'rel_{i}_{u}')]>0.5]
   relay_ids.append(f'R{chosen[0]+1:02d}' if chosen else '')
  s['relay']=relay_ids; s.to_csv(ROOT/'results/continuous_cap11_schedule.csv',index=False); t,dl,e=eval_A(s,d)
  for i,r in s.iterrows():
   if r['relay']:
    e.loc[(e.task_id==r['sortie'])&(e.resource=='relay_pool'),'resource']=f"relay:{r['relay']}"
  e.to_csv(ROOT/'results/continuous_cap11_resource_events.csv',index=False); dl.to_csv(ROOT/'results/continuous_cap11_deliveries.csv',index=False); out.update({'late_boxes':int((dl.lateness>1e-6).sum()),'late_sorties':int(t.late_flag.sum()),'resource_peaks':sweep(e),'makespan':float(t.completion_time.max()),'total_lateness':float(dl.lateness.sum())})
 (ROOT/'results/revalidate_cap11.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8'); (ROOT/'logs/revalidate_cap11.log').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
