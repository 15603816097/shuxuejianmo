"""Regression check for the common event tables at the 60-second baseline."""
from pathlib import Path
import json, pandas as pd
ROOT=Path(__file__).resolve().parents[1]
def peak(g):
    ev=[]
    for _,r in g.iterrows(): ev += [(float(r.start_s),1),(float(r.end_s),-1)]
    cur=pk=0
    for _,d in sorted(ev,key=lambda x:(x[0],x[1])): cur+=d; pk=max(pk,cur)
    return pk
def main():
    s=pd.read_csv(ROOT/'results/final_joint_schedule.csv'); d=pd.read_csv(ROOT/'results/final_joint_deliveries.csv'); e=pd.read_csv(ROOT/'results/final_resource_events.csv')
    grid_starts=bool(((s.start_s/60).round()-s.start_s/60).abs().max()<1e-9)
    tardy=int((~d.hard_ok).sum()) if 'hard_ok' in d else None
    resource_peaks={r:int(peak(g)) for r,g in e.groupby('resource')}
    expected={'tardy_boxes':14,'tardy_sorties':7}
    result={'grid_start_check':grid_starts,'event_resource_peaks':resource_peaks,'expected_original':expected,'tardy_boxes_observed':tardy,'parity':'FAIL','reason':'The current final_joint schedule is grid-constrained, but its event model does not yet independently reproduce every original metric under the same hard-deadline predicate.'}
    (ROOT/'logs/grid60_parity_test.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
