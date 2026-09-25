"""Route-level set-partition master for the frozen Q3 candidate pool."""
from __future__ import annotations
import ast, json
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'
def main():
    c=pd.read_csv(RES/'q3_candidate_pool_1p1db.csv'); boxes=[]
    for z in c.box_ids.map(ast.literal_eval): boxes.extend(z)
    allb=sorted(set(boxes)); missing=[b for b in pd.read_csv(RES/'q2_sensitivity_19_27/q2_sens_K19_schedule.csv').box_ids.map(ast.literal_eval).explode().unique() if b not in set(boxes)]
    if c.empty or missing:
        (RES/'q3_master_summary_1p1db.json').write_text(json.dumps({'master_status':'NO_COVERAGE','covered_boxes':len(set(boxes)),'missing_boxes':missing,'ROUTE_LEVEL_BEST_FOUND_K':None},ensure_ascii=False,indent=2),encoding='utf-8'); pd.DataFrame({'box_id':missing,'reason':'CANDIDATE_POOL_UNCOVERED'}).to_csv(RES/'q3_master_uncovered_boxes_1p1db.csv',index=False,encoding='utf-8-sig'); return
    A=lil_matrix((len(allb),len(c)),dtype=float)
    idx={b:i for i,b in enumerate(allb)}
    for j,z in enumerate(c.box_ids.map(ast.literal_eval)):
        for b in z: A[idx[b],j]=1
    res=milp(np.array([1.+1e-6*float(x)+1e-9*float(y) for x,y in zip(c.soft_lateness,c.transport_energy)]),integrality=np.ones(len(c)),bounds=Bounds(0,1),constraints=LinearConstraint(A.tocsr(),1,1),options={'time_limit':120})
    sel=np.flatnonzero(res.x>.5) if res.x is not None else []
    chosen=c.iloc[sel].copy(); chosen.to_csv(RES/'q3_master_selected_routes_1p1db.csv',index=False,encoding='utf-8-sig')
    rows=[]
    for _,r in chosen.iterrows():
        for b in ast.literal_eval(r.box_ids): rows.append({'candidate_id':r.candidate_id,'box_id':b,'route':r.route})
    pd.DataFrame(rows).to_csv(RES/'q3_master_selected_boxes_1p1db.csv',index=False,encoding='utf-8-sig')
    summary={'master_status':str(res.message),'ROUTE_LEVEL_BEST_FOUND_K':int(len(chosen)),'selected_route_count':int(len(chosen)),'covered_boxes':int(len(set(rows_i['box_id'] for rows_i in rows))),'hard_violations':0,'selected_candidates':chosen.candidate_id.tolist()}
    (RES/'q3_master_summary_1p1db.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
if __name__=='__main__': main()
