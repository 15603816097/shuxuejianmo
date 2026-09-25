from pathlib import Path
import ast
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
def main():
    s=pd.read_csv(ROOT/'checkpoint_best_feasible_11/schedule.csv')
    rows=[]
    for _,r in s.iterrows():
        ids=ast.literal_eval(str(r['box_ids'])) if not isinstance(r['box_ids'],list) else r['box_ids']
        services=[str(r['service'])]
        rows.append({'sortie_id':r['sortie'],'route':f"O01 -> {' -> '.join(services)} -> O01",'service_sequence':';'.join(services),'service_area_count':len(services),'box_count':len(ids),'box_ids':';'.join(ids),'aircraft_id':r['aircraft'],'aircraft_type':r['type'],'start_s':r['start_s'],'return_s':r['return_s']})
    out=pd.DataFrame(rows).sort_values('sortie_id'); out.to_csv(ROOT/'results/q2_sortie_route_summary.csv',index=False)
    print({'total_sorties':len(out),'single_point':int((out.service_area_count==1).sum()),'double_point':int((out.service_area_count==2).sum()),'three_or_more':int((out.service_area_count>=3).sum()),'total_service_visits':int(out.service_area_count.sum()),'max_service_areas':int(out.service_area_count.max())})
if __name__=='__main__': main()
