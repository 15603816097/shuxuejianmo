import ast, json, re, math
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2
from final_joint import strict_q3

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; RES.mkdir(exist_ok=True)
Ks=list(range(18,29))
D=load_inputs(); boxes=D['boxes']
boxmap={b['货箱编号']:b for b in boxes}

def parse_ids(v):
    if pd.isna(v): return []
    s=str(v)
    try:
        z=ast.literal_eval(s)
        if isinstance(z,list): return [str(x) for x in z]
    except Exception: pass
    return [x.strip() for x in re.split(r'[;,|]',s.strip('[]')) if x.strip()]

sets={}; unassigned_rows={}
for k in Ks:
    f=RES/f'q3_K{k}_unassigned.csv'; df=pd.read_csv(f) if f.exists() else pd.DataFrame()
    ids=set(); unassigned_rows[k]=df
    for _,r in df.iterrows(): ids.update(parse_ids(r.get('box_ids')))
    sets[k]=ids
inter=set.intersection(*sets.values()); union=set.union(*sets.values())
rows=[]
for bid in sorted(union):
    b=boxmap.get(bid,{})
    row={'box_id':bid,'service_area':b.get('服务区编号',''),'material_type':b.get('物资类型','')}
    for k in Ks: row[f'K{k}_unassigned']=bid in sets[k]
    rows.append(row)
pd.DataFrame(rows).to_csv(RES/'q3_K18_28_unassigned_box_stability.csv',index=False,encoding='utf-8-sig')

# service summary for fixed intersection (or union if unstable)
fixed=inter if len(inter)==len(union) else union
sr=[]
for svc,g in pd.DataFrame([boxmap[x] | {'box_id':x} for x in fixed]).groupby('服务区编号'):
    types=g['物资类型'].astype(str)
    sr.append({'service_area':svc,'box_count':len(g),'medical_count':int(types.str.contains('医疗').sum()),'water_count':int(types.str.contains('水').sum()),'food_count':int(types.str.contains('食|粮').sum()),'other_count':int((~types.str.contains('医疗|水|食|粮')).sum())})
pd.DataFrame(sr).sort_values('service_area').to_csv(RES/'q3_unassigned_service_summary.csv',index=False,encoding='utf-8-sig')

# service/assigned classification based K19 committed schedule
sched=pd.read_csv(RES/'q3_K19_new_schedule.csv')
assigned=set()
for v in sched.loc[sched.get('status','').astype(str).str.upper().eq('ASSIGNED'),'box_ids'] if 'status' in sched else []: assigned.update(parse_ids(v))
# fallback all rows with nonempty box ids and not UNASSIGNED
if not assigned:
  for _,r in sched.iterrows():
    if str(r.get('status','')).upper()!='UNASSIGNED': assigned.update(parse_ids(r.get('box_ids')))
all_services=sorted({b['服务区编号'] for b in boxes})
class_rows=[]
for svc in all_services:
    ids=[b['货箱编号'] for b in boxes if b['服务区编号']==svc]; un=[x for x in ids if x not in assigned]
    class_rows.append({'service':svc,'boxes_total':len(ids),'boxes_assigned':len(ids)-len(un),'boxes_unassigned':len(un),'direct_possible':False,'relay_possible':False,'best_access_margin':float('nan'),'best_backhaul_margin':float('nan'),'dominant_failure':'OTHER_TRUE_UNKNOWN' if un else 'NONE'})
# enrich with ideal service-level communication map after it is computed below
class_df=pd.DataFrame(class_rows)


# detailed rejection with conservative provenance: strict_dynamic failure has no margins, so unknown
map_service={}
for k in Ks:
    sf=RES/f'q3_K{k}_new_schedule.csv'
    if sf.exists():
      s=pd.read_csv(sf)
      for _,r in s.iterrows(): map_service[str(r.get('sortie'))]=r.get('service','')
    uf=RES/f'q3_K{k}_unassigned.csv'
    if uf.exists():
      u=pd.read_csv(uf)
      for _,r in u.iterrows():
        ids=parse_ids(r.get('box_ids')); sv=boxmap.get(ids[0],{}).get('服务区编号','') if ids else ''
        map_service[str(r.get('sortie'))]=sv
rejrows=[]
for k in Ks:
    f=RES/f'q3_K{k}_candidate_rejection_log.csv'
    if not f.exists(): continue
    df=pd.read_csv(f)
    for _,r in df.iterrows():
      reason=str(r.get('reason',''))
      if reason=='strict_dynamic_link_certificate_failed': cat='OTHER_TRUE_UNKNOWN'
      elif 'no_relay' in reason.lower(): cat='NO_RELAY_HOVER_CANDIDATE'
      elif 'access' in reason.lower() and 'backhaul' in reason.lower(): cat='ACCESS_AND_BACKHAUL_FAIL'
      elif 'access' in reason.lower(): cat='ACCESS_LINK_FAIL'
      elif 'backhaul' in reason.lower(): cat='BACKHAUL_LINK_FAIL'
      elif 'energy' in reason.lower(): cat='RELAY_ENERGY_FAIL'
      elif 'altitude' in reason.lower(): cat='RELAY_ALTITUDE_LIMIT'
      elif 'dem' in reason.lower(): cat='DEM_COVERAGE_FAIL'
      else: cat='OTHER_TRUE_UNKNOWN'
      rejrows.append({'sortie':r.get('sortie'),'service':map_service.get(str(r.get('sortie')),''),'candidate_hover_x':float('nan'),'candidate_hover_y':float('nan'),'candidate_altitude':r.get('candidate_hover'),'required_interval_start':float('nan'),'required_interval_end':float('nan'),'min_access_margin_db':r.get('min_access_margin'),'min_backhaul_margin_db':r.get('min_backhaul_margin'),'access_pass':float('nan'),'backhaul_pass':float('nan'),'rejection_reason':cat,'raw_reason':reason,'candidate_relay':r.get('candidate_relay')})
pd.DataFrame(rejrows).to_csv(RES/'q3_communication_rejection_detailed.csv',index=False,encoding='utf-8-sig')

# service ideal map using one representative Q2 task each
try:
  _,batches=q1(D); tasks,_=q2(D,batches)
  for col in ['service','服务区编号']:
    if col in tasks.columns: service_col=col; break
  else: service_col='service'
  im=[]
  for svc in all_services:
    tt=tasks[tasks[service_col].astype(str)==svc].head(1)
    if tt.empty:
      im.append({'service_area':svc,'direct_full_path_pass':False,'relay_candidate_count':0,'best_access_margin_db':float('nan'),'best_backhaul_margin_db':float('nan'),'best_joint_margin_db':float('nan'),'relay_feasible':False,'failure_reason':'NO_TASK'})
      continue
    cert=strict_q3(D,tt.copy())
    r=cert.iloc[0]
    relay=bool(r.get('relay_feasible',False)); direct=bool(r.get('direct_feasible',False))
    im.append({'service_area':svc,'direct_full_path_pass':direct,'relay_candidate_count':1 if relay else 0,'best_access_margin_db':r.get('min_access_margin_db',r.get('link_margin_db',float('nan'))),'best_backhaul_margin_db':r.get('min_backhaul_margin_db',r.get('link_margin_db',float('nan'))),'best_joint_margin_db':r.get('link_margin_db',float('nan')),'relay_feasible':relay,'failure_reason':'NONE' if relay or direct else 'STRICT_DYNAMIC_CERTIFICATE_FAIL'})
except Exception as e:
  im=[{'service_area':svc,'direct_full_path_pass':False,'relay_candidate_count':0,'best_access_margin_db':float('nan'),'best_backhaul_margin_db':float('nan'),'best_joint_margin_db':float('nan'),'relay_feasible':False,'failure_reason':'ERROR:'+repr(e)} for svc in all_services]
im_df=pd.DataFrame(im)
im_df.to_csv(RES/'q3_service_relay_feasibility_map.csv',index=False,encoding='utf-8-sig')
class_df=class_df.drop(columns=['direct_possible','relay_possible','best_access_margin','best_backhaul_margin'],errors='ignore').merge(im_df[['service_area','direct_full_path_pass','relay_feasible','best_access_margin_db','best_backhaul_margin_db']],left_on='service',right_on='service_area',how='left').rename(columns={'direct_full_path_pass':'direct_possible','relay_feasible':'relay_possible','best_access_margin_db':'best_access_margin','best_backhaul_margin_db':'best_backhaul_margin'}).drop(columns=['service_area'],errors='ignore')
class_df.to_csv(RES/'q3_service_communication_classification.csv',index=False,encoding='utf-8-sig')

# hover audit: inspect code-known generator
pd.DataFrame([{'candidate_count':9,'x_range':'route fractions 0.25,0.50,0.75','y_range':'route fractions 0.25,0.50,0.75','altitude_levels':'base+100,base+200,base+300 m','dem_coverage_ratio':float('nan'),'generation_method':'fixed route-fraction points x 3 altitude offsets','possible_search_gap':'YES: no continuous XY coverage; no full-area enumeration'}]).to_csv(RES/'q3_hover_candidate_generator_audit.csv',index=False,encoding='utf-8-sig')

# diagnostic metadata
meta={'K_values':Ks,'intersection_count':len(inter),'union_count':len(union),'fixed_set':len(inter)==len(union),'assigned_K19_count':len(assigned),'assigned_K19_boxes':len(assigned),'unassigned_K19_boxes':80-len(assigned),'rejection_rows':len(rejrows),'note':'Strict dynamic certificate failures lack access/backhaul margins in source logs; classified conservatively as OTHER_TRUE_UNKNOWN.'}
(RES/'q3_unassigned_diagnostic_meta.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(meta,ensure_ascii=False))
print('services fixed',sorted({boxmap[x].get('服务区编号') for x in fixed}))
