"""Diagnostic only: explain the repeated hard violations in C1."""
from __future__ import annotations
import json, math
from pathlib import Path
import numpy as np, pandas as pd
from minimal_pipeline import load_inputs, q1, q2, route_stats
from final_joint import strict_q3
from q2_sortie_count_sensitivity import split_to_k
from q3_reoptimize_from_q2 import _build_stage_a, _evaluate
from parity_evaluators import _ids, charge_A
from q2_k18_zero_hard import deadlines

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; LOG=ROOT/'logs'; TOL=1e-6

def main():
    D=load_inputs(); _,base=q1(D); base,_=q2(D,base); bm={str(x['货箱编号']):x for x in D['boxes']}
    schedules={}; deliveries={}; certs={}; fixed_sets={}
    for K in range(18,31):
        tasks=split_to_k(D,base,K); cert=strict_q3(D,tasks); s=_build_stage_a(D,tasks,cert); d,e=_evaluate(D,s,cert)
        schedules[K]=s; deliveries[K]=d; certs[K]=cert; fixed_sets[K]=set(d.loc[d.hard_violation,'box_id'])
    inter=set.intersection(*fixed_sets.values()); union=set.union(*fixed_sets.values())
    rows=[]
    for bid in sorted(union):
        b=bm[bid]; row={'box_id':bid,'service_area':str(bid).split('-')[0],'material_type':b.get('物资类型'),'hard_deadline':min(deadlines(b)) if deadlines(b) else np.nan}
        for K in range(18,31): row[f'K{K}_late']=bid in fixed_sets[K]
        rows.append(row)
    pd.DataFrame(rows).to_csv(RES/'q3_fixed_hard_violation_boxes.csv',index=False)
    # Ideal single-box lower bound.  Communication candidate is inherited from
    # the already certified service-level path; resources and other jobs are ignored.
    lb=[]
    for bid in sorted(union):
        b=bm[bid]; service=str(bid).split('-')[0]; ds=deadlines(b); hard=min(ds) if ds else np.nan; candidates=[]
        for typ in ('B','C','A'):
            if typ not in D['drones']: continue
            st=route_stats(D,typ,service,[b])
            if not st['safe']: continue
            dr=D['drones'][typ]; prep=float(dr.prep+dr.load_box); hand=float(dr.handoff+dr.handoff_box)
            relay_ready=float(D['relay']['prep_s']+D['relay']['link_s'])
            earliest=max(prep,relay_ready)+float(st['out_flight_s'])+hand
            candidates.append((earliest,typ,float(st['out_flight_s']),prep,hand,relay_ready))
        if candidates:
            earliest,typ,out,prep,hand,rr=min(candidates)
            slack=float(hard-earliest); poss=bool(earliest<=hard+TOL)
        else: earliest=typ=out=prep=hand=rr=np.nan; slack=np.nan; poss=False
        lb.append({'box_id':bid,'service':service,'hard_deadline':hard,'fastest_aircraft':typ,'minimum_transport_time':float(out+hand+prep) if np.isfinite(out) else np.nan,'minimum_relay_ready_time':rr,'earliest_possible_delivery':earliest,'slack':slack,'theoretically_possible':poss})
    lbdf=pd.DataFrame(lb); lbdf.to_csv(RES/'q3_hard_deadline_lower_bound.csv',index=False)
    # Use K19 as the canonical Q2->Q3 delay decomposition, with an explicit
    # attribution of the binding readiness constraint in the Q3 event list.
    q2f=pd.read_csv(RES/'q2_sensitivity_19_27'/'q2_sens_K19_schedule.csv'); q3=schedules[19]
    q2_by={str(r.sortie):r for _,r in q2f.iterrows()}; q3_by={str(r.sortie):r for _,r in q3.iterrows()}; q2_arr={bid:float(q2_by[str(row.sortie)].delivery_s) for _,row in q2f.iterrows() for bid in _ids(row.box_ids)}
    # Readiness traces from actual Q3 assignments.
    air={}; bat={}; svc={}; rel={}; trace=[]
    for _,r in q3.sort_values('start_s').iterrows():
        typ=str(r.type); service=str(r.service); au=str(r.aircraft); ba=str(r.battery); rid=str(r.relay_id); dep=float(r.delivery_s-r.start_s); ret=float(r.return_s-r.start_s); charge=float(charge_A(D,r)); hand=float(r.handoff_s)
        vals={'aircraft':air.get(au,0.0),'battery':bat.get(ba,0.0),'service':svc.get(service,0.0)-(dep-hand),'relay':rel.get(rid,0.0)+float(D['relay']['prep_s']),'preparation':float(D['relay']['prep_s'])}
        start=float(r.start_s); maxv=max(vals.values()); source=max(vals,key=vals.get)
        air[au]=float(r.return_s); bat[ba]=float(r.return_s)+charge; svc[service]=float(r.delivery_s); rel[rid]=float(r.return_s)+float(D['relay']['turn_s'])
        for bid in _ids(r.box_ids):
            delay=float(r.delivery_s)-q2_arr.get(bid,float(r.delivery_s)); trace.append({'sortie':r.sortie,'boxes':';'.join(_ids(r.box_ids)),'service':service,'hard_boxes':sum(bool(deadlines(bm[x]) and float(r.delivery_s)>min(deadlines(bm[x]))+TOL) for x in _ids(r.box_ids)),'deadline':min([x for bid2 in _ids(r.box_ids) for x in deadlines(bm[bid2])],default=np.nan),'q2_arrival':q2_arr.get(bid,np.nan),'q3_arrival':float(r.delivery_s),'delay_added_by_relay':max(0.0,vals['relay']-max(vals[k] for k in vals if k!='relay')),'delay_added_by_aircraft':max(0.0,vals['aircraft']-max(vals[k] for k in vals if k!='aircraft')),'delay_added_by_battery':max(0.0,vals['battery']-max(vals[k] for k in vals if k!='battery')),'delay_added_by_service':max(0.0,vals['service']-max(vals[k] for k in vals if k!='service')),'dominant_delay_source':source,'total_q2_to_q3_delay':delay})
    pd.DataFrame(trace).groupby(['sortie','boxes','service','hard_boxes','deadline'],as_index=False).agg({'q2_arrival':'min','q3_arrival':'min','delay_added_by_relay':'max','delay_added_by_aircraft':'max','delay_added_by_battery':'max','delay_added_by_service':'max','dominant_delay_source':'first','total_q2_to_q3_delay':'max'}).to_csv(RES/'q3_12_sortie_bottleneck_audit.csv',index=False)
    # Relay interval semantics audit.
    rr=[]
    for _,r in q3.iterrows():
        rr.append({'sortie':r.sortie,'transport_start':r.start_s,'transport_return':r.return_s,'relay_departure':max(0.0,float(r.start_s)-D['relay']['prep_s']),'relay_service_start':r.start_s,'relay_service_end':r.return_s,'relay_return':r.return_s,'relay_busy_end':r.return_s+D['relay']['turn_s'],'relay_prep_s':D['relay']['prep_s'],'relay_link_s':D['relay']['link_s'],'relay_turn_s':D['relay']['turn_s'],'relay_energy_component_charge_separate':False})
    pd.DataFrame(rr).to_csv(RES/'q3_relay_interval_semantics_audit.csv',index=False)
    report={'K_intersection_size':len(inter),'K_union_size':len(union),'same_24_across_all_K':len(inter)==24 and len(union)==24,'intersection_boxes':sorted(inter),'A_count':int((~lbdf.theoretically_possible).sum()),'B_count':int(lbdf.theoretically_possible.sum()),'relay_interval_implementation':'current code occupies relay from max(0,start-prep) through return+turn; link_s is not a separate event and relay energy-component charging is not modeled as a separate resource','next_step':'diagnostic only; do not modify model until independent review'}
    (LOG/'q3_fixed_violation_diagnostic.md').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
