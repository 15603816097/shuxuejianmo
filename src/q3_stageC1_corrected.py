"""Stage-C1 rerun with separated relay airframe and energy-component ledgers."""
from __future__ import annotations
import json,time
from pathlib import Path
import numpy as np,pandas as pd
from minimal_pipeline import load_inputs,q1,q2
from final_joint import strict_q3
from q2_sortie_count_sensitivity import split_to_k,pool_peak
from q3_reoptimize_from_q2 import _build_stage_a,_evaluate
from parity_evaluators import _ids
from q3_official_semantics import relay_component_ledger, relay_time_chain
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; LOG=ROOT/'logs'; TOL=1e-6

def corrected_events(D,s,cert):
    rows=[]; dyn=[]
    comp_avail={f'R-E{i+1:02d}':0.0 for i in range(int(D['relay']['component_inventory']))}
    for _,r in s.iterrows():
        c=cert[cert.sortie.astype(str)==str(r.sortie)].iloc[0]; need=not bool(c.direct_feasible)
        prep=float(D['relay']['prep_s']); link=float(D['relay']['link_s']); turn=float(D['relay']['turn_s'])
        # Static certification is used for geometry; time ledger is explicit.
        if need:
            # Use the official phase chain.  The relay airframe and its
            # energy component have separate ledgers; charging never locks
            # the airframe.
            chain=relay_time_chain(D,str(r.service),c,float(getattr(r,'handoff_s',0.0)),float(r.start_s))
            cid=min(comp_avail, key=comp_avail.get)
            comp=relay_component_ledger(D,cid,chain['prepare_start'],chain['return_end'],chain['total_energy_kwh'],comp_avail[cid])
            comp_avail[cid]=comp['available_again_s']
            prep0=chain['prepare_start']; out_end=chain['outbound_end']; setup_end=chain['link_setup_end']; service_start=chain['service_start']; service_end=chain['service_end']; return_start=chain['return_start']; busy_end=chain['busy_end']
            rows += [
                {'resource':'relay_airframe','sortie':r.sortie,'phase':'prepare','start_s':prep0,'end_s':out_end},
                {'resource':'relay_airframe','sortie':r.sortie,'phase':'link_setup','start_s':chain['link_setup_start'],'end_s':setup_end},
                {'resource':'relay_airframe','sortie':r.sortie,'phase':'service','start_s':service_start,'end_s':service_end},
                {'resource':'relay_airframe','sortie':r.sortie,'phase':'return','start_s':return_start,'end_s':chain['return_end']},
                {'resource':'relay_airframe','sortie':r.sortie,'phase':'turnaround','start_s':chain['turnaround_start'],'end_s':busy_end},
                {'resource':'relay_energy_component','sortie':r.sortie,'component_id':comp['component_id'],'phase':'use','start_s':comp['use_start_s'],'end_s':comp['use_end_s'],'energy_kwh':comp['energy_kwh'],'soc_end':comp['soc_end'],'resource_available_ok':comp['resource_available_ok'],'reserve_ok':comp['reserve_ok']},
                {'resource':'relay_energy_component','sortie':r.sortie,'component_id':comp['component_id'],'phase':'charge','start_s':comp['charge_start_s'],'end_s':comp['charge_end_s'],'energy_kwh':0.0,'soc_end':1.0},
            ]
            dyn.append({'sortie':r.sortie,'time_interval_start':float(chain['prepare_start']),'time_interval_end':float(chain['busy_end']),'direct_available':False,'relay_required':True,'relay_id':getattr(r,'relay_id','R01'),'access_link_ok':bool(c.relay_feasible),'backhaul_link_ok':bool(c.relay_feasible),'communication_ok':False,'communication_status':'DYNAMIC_TRAJECTORY_AUDIT_PENDING','relay_prepare_start':prep0,'relay_outbound_end':out_end,'relay_link_setup_end':setup_end,'relay_service_start':service_start,'relay_service_end':service_end,'relay_return_start':return_start,'relay_busy_end':busy_end,'energy_use_start':float(comp['use_start_s']),'energy_use_end':float(comp['use_end_s']),'energy_charge_start':float(comp['charge_start_s']),'energy_charge_end':float(comp['charge_end_s']),'energy_reserve_ok':bool(comp['reserve_ok']),'component_id':comp['component_id']})
        else:
            dyn.append({'sortie':r.sortie,'time_interval_start':float(r.start_s),'time_interval_end':float(r.return_s),'direct_available':True,'relay_required':False,'relay_id':'','access_link_ok':True,'backhaul_link_ok':True,'communication_ok':False,'communication_status':'DYNAMIC_UNVERIFIED','relay_prepare_start':np.nan,'relay_outbound_end':np.nan,'relay_link_setup_end':np.nan,'relay_service_start':np.nan,'relay_service_end':np.nan,'relay_return_start':np.nan,'relay_busy_end':np.nan,'energy_use_start':np.nan,'energy_use_end':np.nan,'energy_charge_start':np.nan,'energy_charge_end':np.nan})
    return pd.DataFrame(rows),pd.DataFrame(dyn)

def peak(df,res):
    if len(df)==0:return 0
    return pool_peak(df,lambda x:x.resource.astype(str).eq(res))

def main():
    D=load_inputs(); _,base=q1(D); base,_=q2(D,base); out=[]; fixed={}; first=None; k19_dyn=None
    for K in range(18,31):
        tasks=split_to_k(D,base,K); cert=strict_q3(D,tasks); s=_build_stage_a(D,tasks,cert); d,e_old=_evaluate(D,s,cert); ev,dyn=corrected_events(D,s,cert)
        if K==19: k19_dyn=dyn
        hard=int(d.hard_violation.sum()); fixed[K]=set(d.loc[d.hard_violation,'box_id']); soft=d[d.soft_deadline_s.notna()]
        rap=peak(ev,'relay_airframe'); rep=peak(ev,'relay_energy_component'); tp=peak(e_old, 'transport:') if False else pool_peak(e_old,lambda x:x.resource.astype(str).str.startswith('transport:')); bp=pool_peak(e_old,lambda x:x.resource.astype(str).str.startswith('battery:B-')); cp=pool_peak(e_old,lambda x:x.resource.astype(str).str.startswith('battery:C-')); sp=pool_peak(e_old,lambda x:x.resource.astype(str).str.startswith('service:'))
        relay_energy=float(ev.loc[ev.phase=='use','energy_kwh'].sum()) if 'energy_kwh' in ev.columns and len(ev) else 0.0
        use_rows=ev.loc[ev.phase=='use'] if 'phase' in ev.columns else ev.iloc[0:0]
        resource_ok=bool(len(use_rows) and use_rows['resource_available_ok'].all() and use_rows['reserve_ok'].all()
                         and rap <= len(D['relay']['aircraft']) and rep <= int(D['relay']['component_inventory'])
                         and tp <= sum(len(v) for v in D['aircraft'].values())
                         and bp <= D['batteries']['B'] and cp <= D['batteries']['C']) if len(use_rows) else False
        rec={'K':K,'transport_sorties':K,'relay_sorties':int(cert.relay_feasible.sum()),'hard_violation_boxes':hard,'hard_violation_sorties':int(d.loc[d.hard_violation,'sortie_id'].nunique()),'soft_late_boxes':int((soft.soft_lateness_s>TOL).sum()),'soft_total_lateness_s':float(soft.soft_lateness_s.sum()),'soft_max_lateness_s':float(soft.soft_lateness_s.max()),'joint_makespan_s':float(s.return_s.max()),'transport_energy_kwh':float(s.energy_kwh.sum()),'relay_energy_kwh':relay_energy,'relay_airframe_peak':rap,'relay_energy_component_peak':rep,'transport_peak':tp,'B_battery_peak':bp,'C_battery_peak':cp,'service_peak':sp,'communication_feasible':False,'communication_status':'STATIC_CERT_ONLY_DYNAMIC_UNVERIFIED','resource_feasible':resource_ok,'resource_status':'PARTIAL_DYNAMIC_COMMUNICATION_PENDING','complete_feasible':False,'energy_component_charge_defined':True,'relay_energy_component_inventory_known':True,'solver_status':'DETERMINISTIC_C1_AFTER_RESOURCE_SEMANTICS_FIX','proven_optimal':False}
        out.append(rec)
        if rec['complete_feasible'] and first is None:first=(K,s,d,cert,ev,dyn)
    pd.DataFrame(out).to_csv(RES/'q3_stageC1_corrected_summary.csv',index=False)
    inter=set.intersection(*fixed.values()); union=set.union(*fixed.values()); bm={str(x['货箱编号']):x for x in D['boxes']}; fr=[]
    for b in sorted(union): fr.append({'box_id':b,'service_area':b.split('-')[0],'material_type':bm[b]['物资类型'],'K18_late':b in fixed[18],**{f'K{k}_late':b in fixed[k] for k in range(19,31)}})
    pd.DataFrame(fr).to_csv(RES/'q3_fixed_hard_violation_boxes_after_fix.csv',index=False)
    if k19_dyn is not None:k19_dyn.to_csv(RES/'q3_dynamic_communication_audit.csv',index=False)
    if first is None:(RES/'q3_final_metrics.json').write_text(json.dumps({'status':'NO_CORRECTED_C1_FEASIBLE_SOLUTION','energy_component_charge_defined':False},ensure_ascii=False,indent=2),encoding='utf-8')
    else:
        K,s,d,c,e,dyn=first; s.to_csv(RES/'q3_final_schedule.csv',index=False); d.to_csv(RES/'q3_final_deliveries.csv',index=False); e.to_csv(RES/'q3_final_resource_events.csv',index=False); c.to_csv(RES/'q3_final_relay_schedule.csv',index=False); (RES/'q3_final_metrics.json').write_text(json.dumps(out[K-18],ensure_ascii=False,indent=2),encoding='utf-8')
    report={'first_complete_feasible_K':None if first is None else first[0],'all_dynamic_communication_pass':False,'dynamic_communication_status':'UNVERIFIED_STATIC_CERT_ONLY','fixed_box_intersection':len(inter),'fixed_box_union':len(union),'energy_component_charge_parameter_available':False,'energy_component_inventory_contract':6,'energy_component_inventory_connected':False,'note':'合同规定6组中继能源组件，但当前实现尚未接入该库存约束；充电功率参数和充电过程仍未完成核验。'}
    (LOG/'q3_stageC1_corrected.md').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8'); print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
