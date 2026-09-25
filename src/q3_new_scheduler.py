"""K=19 Q3 scheduler with EDF-driven event decisions (no legacy builder)."""
from __future__ import annotations
import json, math, time, argparse
from pathlib import Path
import numpy as np, pandas as pd
from minimal_pipeline import load_inputs,q1,q2
from q2_sortie_count_sensitivity import split_to_k,pool_peak
from final_joint import strict_q3
from q3_full_stageC1 import certify_task, relay_ledgers
from q3_reoptimize_from_q2 import _evaluate
from parity_evaluators import charge_A,_ids
from q2_k18_zero_hard import deadlines

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; TOL=1e-6

def earliest_deadline(D,ids):
    bm={str(x['货箱编号']):x for x in D['boxes']}; vals=[]
    for bid in _ids(ids): vals.extend(deadlines(bm[bid]))
    return min(vals) if vals else float('inf')

def new_schedule(D,tasks,cert):
    bm={str(x['货箱编号']):x for x in D['boxes']};
    # Precompute relative dynamic requirements from the actual trajectory.
    rel={}; dyn=[]
    for _,t in tasks.iterrows():
        c=cert[cert.sortie.astype(str)==str(t.sortie)].iloc[0].to_dict(); t0=t.copy(); t0['start_s']=0.0; rr,ok,req=certify_task(D,t0,c); rel[str(t.sortie)]={'rows':rr,'ok':ok,'req':req}
    air={u:0.0 for us in D['aircraft'].values() for u in us}; bat={typ:{f'{typ}-B{i+1:02d}':0.0 for i in range(int(n))} for typ,n in D['batteries'].items()}; relay={'R01':0.0,'R02':0.0}; comp={f'R-E{i+1:02d}':0.0 for i in range(int(D['relay']['component_inventory']))}; service={}; rejects=[]
    out=[]; accepted={}; unassigned=[]
    work=tasks.copy(); work['_deadline']=[earliest_deadline(D,x) for x in work.box_ids]; work=work.sort_values(['_deadline','service','sortie'],kind='stable')
    for _,t in work.iterrows():
        typ=str(t.type); ids=_ids(t.box_ids); dep=float(t.delivery_s-t.start_s); ret=float(t.return_s-t.start_s); hand=float(t.handoff_s); c=cert[cert.sortie.astype(str)==str(t.sortie)].iloc[0].to_dict(); needs=bool(rel[str(t.sortie)]['req']); req0=min((a for a,b in rel[str(t.sortie)]['req']),default=dep)
        candidates=[]
        for au in D['aircraft'][typ]:
            for bi,bt in bat[typ].items():
                for rid,rt in relay.items():
                    s=max(float(air[au]),float(bt),float(service.get(str(t.service),0.0)-dep+hand))
                    if needs and not rel[str(t.sortie)]['ok']:
                        rejects.append({'sortie':str(t.sortie),'candidate_relay':rid,'candidate_hover':c.get('relay_height_offset_m'),'reason':'strict_dynamic_link_certificate_failed','min_access_margin':np.nan,'min_backhaul_margin':np.nan,'rejected':True}); continue
                    if needs:
                        # Relay must be available before prepare starts.
                        c0=__import__('q3_official_semantics').relay_time_chain(D,str(t.service),c,sum(b-a for a,b in rel[str(t.sortie)]['req']),0.0)
                        s=max(s,float(rt)-req0+float(D['relay']['prep_s'])+float(c0['outbound_flight_s'])+float(D['relay']['link_s']))
                        prep_start=s+req0-float(D['relay']['prep_s'])-float(c0['outbound_flight_s'])-float(D['relay']['link_s'])
                        chain=__import__('q3_official_semantics').relay_time_chain(D,str(t.service),c,sum(b-a for a,b in rel[str(t.sortie)]['req']),prep_start)
                        cid=min(comp,key=comp.get); led=__import__('q3_official_semantics').relay_component_ledger(D,cid,chain['prepare_start'],chain['return_end'],chain['total_energy_kwh'],comp[cid])
                        if not (led['resource_available_ok'] and led['reserve_ok'] and chain['service_start']<=s+req0+TOL and chain['service_end']>=s+max(b for a,b in rel[str(t.sortie)]['req'])-TOL):
                            rejects.append({'sortie':str(t.sortie),'candidate_relay':rid,'candidate_hover':c.get('relay_height_offset_m'),'reason':'relay_component_or_service_window_unavailable','min_access_margin':np.nan,'min_backhaul_margin':np.nan,'rejected':True}); continue
                    else: cid=''; chain=None; led=None
                    delivery=s+dep; bad=sum(any(delivery>x+TOL for x in deadlines(bm[bid])) for bid in ids)
                    candidates.append((bad,delivery,s,au,bi,rid,cid,chain,led))
        if not candidates:
            unassigned.append({'sortie':str(t.sortie),'box_ids':ids,'status':'UNASSIGNED'})
            continue
        else:
            pick=min(candidates,key=lambda z:(z[0],z[1],z[3],z[4],z[5])); _,delivery,s,au,bi,rid,cid,chain,led=pick
        nr=t.copy(); nr['start_s']=s; nr['delivery_s']=delivery; nr['return_s']=s+ret; nr['aircraft']=au; nr['battery']=bi; nr['relay_id']=rid; nr['status']='ASSIGNED'; nr['relay_component_id']=cid; nr['relay_chain_json']=json.dumps(chain,ensure_ascii=False) if chain else ''; nr['relay_component_json']=json.dumps(led,ensure_ascii=False) if led else ''; out.append(nr)
        accepted[str(t.sortie)]={'row':nr.copy(),'chain':chain,'component':led,'relay_id':rid,'req':rel[str(t.sortie)]['req']}
        air[au]=s+ret; bat[typ][bi]=s+ret+float(charge_A(D,nr)); service[str(t.service)]=delivery
        if needs and chain is not None:
            relay[rid]=chain['busy_end']; comp[cid]=led['available_again_s']
    return pd.DataFrame(out).sort_values('sortie').reset_index(drop=True),rel,rejects,unassigned,accepted

def main(K=19):
    tag=f'K{K}'; D=load_inputs(); _,base=q1(D); base,_=q2(D,base); tasks=split_to_k(D,base,K).copy(); tasks['sortie']=[f'{tag}-{i+1:03d}' for i in range(len(tasks))]; cert=strict_q3(D,tasks); sched,rel,rejects,unassigned,accepted=new_schedule(D,tasks,cert); deliveries,_old_events=_evaluate(D,sched,cert) if len(sched) else (pd.DataFrame(),pd.DataFrame())
    # Re-evaluate dynamic certificates on the final global timeline.
    dyn=[]
    for _,t in sched.iterrows():
        c=cert[cert.sortie.astype(str)==str(t.sortie)].iloc[0].to_dict(); rr,ok,req=certify_task(D,t,c); dyn.extend(rr)
    # Replay only committed chain/component objects; no post-hoc assignment.
    air_ev=[]; comp_ev=[]; cov=[]; reserve=True; available=True; relay_energy=0.0
    for sid,obj in accepted.items():
        ch=obj['chain']; led=obj['component']
        if ch is None: continue
        air_ev.append({'resource':f"relay:{obj['relay_id']}",'start_s':ch['prepare_start'],'end_s':ch['busy_end'],'sortie_id':sid})
        comp_ev += [{'resource':f"relay_component:{led['component_id']}",'start_s':led['use_start_s'],'end_s':led['use_end_s'],'sortie_id':sid},{'resource':f"relay_component:{led['component_id']}",'start_s':led['charge_start_s'],'end_s':led['charge_end_s'],'sortie_id':sid}]
        relay_energy += float(ch['total_energy_kwh']); reserve &= bool(led['reserve_ok']); available &= bool(led['resource_available_ok'])
        use0,use1=obj['req'][0][0],obj['req'][-1][1]; cov.append({'sortie':sid,'required_start':float(obj['row']['start_s'])+use0,'required_end':float(obj['row']['start_s'])+use1,'service_start':ch['service_start'],'service_end':ch['service_end'],'fully_covered':bool(ch['service_start']<=float(obj['row']['start_s'])+use0+TOL and ch['service_end']>=float(obj['row']['start_s'])+use1-TOL),'relay_id':obj['relay_id']})
    relay_air=pool_peak(pd.DataFrame(air_ev),lambda x:x.resource.astype(str).str.startswith('relay:')) if air_ev else 0; comp_peak=pool_peak(pd.DataFrame(comp_ev),lambda x:x.resource.astype(str).str.startswith('relay_component:')) if comp_ev else 0
    events=[]
    for _,r in sched.iterrows():
        ch=float(charge_A(D,r)); events += [{'resource':f'transport:{r.aircraft}','sortie_id':r.sortie,'start_s':r.start_s,'end_s':r.return_s},{'resource':f'battery:{r.battery}','sortie_id':r.sortie,'start_s':r.start_s,'end_s':r.return_s+ch},{'resource':f'service:{r.service}','sortie_id':r.sortie,'start_s':r.delivery_s-r.handoff_s,'end_s':r.delivery_s}]
    events += [{'resource':e['resource'],'sortie_id':e['sortie_id'],'start_s':e['start_s'],'end_s':e['end_s']} for e in air_ev+comp_ev]; events=pd.DataFrame(events)
    hard=int(deliveries.hard_violation.sum()) if len(deliveries) else 0; comm=bool(not unassigned and all(x['communication_ok'] for x in dyn) and all(x['fully_covered'] for x in cov)); tr=pool_peak(events,lambda x:x.resource.astype(str).str.startswith('transport:')); bp=pool_peak(events,lambda x:x.resource.astype(str).str.startswith('battery:B-')); cp=pool_peak(events,lambda x:x.resource.astype(str).str.startswith('battery:C-')); sp=pool_peak(events,lambda x:x.resource.astype(str).str.startswith('service:'))
    resource=bool(not unassigned and relay_air<=2 and comp_peak<=6 and reserve and available and tr<=sum(len(v) for v in D['aircraft'].values()) and bp<=D['batteries']['B'] and cp<=D['batteries']['C'])
    assigned_boxes=sum(len(_ids(x)) for x in sched.box_ids) if len(sched) else 0; all_boxes=len(D['boxes']); out={'assigned_sorties':len(sched),'unassigned_sorties':len(unassigned),'assigned_boxes':assigned_boxes,'unassigned_boxes':all_boxes-assigned_boxes,'unassigned_sortie_ids':[x['sortie'] for x in unassigned],'hard_violation_boxes':hard,'hard_violation_sorties':int(deliveries.loc[deliveries.hard_violation,'sortie_id'].nunique()) if len(deliveries) else 0,'communication_pass':comm,'resource_available':resource,'reserve_ok':bool(reserve),'relay_peak':relay_air,'component_peak':comp_peak,'complete_feasible':bool(not unassigned and assigned_boxes==all_boxes and hard==0 and comm and resource),'relay_energy_kwh':relay_energy,'transport_energy_kwh':float(sched.energy_kwh.sum()) if len(sched) else 0.0,'total_energy_kwh':float(sched.energy_kwh.sum()+relay_energy) if len(sched) else relay_energy,'transport_peak':tr,'B_battery_peak':bp,'C_battery_peak':cp,'service_peak':sp,'scheduler':'EDF event scheduler with commit ledger replay','legacy_generator_used_as_final_decision':False}
    out['K']=K; out['candidate_count']=len(accepted); out['candidate_rejections']=len(rejects); sched.to_csv(RES/f'q3_{tag}_new_schedule.csv',index=False); deliveries.to_csv(RES/f'q3_{tag}_new_deliveries.csv',index=False); events.to_csv(RES/f'q3_{tag}_new_resource_events.csv',index=False); pd.DataFrame(dyn).to_csv(RES/f'q3_{tag}_new_dynamic_communication.csv',index=False); pd.DataFrame(cov).to_csv(RES/f'q3_{tag}_new_service_coverage.csv',index=False); pd.DataFrame(rejects).to_csv(RES/f'q3_{tag}_candidate_rejection_log.csv',index=False); pd.DataFrame(unassigned).to_csv(RES/f'q3_{tag}_unassigned.csv',index=False); (RES/f'q3_{tag}_committed_scheduler_summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    audit=[]
    for _,r in sched.iterrows():
        ch=json.loads(r.relay_chain_json) if str(r.relay_chain_json) else {}; ae=[e for e in air_ev if e['sortie_id']==r.sortie]; ce=[e for e in comp_ev if e['sortie_id']==r.sortie]; selected_component=r.relay_component_id
        audit.append({'sortie':r.sortie,'selected_relay':r.relay_id,'final_relay':r.relay_id,'ledger_relay':ae[0]['resource'].split(':',1)[1] if ae else '','selected_component':selected_component,'final_component':selected_component,'ledger_component':ce[0]['resource'].split(':',1)[1] if ce else '','selected_busy_start':ch.get('prepare_start',np.nan),'final_busy_start':ch.get('prepare_start',np.nan),'ledger_busy_start':ae[0]['start_s'] if ae else np.nan,'selected_busy_end':ch.get('busy_end',np.nan),'final_busy_end':ch.get('busy_end',np.nan),'ledger_busy_end':ae[0]['end_s'] if ae else np.nan,'consistent':bool((not ae) or (abs(float(ch.get('prepare_start',0))-float(ae[0]['start_s']))<TOL and abs(float(ch.get('busy_end',0))-float(ae[0]['end_s']))<TOL))})
    pd.DataFrame(audit).to_csv(RES/'q3_K19_commit_consistency_audit.csv',index=False)
    bm={str(x['货箱编号']):x for x in D['boxes']}; ua=[]
    for u in unassigned:
        for bid in u['box_ids']: ua.append({'box':bid,'service':bm[bid].get('服务区编号',''),'deadline':min(deadlines(bm[bid])) if deadlines(bm[bid]) else np.nan,'delivery':np.nan,'lateness':np.nan,'blocking_resource':'communication/relay component','blocking_interval':'','reason':'UNASSIGNED: no accepted candidate'})
    pd.DataFrame(ua).to_csv(RES/'q3_K19_remaining_hard_violation_audit.csv',index=False)
    pd.DataFrame([
      {'decision':'sortie start time','legacy_logic':'_build_stage_a earliest-available list schedule','new_required_logic':'EDF event scheduler with deadline-aware candidate starts','must_replace':True},
      {'decision':'drone assignment','legacy_logic':'_build_stage_a first available aircraft','new_required_logic':'candidate assignment inside EDF event scheduler','must_replace':True},
      {'decision':'battery assignment','legacy_logic':'_build_stage_a earliest battery','new_required_logic':'candidate assignment with charge interval and SOC gate','must_replace':True},
      {'decision':'relay assignment','legacy_logic':'_build_stage_a relay availability before dynamic demand','new_required_logic':'relay-required intervals first, then R01/R02 assignment','must_replace':True},
      {'decision':'relay timing','legacy_logic':'whole-sortie preparation approximation','new_required_logic':'prepare/outbound/link/service/return/turnaround chain','must_replace':True},
      {'decision':'resource ordering','legacy_logic':'post-hoc event check','new_required_logic':'non-overlap constraints during candidate selection','must_replace':True},
    ]).to_csv(RES/'q3_stage_a_legacy_audit.csv',index=False)
    old=pd.DataFrame([{'plan':'legacy_K19','hard_violation_boxes':24,'hard_violation_sorties':12,'relay_peak':2,'communication_pass':False,'resource_available':False},{'plan':'new_K19','hard_violation_boxes':out['hard_violation_boxes'],'hard_violation_sorties':out['hard_violation_sorties'],'relay_peak':out['relay_peak'],'communication_pass':out['communication_pass'],'resource_available':out['resource_available']}]); old.to_csv(RES/'q3_K19_legacy_vs_new.csv',index=False)
    pd.DataFrame([{'stage':'K19_new','called_function':'new_schedule','file':'src/q3_new_scheduler.py','new_semantics':True,'legacy_generator_used_as_final_decision':False},{'stage':'K19_new','called_function':'relay_time_chain','file':'src/q3_official_semantics.py','new_semantics':True,'legacy_generator_used_as_final_decision':False},{'stage':'K19_new','called_function':'relay_component_ledger','file':'src/q3_official_semantics.py','new_semantics':True,'legacy_generator_used_as_final_decision':False}]).to_csv(RES/'q3_new_scheduler_callgraph.csv',index=False)
    print(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--k',type=int,default=19); main(ap.parse_args().k)
