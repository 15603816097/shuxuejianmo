from __future__ import annotations
import ast, json, hashlib, math
from pathlib import Path
import pandas as pd
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import q3_final_pipeline as pipe
import q3_five_service_hover_refinement as comm

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SC='FEASIBILITY_RESTORED_0P1DB'; DELTA=.1
SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'

def main():
    data=pipe.load_inputs(); schedule=pd.read_csv(RES/'q2_sensitivity_19_27/q2_sens_K19_schedule.csv'); schedule['box_ids']=schedule.box_ids.map(ast.literal_eval)
    original=comm.thresholds
    def wrapped_thresholds(D):
        td,ta,tb=original(D); return td,ta,tb+DELTA
    # Both final-pipeline threshold references and the semantics-core runtime
    # import this same accessor, so this is a single uniform sensitivity hook.
    pipe.thresholds=wrapped_thresholds; comm.thresholds=wrapped_thresholds
    trace=[]; tasks=[]; boxes=[]; transport_events=[]; battery_events=[]; relay_events=[]; component_events=[]
    for idx, r in enumerate(schedule.sort_values('start_s').itertuples(index=False),1):
        print(f"replay {idx}/19 {r.sortie}", flush=True)
        out=pipe.evaluate(data,(r.service,),list(r.box_ids),r.type,start_time=float(r.start_s))
        invocation=f'REPLAY-{idx:03d}'
        trace.append({'invocation_id':invocation,'sortie_id':r.sortie,'pipeline_called':True,'pipeline_entrypoint':'q3_final_pipeline.evaluate','pipeline_version_sha256':SHA,'delta_link_budget_db':DELTA})
        tl=out['timeline']; offset=float(r.start_s)
        for seg in tl['segments']:
            for ph in seg['phases']:
                transport_events.append({'resource_id':str(r.aircraft),'task_id':r.sortie,'busy_start':float(ph['start_s'])+offset,'busy_end':float(ph['end_s'])+offset,'phase':ph['phase']})
        # The frozen evaluator exposes the authoritative transport interval and
        # energy, but does not own the Q2 battery allocator.  Record the actual
        # replay use interval and mark charging as unavailable rather than
        # inventing a post-hoc charge result.
        battery_events.append({'battery_id':str(r.battery),'task_id':r.sortie,
                               'use_start':float(tl['events'][0]['start_s'])+offset,
                               'use_end':float(tl['makespan_s'])+offset,
                               'energy_used_kwh':float(out['transport_energy_kwh']),
                               'charge_start':float(tl['makespan_s'])+offset,
                               'charge_end':float('nan'),
                               'next_available':float('nan'),
                               'battery_ledger_from_replay':False})
        hard_ok=True; soft_late=0.; hard_count=0
        for d in out['deadline_ledger']:
            delivery=float(d['delivery_time_s'])+offset; hard=bool(d['hard'] and not d['pass']); hard_ok &= not hard; hard_count += int(hard); soft_late += max(0.,delivery-float(d['deadline_s'])) if not d['hard'] else 0.
            boxes.append({'scenario':SC,'invocation_id':invocation,'sortie_id':r.sortie,'box_id':d['box_id'],'delivery_time_s':delivery,'deadline_s':d['deadline_s'],'hard':d['hard'],'hard_violation':hard,'soft_lateness_s':max(0.,delivery-float(d['deadline_s'])) if not d['hard'] else 0.})
        for rr in out.get('relay_timeline',[]): relay_events.append({'resource_id':rr.get('relay_id',''),'task_id':r.sortie,'busy_start':float(rr['prepare_start'])+offset,'busy_end':float(rr['return_end'])+offset,'service_start':float(rr['service_start'])+offset,'service_end':float(rr['service_end'])+offset})
        for cc in out.get('component_ledger',[]): component_events.append({'component_id':cc.get('component_id',''),'task_id':r.sortie,'use_start':float(cc.get('use_start_s',0))+offset,'use_end':float(cc.get('use_end_s',0))+offset,'charge_start':float(cc.get('charge_start_s',0))+offset,'charge_end':float(cc.get('available_again_s',0))+offset,'soc_after':cc.get('soc_end',None)})
        # q3_final_pipeline exposes required_reserve as usable mission energy;
        # compare mission consumption against it (the evaluator's own gate).
        reserve_ok = bool(float(out['transport_energy_kwh']) <= float(out['required_reserve_kwh']) + 1e-6)
        cands=[x for x in out['hover_candidates'] if x is not None]
        if cands:
            wa=min(float(x.get('access_margin',float('nan'))) for x in cands)
            wb=min(float(x.get('backhaul_margin',float('nan'))) for x in cands)
            wj=min(float(x.get('joint_margin',float('nan'))) for x in cands)
        else: wa=wb=wj=float('nan')
        geom=bool(out['status']['COMMUNICATION_GEOMETRY_PASS'])
        if not geom:
            relay_status='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'; relay_energy=float('nan'); relay_reserve='NOT_EVALUATED'; comp_status='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'; comp_ok='NOT_EVALUATED'; relay_resource='NOT_EVALUATED_AFTER_GEOMETRY_FAIL'; relay_chain='NOT_EVALUATED'
        elif not out['relay_required_blocks']:
            relay_status='NOT_REQUIRED'; relay_energy=0.0; relay_reserve=True; comp_status='NOT_REQUIRED'; comp_ok=True; relay_resource='NOT_REQUIRED'; relay_chain=True
        else:
            relay_status='EVALUATED'; relay_energy=float(out['relay_energy_kwh']); relay_reserve=bool(out['status']['RELAY_CHAIN_PASS']); comp_status='EVALUATED'; comp_ok=bool(out['status']['COMPONENT_PASS']); relay_resource=out['relay_resource_status']; relay_chain=bool(out['status']['RELAY_CHAIN_PASS'])
        failure=''
        if not out['status']['TRANSPORT_FEASIBLE']: failure='PAYLOAD_VOLUME' if not (out['mass_kg']<=data['drones'][r.type].max_mass and out['volume_m3']<=data['drones'][r.type].volume) else 'TRANSPORT_ENERGY_RESERVE'
        elif hard_count: failure='HARD_DEADLINE'
        elif not geom: failure='COMMUNICATION_GEOMETRY'
        elif not relay_chain: failure='RELAY_CHAIN'
        elif not comp_ok: failure='COMPONENT_RESOURCE'
        tasks.append({'scenario':SC,'invocation_id':invocation,'sortie_id':r.sortie,'route':f'O01->{r.service}->O01','box_count':len(r.box_ids),'start_time':r.start_s,'transport_pipeline_called':True,'transport_feasible':out['status']['TRANSPORT_FEASIBLE'],'transport_energy':out['transport_energy_kwh'],'transport_reserve_ok':reserve_ok,'deadline_recomputed':True,'hard_deadline_ok':hard_ok,'hard_violation_count':hard_count,'soft_lateness':soft_late,'communication_pipeline_called':True,'relay_required_interval_count':len(out['relay_required_blocks']),'worst_access_margin':wa,'worst_backhaul_margin':wb,'worst_joint_margin':wj,'communication_ok':geom,'relay_pipeline_called':True,'relay_required':bool(out['relay_required_blocks']),'relay_status':relay_status,'relay_energy':relay_energy,'relay_reserve_ok':relay_reserve,'relay_chain_ok':relay_chain,'component_pipeline_called':True,'component_status':comp_status,'component_ok':comp_ok,'relay_resource_status':relay_resource,'route_feasible':bool(out['status']['ROUTE_FEASIBLE']),'failure_stage':failure})
    td=pd.DataFrame(tasks); bd=pd.DataFrame(boxes); tr=pd.DataFrame(transport_events); ba=pd.DataFrame(battery_events); re=pd.DataFrame(relay_events); ce=pd.DataFrame(component_events); ct=pd.DataFrame(trace)
    # All returned intervals are checked for overlap within each shared resource.
    def peak_and_overlap(df, start='busy_start', end='busy_end', idcol='resource_id'):
        if df.empty:return {},False
        peaks={}; bad=False
        for rid,g in df.groupby(idcol):
            ints=sorted((float(a),float(b)) for a,b in zip(g[start],g[end])); active=[]; p=0
            for a,b in ints:
                bad |= any(a < e-1e-6 for e in active)
                active=[e for e in active if e > a+1e-6]; active.append(b); p=max(p,len(active))
            peaks[rid]=p
        return peaks,bad
    tp,tbad=peak_and_overlap(tr); rp,rbad=peak_and_overlap(re); cp,cbad=peak_and_overlap(ce,'use_start','use_end','component_id')
    td.to_csv(RES/'q3_0p1db_replay_task_audit.csv',index=False,encoding='utf-8-sig'); ct.to_csv(RES/'q3_0p1db_pipeline_call_trace.csv',index=False,encoding='utf-8-sig'); bd.to_csv(RES/'q3_0p1db_box_recomputed.csv',index=False,encoding='utf-8-sig'); tr.to_csv(RES/'q3_0p1db_transport_ledger.csv',index=False,encoding='utf-8-sig'); ba.to_csv(RES/'q3_0p1db_battery_ledger.csv',index=False,encoding='utf-8-sig'); re.to_csv(RES/'q3_0p1db_relay_ledger.csv',index=False,encoding='utf-8-sig'); ce.to_csv(RES/'q3_0p1db_component_ledger.csv',index=False,encoding='utf-8-sig')
    battery_pass=False  # frozen evaluator does not expose a transport battery ledger
    assigned=[]
    for rr in schedule.itertuples(index=False): assigned.extend(list(rr.box_ids))
    assignment_dups=len(assigned)-len(set(assigned)); assigned_n=len(set(assigned))
    all_route=bool(len(ct)==19 and len(td)==19 and td.route_feasible.all())
    hard_violations=int(bd.groupby('box_id').hard_violation.max().sum()) if not bd.empty else 0
    soft_late_boxes=int((bd.groupby('box_id').soft_lateness_s.max()>0).sum()) if not bd.empty else 0
    total_soft=float(bd.groupby('box_id').soft_lateness_s.max().sum()) if not bd.empty else 0.
    max_soft=float(bd.groupby('box_id').soft_lateness_s.max().max()) if not bd.empty else 0.
    summary={'scenario':SC,'delta_link_budget_db':DELTA,'pipeline_sha256':SHA,'expected_pipeline_calls':19,'actual_pipeline_calls':len(ct),'route_evaluations_completed':len(td),'boxes_assigned':assigned_n,'duplicate_boxes':assignment_dups,'missing_boxes':int(80-assigned_n),'hard_violations':hard_violations,'soft_late_boxes':soft_late_boxes,'total_soft_lateness':total_soft,'max_soft_lateness':max_soft,'joint_makespan':float(max(schedule.return_s.max(), re.busy_end.max() if not re.empty else 0)),'transport_energy':float(td.transport_energy.sum()),'relay_energy':float(td.relay_energy.dropna().sum()),'total_energy':float(td.transport_energy.sum()+td.relay_energy.dropna().sum()),'transport_sorties':19,'relay_sorties':int(td.relay_required.sum()),'transport_peak':max(tp.values()) if tp else 0,'B_battery_peak':0,'C_battery_peak':0,'relay_peak':max(rp.values()) if rp else 0,'component_peak':max(cp.values()) if cp else 0,'transport_resource_pass':not tbad,'battery_resource_pass':battery_pass,'relay_resource_pass':not rbad,'component_resource_pass':not cbad,'communication_pass':bool(td.communication_ok.all()),'COMPLETE_FEASIBLE':False}
    summary['COMPLETE_FEASIBLE']=bool(all_route and assigned_n==80 and assignment_dups==0 and hard_violations==0 and td.communication_ok.all() and not tbad and battery_pass and not rbad and not cbad)
    pd.DataFrame([summary]).to_csv(RES/'q3_0p1db_replay_final_summary.csv',index=False,encoding='utf-8-sig'); print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
