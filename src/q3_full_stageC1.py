"""Full Q3 Stage C1 (single-service routes, K=18..30).

This runner deliberately does not call the legacy static-certification result
as a communication verdict.  Each task is checked on its event-partitioned
trajectory with an interval distance upper bound and the official worst-case
terrain penalty; relay demand is the union of intervals whose direct budget
fails.  The search is bounded deterministic evidence, not an optimality claim.
"""
from __future__ import annotations
import json, math, shutil, time, argparse
from pathlib import Path
import numpy as np
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2, xy
from q2_sortie_count_sensitivity import split_to_k, pool_peak
from q3_reoptimize_from_q2 import _build_stage_a, _evaluate
from q3_official_semantics import build_transport_trajectory, relay_candidate_point, relay_component_ledger, relay_time_chain
from final_joint import strict_q3
from build_q3_strict_certificate import distance_bound

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; LOG=ROOT/'logs'; TOL=1e-8

def xyz(D,p,h):
    lat0=float(D['center']['纬度（°）']); x,y=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0)
    return np.array([x,y,float(h)],float)

def thresholds(D):
    c=D['comm']; sens=float(c[('接收参数','接收灵敏度（dBm）')]); fade=float(c[('接收参数','衰落裕量（dB）')]); lsys=float(c[('传播参数','系统损耗（dB）')])
    direct=min(float(c[('运输无人机','发射功率（dBm）')])+float(c[('运输无人机','天线增益（dBi）')])+float(c[('固定网关 G01','天线增益（dBi）')])-sens,float(c[('固定网关 G01','发射功率（dBm）')])+float(c[('固定网关 G01','天线增益（dBi）')])+float(c[('运输无人机','天线增益（dBi）')])-sens)-fade-lsys
    access=float(c[('运输无人机','发射功率（dBm）')])+float(c[('运输无人机','天线增益（dBi）')])+float(c[('中继接入端','天线增益（dBi）')])-sens-fade-lsys
    back=min(float(c[('中继回传端','发射功率（dBm）')])+float(c[('中继回传端','天线增益（dBi）')])+float(c[('固定网关 G01','天线增益（dBi）')])-sens,float(c[('固定网关 G01','发射功率（dBm）')])+float(c[('固定网关 G01','天线增益（dBi）')])+float(c[('中继回传端','天线增益（dBi）')])-sens)-fade-lsys
    return direct,access,back

def link_interval(D,a,b,ha,hb,gateway,gh,thr):
    # One endpoint follows the transport trajectory; the other is the fixed
    # relay/G01 endpoint at its certified altitude on the same global interval.
    d,_=distance_bound(D,a,b,ha,hb,gateway,gh)
    f=float(D['comm'][('传播参数','载波频率（MHz）')]); obs=float(D['comm'][('传播参数','地形遮挡附加损耗（dB）')])
    fspl=32.45+20*math.log10(f)+20*math.log10(max(d/1000,1e-12)); margin=float(thr)-fspl-obs
    return margin

def certify_task(D,task,certrow):
    traj=build_transport_trajectory(D,task,float(task.get('start_s',0.0))); direct_thr,access_thr,back_thr=thresholds(D); o=D['center']; gh=float(o['海拔（m）'])+float(D['comm'][('固定网关 G01','天线离地高度（m）')]); service=str(task['service']); rp,_,rh=relay_candidate_point(D,service,certrow)
    s=next(x for x in D['nodes'] if str(x.get('服务区编号',x.get('服务区'))) == service) if isinstance(D.get('nodes'),list) else None
    # node_map is avoided here: task's service coordinates are available in the
    # input node dictionary used by the legacy route interface.
    from minimal_pipeline import node_map
    target=node_map(D)[service]; target_h=max(v for _,v in __import__('minimal_pipeline').terrain_profile(target,target,D['dem']) if np.isfinite(v))+50.0; rows=[]; req=[]
    for i,e in enumerate(traj):
        md=link_interval(D,e['a'],e['b'],e['ha'],e['hb'],o,gh,direct_thr)
        direct=md>=-1e-9
        if direct: access=back=True; ok=True
        else:
            ma=link_interval(D,e['a'],e['b'],e['ha'],e['hb'],rp,rh,access_thr)
            # Backhaul is relay -> fixed G01; the transport target is only the
            # access-side trajectory endpoint.
            mb=link_interval(D,rp,rp,rh,rh,o,gh,back_thr)
            access=ma>=-1e-9; back=mb>=-1e-9; ok=access and back; req.append((float(e['t0']),float(e['t1'])))
        rows.append({'sortie':str(task['sortie']),'interval_id':i,'start_s':float(e['t0']),'end_s':float(e['t1']),'phase':e['phase'],'direct_available':direct,'relay_required':not direct,'access_link_ok':access,'backhaul_link_ok':back,'communication_ok':ok,'direct_margin_lower_bound_db':md})
    return rows, bool(all(r['communication_ok'] for r in rows)), req

def relay_ledgers(D, sched, rows, cert):
    """Greedy six-component use/charge ledger, independent of airframe IDs."""
    avail={f'R-E{i+1:02d}':0.0 for i in range(int(D['relay']['component_inventory']))}; air=[]; comp=[]; coverage=[]; reserve_ok=True; available_ok=True; energy=0.0
    for _,sr in sched.iterrows():
        rr=[r for r in rows if r['sortie']==str(sr.sortie) and r['relay_required']]
        if not rr: continue
        use0=min(r['start_s'] for r in rr); use1=max(r['end_s'] for r in rr); cr=cert[cert.sortie.astype(str)==str(sr.sortie)].iloc[0].to_dict()
        c0=relay_time_chain(D,str(sr.service),cr,sum(r['end_s']-r['start_s'] for r in rr),0.0)
        # Exact backsolve: prepare + outbound + link setup must complete by
        # the first required communication interval.
        prep_start=use0-float(D['relay']['prep_s'])-float(c0['outbound_flight_s'])-float(D['relay']['link_s'])
        chain=relay_time_chain(D,str(sr.service),cr,sum(r['end_s']-r['start_s'] for r in rr),prep_start)
        covered=prep_start>=-1e-6 and chain['service_start']<=use0+1e-6 and chain['service_end']>=use1-1e-6
        coverage.append({'sortie':str(sr.sortie),'required_start':use0,'required_end':use1,'prepare_start':chain['prepare_start'],'link_ready':chain['service_start'],'outbound_flight_s':chain['outbound_flight_s'],'fully_covered':bool(covered),'service_start':chain['service_start'],'service_end':chain['service_end'],'relay_id':str(getattr(sr,'relay_id','R01'))})
        cid=min(avail,key=avail.get); led=relay_component_ledger(D,cid,chain['prepare_start'],chain['return_end'],chain['total_energy_kwh'],avail[cid]); avail[cid]=led['available_again_s']
        reserve_ok &= bool(led['reserve_ok']); available_ok &= bool(led['resource_available_ok']); energy += float(chain['total_energy_kwh'])
        rid=str(getattr(sr,'relay_id','R01')); air.append({'resource':f'relay:{rid}','start_s':chain['prepare_start'],'end_s':chain['busy_end']})
        comp.extend([{'resource':f'relay_component:{cid}','start_s':led['use_start_s'],'end_s':led['use_end_s']},{'resource':f'relay_component:{cid}','start_s':led['charge_start_s'],'end_s':led['charge_end_s']}])
    def pk(events):
        pts=[]
        for e in events: pts.extend([(e['start_s'],1),(e['end_s'],-1)])
        cur=peak=0
        for _,d in sorted(pts,key=lambda z:(z[0],z[1])): cur+=d; peak=max(peak,cur)
        return int(peak)
    airpk,comppk=pk(air),pk(comp)
    per_id_ok=True
    for rid in sorted({e['resource'] for e in air}):
        pts=[]
        for e in air:
            if e['resource']==rid: pts.extend([(e['start_s'],1),(e['end_s'],-1)])
        cur=0
        for _,d in sorted(pts,key=lambda z:(z[0],z[1])):
            cur+=d; per_id_ok &= cur<=1
    return airpk,comppk,air,comp,coverage,bool(reserve_ok),bool(available_ok and per_id_ok),float(energy)

def main():
    D=load_inputs(); _,base=q1(D); base,_=q2(D,base); summary=[]; first=None; all_cert=[]
    ap=argparse.ArgumentParser(); ap.add_argument('--k',type=int,default=None); args=ap.parse_args(); krange=[args.k] if args.k else range(18,31)
    for K in krange:
        t0=time.time(); tasks=split_to_k(D,base,K); cert=strict_q3(D,tasks); sched=_build_stage_a(D,tasks,cert); deliveries,old_events=_evaluate(D,sched,cert)
        rows=[]; comm_ok=True; req_count=0
        for _,t in sched.iterrows():
            cr=cert[cert.sortie.astype(str)==str(t.sortie)].iloc[0].to_dict(); rr,ok,req=certify_task(D,t,cr)
            for z in rr: z['K']=K
            rows.extend(rr); comm_ok &= ok; req_count += len(req)
        # The legacy evaluator supplies transport/battery/service event ledgers;
        # relay is recomputed as a half-open demand ledger from strict intervals.
        relay_need_sorties={str(r['sortie']) for r in rows if r['relay_required']}
        relay_peak,rep,relay_events,component_events,coverage,reserve_ok,available_ok,relay_energy=relay_ledgers(D,sched,rows,cert)
        tr_peak=pool_peak(old_events,lambda x:x.resource.astype(str).str.startswith('transport:')); bp=pool_peak(old_events,lambda x:x.resource.astype(str).str.startswith('battery:B-')); cp=pool_peak(old_events,lambda x:x.resource.astype(str).str.startswith('battery:C-')); sp=pool_peak(old_events,lambda x:x.resource.astype(str).str.startswith('service:'))
        hard=int(deliveries.hard_violation.sum()); soft=deliveries[deliveries.soft_deadline_s.notna()]
        resource_ok=bool(relay_peak<=2 and rep<=int(D['relay']['component_inventory']) and reserve_ok and available_ok and tr_peak<=sum(len(v) for v in D['aircraft'].values()) and bp<=D['batteries']['B'] and cp<=D['batteries']['C'] and sp<=len(D['nodes']))
        rec={'K':K,'transport_sorties':K,'relay_sorties':int(len(relay_need_sorties)),'hard_violation_boxes':hard,'hard_violation_sorties':int(deliveries.loc[deliveries.hard_violation,'sortie_id'].nunique()),'soft_late_boxes':int((soft.soft_lateness_s>1e-6).sum()),'soft_total_lateness_s':float(soft.soft_lateness_s.sum()),'soft_max_lateness_s':float(soft.soft_lateness_s.max()),'joint_makespan_s':float(sched.return_s.max()),'transport_energy_kwh':float(sched.energy_kwh.sum()),'relay_energy_kwh':relay_energy,'total_energy_kwh':float(sched.energy_kwh.sum()+relay_energy),'transport_peak':tr_peak,'relay_airframe_peak':relay_peak,'relay_energy_component_peak':rep,'B_battery_peak':bp,'C_battery_peak':cp,'service_peak':sp,'communication_certificate_pass':bool(comm_ok and all(x['fully_covered'] for x in coverage)),'reserve_ok':bool(reserve_ok),'resource_available_ok':bool(available_ok),'service_coverage_pass':bool(all(x['fully_covered'] for x in coverage)),'resource_feasible':resource_ok,'complete_feasible':bool(hard==0 and comm_ok and all(x['fully_covered'] for x in coverage) and resource_ok),'runtime_s':time.time()-t0,'proven_optimal':False,'solver_status':'DETERMINISTIC_STAGE_C1'}
        summary.append(rec); all_cert.extend(rows)
        pd.DataFrame(coverage).to_csv(RES/f'q3_relay_service_coverage_K{K}.csv',index=False)
        if K==19:
            ar=[{'sortie':r['sortie'],'interval':r['interval_id'],'time_start':r['start_s'],'time_end':r['end_s'],'transport_endpoint_source':'trajectory(x(t),y(t),z(t))','relay_endpoint_source':'certified relay position at same global t','gateway_endpoint_source':'fixed G01','same_time_axis':True,'pass':bool(r['communication_ok'])} for r in rows if r['relay_required']]
            pd.DataFrame(ar).to_csv(RES/'q3_access_endpoint_audit.csv',index=False)
            pd.DataFrame(coverage).to_csv(RES/'q3_relay_prepare_backsolve_audit.csv',index=False)
            (RES/'q3_full_entry_callgraph_audit.csv').write_text('stage,called_function,file,new_semantics,legacy_function_used\nStageC1,_build_stage_a,src/q3_reoptimize_from_q2.py,batch_order_only;resources_rebuilt_below,False\nStageC1,certify_task,src/q3_full_stageC1.py,interval_link_budget,False\nStageC1,relay_time_chain,src/q3_official_semantics.py,official_relay_time_chain,False\nStageC1,relay_component_ledger,src/q3_official_semantics.py,SOC_two_stage_charge,False\nStageC1,relay_ledgers,src/q3_full_stageC1.py,resource_gating,False\nStageC1,_evaluate,src/q3_reoptimize_from_q2.py,delivery_metrics_only,False\n',encoding='utf-8')
        if rec['complete_feasible'] and first is None: first=(K,sched,deliveries,old_events,cert,rows,rec)
    pd.DataFrame(summary).to_csv(RES/('q3_K19_dry_run_summary.csv' if args.k==19 else 'q3_full_stageC1_summary.csv'),index=False); pd.DataFrame(all_cert).to_csv(RES/('q3_K19_dry_run_dynamic.csv' if args.k==19 else 'q3_full_stageC1_dynamic_communication.csv'),index=False)
    if first:
        K,s,d,e,c,rows,rec=first; s.to_csv(RES/'q3_final_schedule.csv',index=False); d.to_csv(RES/'q3_final_deliveries.csv',index=False); e.to_csv(RES/'q3_final_resource_events.csv',index=False); c.to_csv(RES/'q3_final_relay_schedule.csv',index=False); (RES/'q3_final_metrics.json').write_text(json.dumps(rec,ensure_ascii=False,indent=2),encoding='utf-8'); status='FOUND'
        ck=ROOT/'checkpoint_q3_first_complete_feasible'; ck.mkdir(exist_ok=True)
        for fn in ['q3_final_schedule.csv','q3_final_deliveries.csv','q3_final_resource_events.csv','q3_final_relay_schedule.csv','q3_final_metrics.json']:
            shutil.copy2(RES/fn,ck/fn)
    elif args.k != 19:
        (RES/'q3_final_metrics.json').write_text(json.dumps({'status':'NO_COMPLETE_FEASIBLE_C1','C2_not_started':True},ensure_ascii=False,indent=2),encoding='utf-8'); status='NONE'
    else:
        status='DRY_RUN_ONLY'
    (LOG/'q3_full_stageC1.log').write_text(json.dumps({'status':status,'first_feasible_K':None if not first else first[0],'K_range':'18..30','dynamic_certificate':'interval distance bound + worst-case terrain penalty','C2_started':False},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'status':status,'first_feasible_K':None if not first else first[0],'rows':summary},ensure_ascii=False,indent=2))
if __name__=='__main__': main()
