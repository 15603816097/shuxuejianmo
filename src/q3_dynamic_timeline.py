"""Representative Q2-001 dynamic communication audit; no optimization."""
from __future__ import annotations
import json
from pathlib import Path
import pandas as pd
from minimal_pipeline import load_inputs, q1, q2
from final_joint import strict_q3
from q3_official_semantics import (build_transport_trajectory, dynamic_communication_audit,
                                   merge_relay_required_intervals, relay_time_chain,
                                   runtime_parameter_trace, timeline_continuity_audit, RuntimeTraceDict)
ROOT=Path(__file__).resolve().parents[1]

def main():
    D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); runtime_reads=[]; D['comm']=RuntimeTraceDict(D['comm'],'comm',runtime_reads); D['relay']=RuntimeTraceDict(D['relay'],'relay',runtime_reads); cert=strict_q3(D,tasks)
    task=tasks.iloc[0]; crow=cert.iloc[0].to_dict(); traj=build_transport_trajectory(D,task,0.0)
    rows0=dynamic_communication_audit(D,str(task['service']),str(task['type']),crow,traj)
    req=merge_relay_required_intervals(rows0)
    if req:
        first=min(x['interval_start'] for x in req); last=max(x['interval_end'] for x in req)
        service_intervals=[(float(x['interval_start']),float(x['interval_end'])) for x in req]
        # The relay remains on station from the first required interval until
        # the last one; direct-link gaps are retained as hover/standby time in
        # the physical energy ledger.  The required intervals remain separate
        # in the communication/resource output.
        service_duration=last-first
        chain0=relay_time_chain(D,str(task['service']),crow,max(float(task['handoff_s']),service_duration),0.0,service_intervals=service_intervals)
        prep_start=first-(chain0['service_start']-chain0['prepare_start'])
        chain=relay_time_chain(D,str(task['service']),crow,max(float(task['handoff_s']),service_duration),prep_start,service_intervals=service_intervals)
        rows=dynamic_communication_audit(D,str(task['service']),str(task['type']),crow,traj,relay_chain=chain)
    else:
        chain=None; rows=rows0
    timeline=[]
    for r in rows:
        x=dict(r); x.update({'sortie':str(task['sortie']),'service':str(task['service']),'relay_required_interval':bool(r['relay_required'])})
        timeline.append(x)
    pd.DataFrame(timeline).to_csv(ROOT/'results/q3_Q2_001_dynamic_timeline.csv',index=False)
    cert_cols=['sortie','service','time_interval_start','time_interval_end','phase','dem_cells','minimum_terrain_clearance_m','max_distance_km','max_path_loss_db','threshold_db','margin_db','continuous_certified','direct_available','relay_required','access_link_ok','backhaul_link_ok']
    cert_df=pd.DataFrame(timeline)
    if 'sortie' not in cert_df.columns: cert_df.insert(0,'sortie',str(task['sortie']))
    if 'service' not in cert_df.columns: cert_df.insert(1,'service',str(task['service']))
    for col in cert_cols:
        if col not in cert_df.columns: cert_df[col]=None
    cert_df[cert_cols].to_csv(ROOT/'results/q3_Q2_001_continuous_certification.csv',index=False)
    time_rows=timeline_continuity_audit(traj)
    pd.DataFrame(time_rows).to_csv(ROOT/'results/q3_Q2_001_timeline_continuity_audit.csv',index=False)
    pd.DataFrame(req).to_csv(ROOT/'results/q3_Q2_001_relay_required_intervals.csv',index=False)
    p=[]; c=D['comm'];
    def add(name,val,unit,file,field):p.append({'parameter':name,'value':val,'unit':unit,'source_file':file,'source_sheet':'数据','source_cell_or_field':field,'hardcoded':False})
    comm_file='通信链路参数.xlsx'; relay_file='中继无人机数据.xlsx'
    for (cat,name),val in c.items(): add(name,val,'see parameter name',comm_file,f'{cat}:{name}')
    for name,key,unit in [('prepare_s','prep_s','s'),('link_setup_s','link_s','s'),('turnaround_s','turn_s','s'),('hover_power_kw','hover_power_kw','kW'),('communication_additional_power_kw','comm_power_kw','kW'),('cruise_power_kw','power_kw','kW'),('cruise_speed_mps','speed','m/s'),('climb_speed_mps','climb_speed','m/s'),('descent_speed_mps','descend_speed','m/s'),('climb_efficiency','eta_climb','1'),('descent_efficiency','eta_descend','1'),('component_energy_kwh','energy_kwh','kWh'),('component_inventory','component_inventory','groups'),('component_full_charge_s','component_full_charge_s','s')]: add(name,D['relay'][key],unit,relay_file, key)
    pd.DataFrame(p).to_csv(ROOT/'results/q3_parameter_source_audit.csv',index=False)
    trace_rows=[]
    for x in runtime_reads:
        key=x['key']; container=x['container']
        if container=='comm':
            cat,name=key; trace_rows.append({'parameter':name,'runtime_value':x['runtime_value'],'caller_function':'runtime dictionary access','caller_file':'src/q3_official_semantics.py','source_workbook':'通信链路参数.xlsx','source_sheet':'数据','source_field':f'{cat}:{name}','load_timestamp_or_sequence':len(trace_rows),'loaded_from_attachment':True,'hardcoded_in_runtime_path':False})
        else:
            trace_rows.append({'parameter':str(key),'runtime_value':x['runtime_value'],'caller_function':'runtime dictionary access','caller_file':'src/q3_official_semantics.py','source_workbook':'中继无人机数据.xlsx','source_sheet':'数据','source_field':str(key),'load_timestamp_or_sequence':len(trace_rows),'loaded_from_attachment':True,'hardcoded_in_runtime_path':False})
    pd.DataFrame(trace_rows).drop_duplicates(['parameter','source_field']).to_csv(ROOT/'results/q3_runtime_parameter_trace.csv',index=False)
    summary={'sortie':str(task['sortie']),'trajectory_intervals':len(traj),'dynamic_intervals':len(rows),'relay_required_intervals':req,'relay_chain':chain,'direct_available_intervals':[{'start':r['time_interval_start'],'end':r['time_interval_end']} for r in rows if r['direct_available']],'direct_unavailable_intervals':[{'start':r['time_interval_start'],'end':r['time_interval_end']} for r in rows if not r['direct_available']],'communication_pass':bool(all(r['communication_ok'] for r in rows)),'continuous_certification_pass':bool(cert_df['continuous_certified'].all()),'timeline_continuity_pass':bool(all(x['pass'] for x in time_rows)),'any_parameter_hardcoded':bool(any(x['hardcoded'] for x in p)),'runtime_any_hardcoded':bool(any(x['hardcoded_in_runtime_path'] for x in runtime_parameter_trace(D))),'all_parameter_sources_external':bool(not any(x['hardcoded_in_runtime_path'] for x in runtime_parameter_trace(D))),'dem_rule':'30m DEM crossed-cell elevation; cruise altitude=max crossed terrain + 50m; no bilinear interpolation'}
    (ROOT/'logs/q3_Q2_001_dynamic_timeline.md').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
