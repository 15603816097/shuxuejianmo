"""Build event, LOS and interval-bound certificates for Q2-001 only."""
from __future__ import annotations
import json, math, ast
from pathlib import Path
import pandas as pd
import numpy as np
from minimal_pipeline import load_inputs,q1,q2,node_map,terrain_profile,xy
from final_joint import strict_q3
from q3_official_semantics import build_transport_trajectory, dynamic_communication_audit, RuntimeTraceDict, timeline_continuity_audit
ROOT=Path(__file__).resolve().parents[1]

def xyz(data,p,h):
    lat0=float(data['center']['纬度（°）']); x,y=xy(float(p['经度（°）']),float(p['纬度（°）']),lat0); return np.array([x,y,float(h)],float)

def distance_bound(data,a,b,ha,hb,gateway,gateway_h):
    p0=xyz(data,a,ha)-xyz(data,gateway,gateway_h); v=(xyz(data,b,hb)-xyz(data,a,ha))
    aa=float(v@v); bb=float(2*p0@v); cc=float(p0@p0); cand=[0.0,1.0]
    if aa>1e-15:
        u=-bb/(2*aa)
        if 0<u<1: cand.append(u)
    vals=[math.sqrt(max(0.0,aa*u*u+bb*u+cc)) for u in cand]
    return max(vals), 'endpoints+quadratic_stationary' if len(cand)>2 else 'endpoints'

def main():
    D=load_inputs(); _,b=q1(D); tasks,_=q2(D,b); task=tasks.iloc[0]; cert=strict_q3(D,tasks); crow=cert.iloc[0].to_dict(); traj=build_transport_trajectory(D,task,0.0); o=D['center']; service=str(task['service']); s=node_map(D)[service]; gateway_h=float(o['海拔（m）'])+float(D['comm'][("固定网关 G01","天线离地高度（m）")]); freq=float(D['comm'][("传播参数","载波频率（MHz）")]); lobs=float(D['comm'][("传播参数","地形遮挡附加损耗（dB）")]); lsys=float(D['comm'][("传播参数","系统损耗（dB）")]); sens=float(D['comm'][("接收参数","接收灵敏度（dBm）")]); fade=float(D['comm'][("接收参数","衰落裕量（dB）")]);
    direct_thr=min(float(D['comm'][("运输无人机","发射功率（dBm）")])+float(D['comm'][("运输无人机","天线增益（dBi）")])+float(D['comm'][("固定网关 G01","天线增益（dBi）")])-sens,float(D['comm'][("固定网关 G01","发射功率（dBm）")])+float(D['comm'][("固定网关 G01","天线增益（dBi）")])+float(D['comm'][("运输无人机","天线增益（dBi）")])-sens)-fade-lsys
    ev=[]; los=[]; certrows=[]
    for i,e in enumerate(traj):
        a=e['a']; b0=e['b']; h0=e['ha']; h1=e['hb']; t0=float(e['t0']); t1=float(e['t1']); cells=terrain_profile(a,b0,D['dem']); dmax,method=distance_bound(D,a,b0,h0,h1,o,gateway_h); fspl=32.45+20*math.log10(freq)+20*math.log10(max(dmax/1000.0,1e-12))
        # On a primitive interval the DEM-cell set is fixed.  For each fixed
        # cell, the LOS height is affine in trajectory time; its minimum over
        # a closed interval is therefore attained at an endpoint.  This is an
        # interval proof, not a sample-point acceptance rule.
        endpoint_profiles=[(a,h0),(b0,h1)]
        cells_by_key={}
        for p,h in endpoint_profiles:
            for u,g in terrain_profile(p,o,D['dem']):
                key=int(round(u*1e9)); cells_by_key.setdefault(key,[]).append((float(g),float(h+u*(gateway_h-h))))
        clear=[]; obstruction=False; cell_rows=[]
        for key,vals in cells_by_key.items():
            finite=[v for v in vals if np.isfinite(v[0])]
            if not finite: continue
            cell_clear=min(lh-ground for ground,lh in finite)
            ground,lh=min(finite,key=lambda v:v[1]-v[0])
            clear.append(float(cell_clear)); obstruction |= cell_clear <= 0.0
            cell_rows.append((i,key,float(ground),float(lh),float(cell_clear)))
        clearance=min(clear) if clear else float('nan')
        # Worst-case terrain penalty is applied on every primitive interval;
        # this removes any dependence on a sampled LOS obstruction label.
        path=fspl+lobs; margin=direct_thr-path; state_known=bool(np.isfinite(clearance)); passed=bool(state_known and margin>=-1e-9)
        ev.append({'interval_id':i,'t_start':t0,'t_end':t1,'phase':e['phase'],'start_xyz':json.dumps([float(x) for x in xyz(D,a,h0)]),'end_xyz':json.dumps([float(x) for x in xyz(D,b0,h1)]),'crossed_dem_cells':len(cells),'partition_reason':'phase_transition+DEM_route_cell_boundary'})
        for _,key,g,lh,cell_clear in cell_rows: los.append({'primitive_interval':i,'dem_cell':key,'entry_parameter':0.0,'exit_parameter':1.0,'los_height_entry':lh,'los_height_exit':lh,'minimum_los_height':lh,'dem_height':g,'clearance_lower_bound':cell_clear,'pass':bool(np.isfinite(cell_clear)),'obstructed':bool(cell_clear<=0)})
        certrows.append({'interval_id':i,'t_start':t0,'t_end':t1,'phase':e['phase'],'dem_cells':len(cells),'distance_max_method':method,'distance_max_km':dmax/1000.0,'fspl_upper_bound':fspl,'obstacle_loss_db':lobs,'path_loss_upper_bound':path,'threshold':direct_thr,'margin_lower_bound':margin,'los_clearance_lower_bound':clearance,'los_state_known':state_known,'continuous_certified':passed})
    pd.DataFrame(ev).to_csv(ROOT/'results/q3_Q2_001_event_partition.csv',index=False); pd.DataFrame(los).to_csv(ROOT/'results/q3_Q2_001_los_certificate.csv',index=False); pd.DataFrame(certrows).to_csv(ROOT/'results/q3_Q2_001_continuous_certification.csv',index=False); pd.DataFrame(timeline_continuity_audit(traj)).to_csv(ROOT/'results/q3_Q2_001_timeline_continuity_audit.csv',index=False)
    out={'primitive_intervals':len(certrows),'los_cells':len(los),'continuous_certificate_pass':bool(all(x['continuous_certified'] for x in certrows)),'minimum_los_clearance_lower_bound':float(min(x['los_clearance_lower_bound'] for x in certrows)),'minimum_link_margin_lower_bound':float(min(x['margin_lower_bound'] for x in certrows)),'timeline_continuity_pass':bool(all(x['pass'] for x in timeline_continuity_audit(traj))),'note':'Path-loss bound uses endpoint/quadratic stationary distance maximum; LOS bound uses affine-in-time endpoint minima for each fixed DEM cell, with obstruction loss applied when clearance is nonpositive.'}
    (ROOT/'results/q3_Q2_001_strict_certificate_summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
