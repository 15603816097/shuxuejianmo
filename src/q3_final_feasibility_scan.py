"""Bounded K=19..26 feasibility scan using the frozen Q3 evaluator.

Each K starts from an existing Q2 batching warm start, reorders/reassigns the
tasks with an event decoder, and then calls q3_final_pipeline.evaluate for
every task.  This is a best-known heuristic scan; it is not an optimality
proof.
"""
from __future__ import annotations
import ast, json, hashlib, sys, time
from pathlib import Path
import pandas as pd
sys.path.insert(0, str(Path(__file__).resolve().parent))
import q3_final_pipeline as pipe
import q3_five_service_hover_refinement as comm

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'; SHA='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'; DELTA=.1
BASE_THRESHOLDS=comm.thresholds

def overlap_peak(df, start, end, rid):
    if df.empty: return 0, False
    peaks={}; bad=False
    for key,g in df.groupby(rid):
        active=[]; peak=0
        for a,b in sorted(zip(g[start],g[end])):
            bad |= any(a < z-1e-6 for z in active)
            active=[z for z in active if z>a+1e-6]; active.append(b); peak=max(peak,len(active))
        peaks[key]=peak
    return peaks,bad

def load_warm(K):
    p=RES/'q2_sensitivity_19_27'/f'q2_sens_K{K}_schedule.csv'
    if not p.exists(): return None
    d=pd.read_csv(p); d['box_ids']=d.box_ids.map(ast.literal_eval); return d

def charge_time(data, typ, energy):
    d=data['drones'][typ]; soc=1.0-float(energy)/float(d.energy); full={'A':1800.,'B':2400.,'C':3000.}[typ]
    return full*(0.65*(0.9-soc)/0.9+0.35) if soc<0.9 else full*0.35*(1-soc)/0.1

def deadline_rows(data, tl, offset):
    out=[]
    for x in tl['events']:
        if x.get('event')=='handoff': out.append((x['box_id'],float(x['end_s'])+offset))
    bm={x['货箱编号']:x for x in data['boxes']}; return out,bm

def run(K, limit=None):
    t0=time.time(); data=pipe.load_inputs(); warm=load_warm(K)
    if warm is None: return {'K':K,'complete_feasible':False,'search_status':'FEASIBLE_NOT_FOUND_WITHIN_SEARCH','main_failure':'NO_WARM_CANDIDATE','runtime_seconds':time.time()-t0}
    original=BASE_THRESHOLDS
    def wrapped(D):
        td,ta,tb=original(D); return td,ta,tb+DELTA
    comm.thresholds=wrapped; pipe.thresholds=wrapped
    # EDF warm-start reordering; all starts and resource assignments are rebuilt.
    bm={x['货箱编号']:x for x in data['boxes']}
    def dl(row):
        vals=[]
        for bid in row.box_ids:
            b=bm[bid]
            if b['物资类型']=='医疗物资': vals.append(float(b['期望送达时间（s）']))
            if b['是否首批保障']=='是': vals.append(float(b['首批截止时间（s）']))
        return min(vals) if vals else float('inf')
    warm=warm.assign(_deadline=warm.apply(dl,axis=1)).sort_values(['_deadline','service','sortie']).reset_index(drop=True)
    if limit is not None: warm=warm.head(int(limit)).copy()
    air={u:0. for typ,us in data['aircraft'].items() for u in us}; bat={typ:{f'{typ}-B{i+1:02d}':0. for i in range(int(n))} for typ,n in data['batteries'].items()}
    tasks=[]; task_rows=[]; boxes=[]; tr=[]; bat_events=[]; rel=[]; comp=[]
    for j,r in warm.iterrows():
        typ=str(r.type); ids=list(r.box_ids); au=min(data['aircraft'][typ],key=lambda u:air[u]); battery=min(bat[typ],key=lambda b:bat[typ][b]); start=max(air[au],bat[typ][battery]);
        out=pipe.evaluate(data,(str(r.service),),ids,typ,drone_id=au,start_time=start,battery_id=battery)
        tasks.append(out); inv=f'K{K}-{j+1:03d}'
        tl=out['timeline']; off=float(start)
        for seg in tl['segments']:
            for ph in seg['phases']: tr.append({'resource_id':au,'task_id':inv,'busy_start':ph['start_s']+off,'busy_end':ph['end_s']+off,'phase':ph['phase']})
        dlrows,_=deadline_rows(data,tl,off); boxes.extend({'task_id':inv,'box_id':b,'delivery_time_s':d} for b,d in dlrows)
        geom=bool(out['status']['COMMUNICATION_GEOMETRY_PASS']); hard=all(x['pass'] for x in out['deadline_ledger'] if x['hard']); reserve=bool(out['transport_energy_kwh'] <= out['required_reserve_kwh']+1e-6)
        if geom:
            for rr in out.get('relay_timeline',[]): rel.append({'resource_id':rr.get('relay_id','R01'),'task_id':inv,'busy_start':rr['prepare_start']+off,'busy_end':rr['return_end']+off})
            for cc in out.get('component_ledger',[]): comp.append({'resource_id':cc.get('component_id',''),'task_id':inv,'busy_start':cc.get('use_start_s',0)+off,'busy_end':cc.get('use_end_s',0)+off,'charge_start':cc.get('charge_start_s',0)+off,'charge_end':cc.get('available_again_s',0)+off})
        delivery={b:d for b,d in dlrows}; hardfail=not hard
        reason='' if out['status']['ROUTE_FEASIBLE'] else ('HARD_DEADLINE' if hardfail else 'COMMUNICATION' if not geom else 'RELAY_RESOURCE' if not out['status']['RELAY_CHAIN_PASS'] else 'COMPONENT_RESOURCE')
        task_rows.append({'K':K,'task_id':inv,'service':r.service,'type':typ,'start_time':start,'box_count':len(ids),'transport_feasible':out['status']['TRANSPORT_FEASIBLE'],'reserve_ok':reserve,'hard_deadline_ok':hard,'communication_ok':geom,'relay_chain_ok':out['status']['RELAY_CHAIN_PASS'],'component_ok':out['status']['COMPONENT_PASS'],'route_feasible':out['status']['ROUTE_FEASIBLE'],'transport_energy':out['transport_energy_kwh'],'relay_energy':out['relay_energy_kwh'],'failure_stage':reason})
        # Recompute next availability from this evaluated task, preserving the
        # warm-start assignment policy but using actual returned duration.
        end=off+float(tl['makespan_s']); air[au]=end
        chg=charge_time(data,typ,out['transport_energy_kwh']); bat_events.append({'resource_id':battery,'battery_type':typ,'task_id':inv,'busy_start':off,'busy_end':end+chg,'use_end':end,'charge_start':end,'charge_end':end+chg,'energy_kwh':out['transport_energy_kwh']}); bat[typ][battery]=end+chg
    td=pd.DataFrame(task_rows); bd=pd.DataFrame(boxes); trd=pd.DataFrame(tr); rd=pd.DataFrame(rel); cd=pd.DataFrame(comp)
    badf=pd.DataFrame(bat_events); tp,tbad=overlap_peak(trd,'busy_start','busy_end','resource_id'); bp,bbad=overlap_peak(badf,'busy_start','busy_end','resource_id'); rp,rbad=overlap_peak(rd,'busy_start','busy_end','resource_id'); cp,cbad=overlap_peak(cd,'busy_start','busy_end','resource_id')
    bpeak=max([v for rid,v in bp.items() if str(rid).startswith('B-')] or [0]); cpeak=max([v for rid,v in bp.items() if str(rid).startswith('C-')] or [0])
    all_ids=[]
    for r in warm.itertuples(): all_ids.extend(r.box_ids)
    hard=0; soft=[]
    for bid,g in bd.groupby('box_id'):
        b=bm[bid]; d=float(g.delivery_time_s.iloc[0]); ishard=(b['物资类型']=='医疗物资' or b['是否首批保障']=='是'); deadline=float(b['期望送达时间（s）'] if b['物资类型']=='医疗物资' else b['首批截止时间（s）']) if ishard else float(b['期望送达时间（s）']);
        if ishard and d>deadline+1e-6: hard+=1
        if not ishard: soft.append(max(0.,d-deadline))
    complete=bool(limit is None and len(all_ids)==80 and len(set(all_ids))==80 and len(td)==K and td.route_feasible.all() and hard==0 and not tbad and not bbad and not rbad and not cbad)
    starts=[float(x['start_time'])+float(task['timeline']['makespan_s']) for x,task in zip(task_rows,tasks)]
    rec={'K':K,'complete_feasible':complete,'assigned_boxes':len(set(all_ids)),'duplicate_boxes':len(all_ids)-len(set(all_ids)),'missing_boxes':80-len(set(all_ids)),'hard_violations':hard,'route_feasible_count':int(td.route_feasible.sum()),'communication_failures':int((~td.communication_ok).sum()),'transport_energy_failures':int((~td.transport_feasible).sum()),'deadline_failures':int((~td.hard_deadline_ok).sum()),'transport_resource_failures':int(tbad),'battery_failures':int(bbad),'relay_resource_failures':int(rbad),'component_failures':int(cbad),'soft_late_boxes':sum(x>1e-6 for x in soft),'total_soft_lateness':sum(soft),'max_soft_lateness':max(soft) if soft else 0.,'joint_makespan':max(starts or [0.]),'transport_energy':float(td.transport_energy.sum()),'relay_energy':float(td.relay_energy.dropna().sum()),'total_energy':float(td.transport_energy.sum()+td.relay_energy.dropna().sum()),'transport_sorties':len(warm),'relay_sorties':int(sum(bool(x['relay_required_blocks']) for x in tasks)),'transport_peak':max(tp.values()) if tp else 0,'B_battery_peak':bpeak,'C_battery_peak':cpeak,'battery_peak':bp,'relay_peak':max(rp.values()) if rp else 0,'component_peak':max(cp.values()) if cp else 0,'candidate_count':len(warm),'search_iterations':1,'runtime_seconds':time.time()-t0,'search_status':'COMPLETE_FEASIBLE' if complete else 'FEASIBLE_NOT_FOUND_WITHIN_SEARCH','main_failure':('COMPLETE_FEASIBLE' if complete else (td.failure_stage.dropna().iloc[0] if td.failure_stage.notna().any() else 'RESOURCE_LEDGER'))}
    tag=f'q3_K{K}_final_scan'; td.to_csv(RES/f'{tag}_task_audit.csv',index=False); bd.to_csv(RES/f'{tag}_boxes.csv',index=False); trd.to_csv(RES/f'{tag}_transport_ledger.csv',index=False); badf.to_csv(RES/f'{tag}_battery_ledger.csv',index=False); rd.to_csv(RES/f'{tag}_relay_ledger.csv',index=False); cd.to_csv(RES/f'{tag}_component_ledger.csv',index=False); pd.DataFrame([rec]).to_csv(RES/f'{tag}_summary.csv',index=False)
    return rec

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument('--k',type=int,required=True); ap.add_argument('--limit',type=int); args=ap.parse_args(); print(json.dumps(run(args.k,args.limit),ensure_ascii=False,indent=2))
