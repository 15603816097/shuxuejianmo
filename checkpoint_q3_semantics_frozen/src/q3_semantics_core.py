"""Frozen Q3 semantic core shared by C1/C2/C3.

All route evaluators must obtain their timeline, deadline ledger, payload-aware
energy, communication blocks, relay accounting and resource events here.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterable
import numpy as np
from minimal_pipeline import node_map, line_geometry, energy_leg, terrain_profile, xy
from q3_official_semantics import relay_leg_time_energy, relay_component_ledger

TOL=1e-6

def build_authoritative_timeline(data, visit_order, aircraft_type, box_ids):
    nm=node_map(data); d=data['drones'][aircraft_type]; bm={x['货箱编号']:x for x in data['boxes']}; boxes=[bm[x] for x in box_ids]
    payload=sum(float(x['单箱质量（kg）']) for x in boxes); t=float(d.prep+d.load_box*len(boxes)); events=[{'event':'loading','phase':'loading','start_s':0.,'end_s':t,'payload_before':payload,'payload_after':payload,'box_id':''}]; deliveries={}; prev='O01'; segments=[]
    for nxt in tuple(visit_order)+('O01',):
        a,b=nm[prev],nm[nxt]; h0=float(a['海拔（m）'])+(30 if prev!='O01' else 0); h1=float(b['海拔（m）'])+(30 if nxt!='O01' else 0); dist,peak,*_=line_geometry(a,b,data['dem']); energy,dur=energy_leg(d,dist,peak,h0,h1,payload); cruise=peak+50.; up=max(0.,cruise-h0)/d.climb_speed; down=max(0.,cruise-h1)/d.descend_speed
        phases=[]
        if up>TOL: phases.append(('climb',up,a,a,h0,cruise))
        phases.append(('cruise',dist/d.speed,a,b,cruise,cruise))
        if down>TOL: phases.append(('descent',down,b,b,cruise,h1))
        leg=[]
        for ph,dt,p0,p1,ha,hb in phases:
            z={'event':'flight','phase':ph,'start_s':t,'end_s':t+dt,'from_node':prev,'to_node':nxt,'payload_before':payload,'payload_after':payload,'box_id':'','a':p0,'b':p1,'ha':ha,'hb':hb,'energy_kwh':energy*(dt/dur if dur else 0.)}; events.append(z); leg.append(z); t+=dt
        if nxt!='O01':
            for bx in [x for x in boxes if x['服务区编号']==nxt]:
                dt=float(d.handoff_box); events.append({'event':'handoff','phase':'handoff','start_s':t,'end_s':t+dt,'from_node':nxt,'to_node':nxt,'payload_before':payload,'payload_after':payload-float(bx['单箱质量（kg）']),'box_id':bx['货箱编号'],'delivery_time_s':t+dt,'a':b,'b':b,'ha':h1,'hb':h1,'energy_kwh':0.}); t+=dt; deliveries[bx['货箱编号']]=t; payload-=float(bx['单箱质量（kg）'])
        segments.append({'from':prev,'to':nxt,'phases':leg,'energy_kwh':energy,'payload_before':sum(float(x['单箱质量（kg）']) for x in boxes if x['货箱编号'] in box_ids)})
        prev=nxt
    events.append({'event':'return_end','phase':'return_end','start_s':t,'end_s':t,'payload_before':payload,'payload_after':payload,'box_id':''})
    return {'events':events,'deliveries':deliveries,'segments':segments,'makespan_s':t,'box_ids':list(box_ids),'visit_order':tuple(visit_order),'aircraft_type':aircraft_type}

def deadline_ledger(data,timeline):
    bm={x['货箱编号']:x for x in data['boxes']}; out=[]
    for bid,delivery in timeline['deliveries'].items():
        b=bm[bid]; rules=[]
        if b['物资类型']=='医疗物资': rules.append(('MEDICAL_HARD',float(b['期望送达时间（s）'])))
        if b['是否首批保障']=='是': rules.append(('FIRST_BATCH_HARD',float(b['首批截止时间（s）'])))
        if not rules: rules.append(('EXPECTED_SOFT',float(b['期望送达时间（s）'])))
        for source,deadline in rules: out.append({'box_id':bid,'delivery_time_s':delivery,'deadline_s':deadline,'source':source,'hard':source!='EXPECTED_SOFT','pass':delivery<=deadline+TOL if source!='EXPECTED_SOFT' else True})
    return out

def transport_energy_ledger(data,timeline):
    rows=[]; cumulative=0.
    for s in timeline['segments']:
        for p in s['phases']:
            cumulative+=float(p['energy_kwh']); rows.append({'from':s['from'],'to':s['to'],'phase':p['phase'],'payload_before':p['payload_before'],'duration_s':p['end_s']-p['start_s'],'energy_kwh':p['energy_kwh'],'cumulative_energy_kwh':cumulative})
    return rows

def direct_and_relay_blocks(data,timeline):
    c=data['center']; freq=float(data['comm'][('传播参数','载波频率（MHz）')]); obs=float(data['comm'][('传播参数','地形遮挡附加损耗（dB）')]);
    # thresholds are loaded from workbook by the audited communication module
    from q3_five_service_hover_refinement import thresholds
    td,ta,tb=thresholds(data); G=xyz(data,c,float(c['海拔（m）'])+float(data['comm'][('固定网关 G01','天线离地高度（m）')]))
    req=[]
    for p in timeline['events']:
        if p.get('event')!='flight': continue
        dmax=max(np.linalg.norm(xyz(data,p['a'],p['ha'])-G),np.linalg.norm(xyz(data,p['b'],p['hb'])-G)); margin=td-(32.45+20*math.log10(freq)+20*math.log10(max(dmax/1000,1e-12))+obs)
        if margin<0: req.append(dict(p,direct_margin_db=margin))
    blocks=[]
    for p in req:
        if blocks and p['phase']==blocks[-1]['phase'] and abs(p['start_s']-blocks[-1]['end_s'])<TOL: blocks[-1]['items'].append(p); blocks[-1]['end_s']=p['end_s']
        else: blocks.append({'start_s':p['start_s'],'end_s':p['end_s'],'phase':p['phase'],'items':[p]})
    return blocks

def continuous_interval_certificate(data, segment, endpoint, endpoint_h):
    """Conservative primitive certificate: quadratic distance upper bound and
    DEM-cell LOS lower bound over a closed interval, never midpoint-only."""
    c=data['center']; freq=float(data['comm'][('传播参数','载波频率（MHz）')]); obs=float(data['comm'][('传播参数','地形遮挡附加损耗（dB）')]);
    from q3_five_service_hover_refinement import thresholds
    threshold=thresholds(data)[0]; gateway_h=float(c['海拔（m）'])+float(data['comm'][('固定网关 G01','天线离地高度（m）')]); a,b=segment['a'],segment['b']; p0=xyz(data,a,segment['ha'])-xyz(data,endpoint,endpoint_h); v=xyz(data,b,segment['hb'])-xyz(data,a,segment['ha']); aa=float(v@v); bb=float(2*p0@v); cand=[0.,1.]
    if aa>1e-15:
        u=-bb/(2*aa)
        if 0<u<1: cand.append(u)
    dmax=max(float(np.linalg.norm(p0+u*v)) for u in cand); fspl=32.45+20*math.log10(freq)+20*math.log10(max(dmax/1000.,1e-12)); cells=terrain_profile(a,b,data['dem']); clear=[]
    for _,ground in cells:
        if np.isfinite(ground): clear.append(min(float(segment['ha']),float(segment['hb']))-float(ground))
    clearance=min(clear) if clear else float('nan'); path=fspl+obs; return {'distance_max':dmax,'distance_max_method':'endpoints+quadratic_stationary','fspl_upper_bound':fspl,'path_loss_upper_bound':path,'threshold':threshold,'margin_lower_bound':threshold-path,'los_clearance_lower_bound':clearance,'continuous_certified':bool(np.isfinite(clearance) and clearance>0 and threshold-path>=0),'dem_cells':len(cells)}

def xyz(data,p,h):
    q=xy(float(p['经度（°）']),float(p['纬度（°）']),float(data['center']['纬度（°）'])); return np.array([q[0],q[1],float(h)])

def official_relay_energy(data,relay_point,altitude,block):
    dist,peak,*_=line_geometry(data['center'],relay_point,data['dem']); tout,eout=relay_leg_time_energy(data,dist,peak,float(data['center']['海拔（m）']),altitude); tback,eback=relay_leg_time_energy(data,dist,peak,altitude,float(data['center']['海拔（m）'])); service=(float(data['relay']['hover_power_kw'])+float(data['relay']['comm_power_kw']))*(block['end_s']-block['start_s'])/3600.; return {'outbound_time_s':tout,'return_time_s':tback,'outbound_energy_kwh':eout,'return_energy_kwh':eback,'service_energy_kwh':service,'total_energy_kwh':eout+eback+service}

def component_ledger(data,component_id,start_s,end_s,energy,previous_available_s=0.): return relay_component_ledger(data,component_id,start_s,end_s,energy,previous_available_s)

def resource_events(timeline,components=()):
    events=[]
    for e in timeline['events']:
        if e.get('event')=='flight' or e.get('event')=='handoff': events.append({'resource':'transport','start_s':e['start_s'],'end_s':e['end_s']})
    events.extend(components); return events

def status(transport_ok,geometry_ok,relay_ok,component_ok):
    return {'TRANSPORT_FEASIBLE':bool(transport_ok),'COMMUNICATION_GEOMETRY_PASS':bool(geometry_ok),'RELAY_CHAIN_PASS':bool(relay_ok),'COMPONENT_PASS':bool(component_ok),'ROUTE_FEASIBLE':bool(transport_ok and geometry_ok and relay_ok and component_ok)}
