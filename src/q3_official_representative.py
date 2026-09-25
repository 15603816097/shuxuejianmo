"""Representative, non-optimizing Q3 official-semantics closed-loop audit."""
from __future__ import annotations
import ast, json, math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from minimal_pipeline import load_inputs, q1, q2, node_map, line_geometry, energy_leg
from final_joint import strict_q3
from q3_official_semantics import dynamic_communication_audit, relay_component_ledger, relay_time_chain, hard_deadline_ok

ROOT = Path(__file__).resolve().parents[1]

def main():
    data = load_inputs(); _, batches = q1(data); tasks, _ = q2(data, batches); cert = strict_q3(data, tasks)
    task = tasks.iloc[0]; c = cert.iloc[0].to_dict(); service = str(task['service']); nm = node_map(data); o = data['center']; s = nm[service]; drone = data['drones'][str(task['type'])]
    dist, peak, *_ = line_geometry(o, s, data['dem']); cruise_h = peak + 50.0; oh = float(o['海拔（m）']); sh = float(s['海拔（m）']) + 30.0
    hup = max(0.0, cruise_h-oh); hdown = max(0.0, cruise_h-sh); tcl = hup/drone.climb_speed; tcr = dist/drone.speed; tde = hdown/drone.descend_speed; hand = float(task['handoff_s'])
    segments = []; t = 0.0
    def add(dt, a, b, ha, hb, phase):
        nonlocal t
        segments.append({'t0':t,'t1':t+dt,'a':a,'b':b,'ha':ha,'hb':hb,'phase':phase}); t += dt
    add(tcl, o, o, oh, cruise_h, 'climb')
    add(tcr, o, s, cruise_h, cruise_h, 'cruise')
    add(tde, s, s, cruise_h, sh, 'descent')
    add(hand, s, s, sh, sh, 'delivery')
    tcl_return = max(0.0, cruise_h-sh) / drone.climb_speed
    tde_return = max(0.0, cruise_h-oh) / drone.descend_speed
    add(tcl_return, s, s, sh, cruise_h, 'return_climb')
    add(tcr, s, o, cruise_h, cruise_h, 'return_cruise')
    add(tde_return, o, o, cruise_h, oh, 'return_descent')
    # For a continuous-communication test the relay must be ready before the
    # transport UAV launches.  Start its preparation early enough that service
    # begins at transport t=0 and lasts through the complete trajectory.
    transport_duration = float(t)
    chain0 = relay_time_chain(data, service, c, max(hand, transport_duration), 0.0)
    chain = relay_time_chain(data, service, c, max(hand, transport_duration), -chain0['service_start'])
    comm = dynamic_communication_audit(data, service, str(task['type']), c, segments, relay_chain=chain)
    comp = relay_component_ledger(data, 'R-E01', chain['prepare_start'], chain['return_end'], chain['total_energy_kwh'])
    delivery = float(task['delivery_s'])
    box0 = next(b for b in data['boxes'] if b['货箱编号'] in (task['box_ids'] if isinstance(task['box_ids'], list) else ast.literal_eval(str(task['box_ids']))))
    deadline = min([x for x in [float(box0['期望送达时间（s）']) if box0['物资类型']=='医疗物资' else None, float(box0['首批截止时间（s）']) if box0['是否首批保障']=='是' else None] if x is not None], default=float('inf'))
    out = {'task_id': str(task['sortie']), 'service': service, 'certification_input': {'relay_feasible': bool(c['relay_feasible']), 'relay_fraction': float(c['relay_fraction']), 'relay_height_offset_m': float(c['relay_height_offset_m'])}, 'transport_formula': {'climb_s': tcl, 'cruise_s': tcr, 'descent_s': tde, 'handoff_s': hand, 'delivery_time_s': delivery}, 'relay_time_chain': chain, 'relay_component': comp, 'dynamic_communication': comm, 'all_communication_ok': bool(all(x['communication_ok'] for x in comm)), 'hard_deadline_manual': {'delivery_time_s': delivery, 'deadline_s': deadline, 'ok': hard_deadline_ok(delivery, deadline)}, 'resource_events': [{'resource':'relay_airframe','start_s':chain['prepare_start'],'end_s':chain['busy_end']},{'resource':'relay_energy_component','start_s':comp['use_start_s'],'end_s':comp['use_end_s']},{'resource':'relay_energy_component_charge','start_s':comp['charge_start_s'],'end_s':comp['charge_end_s']}], 'status':'REPRESENTATIVE_SEMANTICS_ONLY_NO_OPTIMIZATION'}
    (ROOT/'results/q3_official_representative_audit.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    (ROOT/'logs/q3_official_representative_audit.md').write_text('# Q3 官方语义代表性闭环审计\n\n```json\n'+json.dumps(out,ensure_ascii=False,indent=2)+'\n```\n',encoding='utf-8')
    print(json.dumps({'task_id':out['task_id'],'all_communication_ok':out['all_communication_ok'],'reserve_ok':comp['reserve_ok'],'hard_deadline_ok':out['hard_deadline_manual']['ok'],'relay_total_energy_kwh':chain['total_energy_kwh'],'component_charge_s':comp['charge_end_s']-comp['charge_start_s'],'trajectory_intervals':len(comm)},ensure_ascii=False,indent=2))
if __name__=='__main__': main()
