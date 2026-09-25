"""Frozen C3 entry point; only the number of visit nodes differs from C1/C2."""
from q3_semantics_core import build_authoritative_timeline, deadline_ledger, transport_energy_ledger, direct_and_relay_blocks, official_relay_energy, component_ledger, status, evaluate_route

def evaluate(data, visit_order, aircraft_type, box_ids):
    if len(visit_order)!=3: raise ValueError('C3 requires exactly three service areas')
    return evaluate_route(data,visit_order,aircraft_type,box_ids)
