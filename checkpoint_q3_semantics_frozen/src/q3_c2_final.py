"""Frozen C2 entry point; only the number of visit nodes differs from C1."""
from q3_semantics_core import build_authoritative_timeline, deadline_ledger, transport_energy_ledger, direct_and_relay_blocks, official_relay_energy, component_ledger, status

def evaluate(data, visit_order, aircraft_type, box_ids):
    if len(visit_order)!=2: raise ValueError('C2 requires exactly two service areas')
    tl=build_authoritative_timeline(data,visit_order,aircraft_type,box_ids)
    return tl, deadline_ledger(data,tl), transport_energy_ledger(data,tl), direct_and_relay_blocks(data,tl)
