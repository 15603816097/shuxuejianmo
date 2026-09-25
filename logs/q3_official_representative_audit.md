# Q3 官方语义代表性闭环审计

```json
{
  "task_id": "Q2-001",
  "service": "S001",
  "certification_input": {
    "relay_feasible": true,
    "relay_fraction": 0.5,
    "relay_height_offset_m": 200.0
  },
  "transport_formula": {
    "climb_s": 58.142644042968755,
    "cruise_s": 204.23067064990087,
    "descent_s": 44.52830505371094,
    "handoff_s": 432.0,
    "delivery_time_s": 1248.9016197465805
  },
  "relay_time_chain": {
    "prepare_start": -368.20985513087277,
    "prepare_end": -188.20985513087277,
    "outbound_start": -188.20985513087277,
    "outbound_end": -30.0,
    "link_setup_start": -30.0,
    "link_setup_end": 0.0,
    "service_start": 0.0,
    "service_end": 1051.4332394931612,
    "return_start": 1051.4332394931612,
    "return_end": 1228.3396436642695,
    "turnaround_start": 1228.3396436642695,
    "busy_end": 1528.3396436642695,
    "outbound_flight_s": 158.20985513087277,
    "return_flight_s": 176.90640417110828,
    "service_duration_s": 1051.4332394931612,
    "outbound_energy_kwh": 0.05257658107364616,
    "return_energy_kwh": 0.03261965771140239,
    "service_energy_kwh": 0.3212712676229104,
    "total_energy_kwh": 0.406467506407959,
    "relay_ground_m": 152.0780792236328,
    "relay_height_m": 352.0780792236328,
    "relay_position_lon": 109.2370418,
    "relay_position_lat": 23.0210511
  },
  "relay_component": {
    "component_id": "R-E01",
    "energy_kwh": 0.406467506407959,
    "soc_start": 1.0,
    "soc_end": 0.8729789042475128,
    "reserve_fraction": 0.2,
    "reserve_ok": true,
    "available_before_task_s": 0.0,
    "resource_available_ok": false,
    "use_start_s": -368.20985513087277,
    "use_end_s": 1228.3396436642695,
    "charge_start_s": 1228.3396436642695,
    "charge_end_s": 1893.4670681425027,
    "available_again_s": 1893.4670681425027,
    "full_charge_s": 1800.0,
    "inventory": 6
  },
  "dynamic_communication": [
    {
      "time_interval_start": 0.0,
      "time_interval_end": 58.142644042968755,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "",
      "access_link_ok": false,
      "backhaul_link_ok": true,
      "communication_ok": false,
      "relay_service_window_ok": true,
      "midpoint_time_s": 29.071322021484377,
      "sample_count": 3,
      "phase": "climb"
    },
    {
      "time_interval_start": 58.142644042968755,
      "time_interval_end": 262.3733146928696,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "",
      "access_link_ok": true,
      "backhaul_link_ok": true,
      "communication_ok": true,
      "relay_service_window_ok": true,
      "midpoint_time_s": 160.25797936791918,
      "sample_count": 3,
      "phase": "cruise"
    },
    {
      "time_interval_start": 262.3733146928696,
      "time_interval_end": 306.90161974658054,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "R01",
      "access_link_ok": true,
      "backhaul_link_ok": true,
      "communication_ok": true,
      "relay_service_window_ok": true,
      "midpoint_time_s": 284.63746721972507,
      "sample_count": 3,
      "phase": "descent"
    },
    {
      "time_interval_start": 306.90161974658054,
      "time_interval_end": 738.9016197465805,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "R01",
      "access_link_ok": true,
      "backhaul_link_ok": true,
      "communication_ok": true,
      "relay_service_window_ok": true,
      "midpoint_time_s": 522.9016197465805,
      "sample_count": 3,
      "phase": "delivery"
    },
    {
      "time_interval_start": 738.9016197465805,
      "time_interval_end": 774.5242637895493,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "R01",
      "access_link_ok": true,
      "backhaul_link_ok": true,
      "communication_ok": true,
      "relay_service_window_ok": true,
      "midpoint_time_s": 756.7129417680649,
      "sample_count": 3,
      "phase": "return_climb"
    },
    {
      "time_interval_start": 774.5242637895493,
      "time_interval_end": 978.7549344394502,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "",
      "access_link_ok": true,
      "backhaul_link_ok": true,
      "communication_ok": true,
      "relay_service_window_ok": true,
      "midpoint_time_s": 876.6395991144998,
      "sample_count": 3,
      "phase": "return_cruise"
    },
    {
      "time_interval_start": 978.7549344394502,
      "time_interval_end": 1051.4332394931612,
      "direct_available": false,
      "relay_required": true,
      "relay_id": "",
      "access_link_ok": false,
      "backhaul_link_ok": true,
      "communication_ok": false,
      "relay_service_window_ok": true,
      "midpoint_time_s": 1015.0940869663057,
      "sample_count": 3,
      "phase": "return_descent"
    }
  ],
  "all_communication_ok": false,
  "hard_deadline_manual": {
    "delivery_time_s": 1248.9016197465805,
    "deadline_s": 3600.0,
    "ok": true
  },
  "resource_events": [
    {
      "resource": "relay_airframe",
      "start_s": -368.20985513087277,
      "end_s": 1528.3396436642695
    },
    {
      "resource": "relay_energy_component",
      "start_s": -368.20985513087277,
      "end_s": 1228.3396436642695
    },
    {
      "resource": "relay_energy_component_charge",
      "start_s": 1228.3396436642695,
      "end_s": 1893.4670681425027
    }
  ],
  "status": "REPRESENTATIVE_SEMANTICS_ONLY_NO_OPTIMIZATION"
}
```
