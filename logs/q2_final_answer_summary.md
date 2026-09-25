# Q2 final answer summary

- Current schedule: **18 sorties**, all are single-point round trips.
- It visits **15 service areas**; S001, S002 and S003 are each split into two sorties.
- Independent audit: **11 late boxes and 6 late sorties**.
- Makespan (last return): **98669.597086 s**.
- Transport flight energy: **59.654458 kWh**; relay energy: **2.165069 kWh**; total energy: **61.82 kWh**.
- Resource capacity audit: passed for transport, relay, B/C batteries and per-service resources.
- Status: **BEST_KNOWN_FEASIBLE_NOT_PROVEN_OPTIMAL**.

## Splitting explanation

S001 has 154 kg and 0.394 m³ total demand, exceeding the maximum C-type payload and volume. S002 and S003 each have 81 kg, exceeding the C-type 80 kg payload while their 0.211 m³ volume does not exceed the C-type 0.25 m³ volume. Their split is therefore driven primarily by payload feasibility, not multi-point routing.

## Trade-off

Compared with the 60-second baseline, the current schedule reduces late boxes from 14 to 11 and late sorties from 7 to 6, but has larger total lateness, maximum lateness and makespan. Both schedules have 18 single-point sorties, so this is a timing/resource scheduling trade-off rather than a strict Pareto comparison.
