# Time semantics audit

All times use seconds from the dispatch-day origin t=0; no calendar-day rollover is present. `start_s`, `delivery_s`, `return_s`, and deadlines are all seconds. Arrival is `delivery_s - handoff_s`; hard deadline is the medical/first-batch contractual deadline; lateness is `max(0, completion_time - hard_deadline)` with a 1e-6 s audit tolerance.

- earliest task: Q2-013: start=180.000000s, arrival=1024.834822s, completion=1264.834822s
- latest delivery task: Q2-002: start=97042.163847s, arrival=97889.065467s, completion=98357.065467s
- maximum-lateness box: S012-MED-01, completion=93670.226089s, deadline=3600.000000s, lateness=90070.226089s
- on-time sample: S001-MED-01, deadline=3600.0, lateness=0.000000s

Makespan is the last transport return `98669.597086 s`; last delivery is `98357.065467 s`. Recomputed total lateness=558271.122744s and maximum lateness=90070.226089s. Units and origin audit: **PASS after makespan correction**.
