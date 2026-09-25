# Final metric audit

Independent recomputation from checkpoint `schedule.csv` and `deliveries.csv`. Summary files were not used.

- late boxes: **11**
- late sorties: **6** (Q2-004, Q2-006, Q2-009, Q2-010, Q2-015, Q2-018)
- total lateness: **558271.122744 s**
- maximum lateness: **90070.226089 s**
- makespan: **98669.597086 s**

The earlier 11-box/7-sortie count came from the pre-correction schedule/assignment and its resource aggregation. After correcting the disjunctive resource semantics and extracting solver-selected aircraft, battery and relay assignments, the frozen schedule independently gives 11 boxes across 6 sorties; the deadline predicate and tolerance are unchanged.
