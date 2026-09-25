# Q2-v2: multi-point route-pool + CP-SAT scheduling

The old Q2 K=19..27 experiment is not a true multi-point optimization: its own code reports `multi_point_sorties = 0` and constructs every route as `O01 -> service -> O01`. It therefore cannot be used to judge whether a ~7000 s makespan is reachable under the full Q2 model.

Q2-v2 uses a different method:
1. Generate physically feasible single-, two- and selected three-service route skeletons.
2. Generate multiple box bundles per skeleton (deadline-first, weight/volume variants, plus guaranteed single-box/Q1 fallback columns).
3. Recompute every candidate with the official DEM leg rule, payload evolution, preparation/loading, service handoff, energy and reserve.
4. Use CP-SAT to select an exact cover of all 80 boxes while jointly scheduling optional route intervals under transport-aircraft and same-type shared-battery cumulative capacities.
5. Enforce medical expected times and first-batch deadlines as hard start-time limits. Non-medical expected times are soft lateness.
6. Test `Cmax <= 7000 s` directly; separately run a makespan-minimization search.
7. Reconstruct concrete aircraft and battery IDs by interval coloring after CP-SAT; cumulative feasibility guarantees such a coloring exists for interval resources.

The first goal is not to assume the classmate's 7000 s is correct; it is to make 7000 s a reproducible feasibility test under the same official constraints.
