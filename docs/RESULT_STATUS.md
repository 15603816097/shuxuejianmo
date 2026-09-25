# Result status and trust map

This file prevents historical experiments from being mistaken for final answers.

## Trusted / frozen
- Q1: current official Q1 outputs are retained; Q1 is frozen by user decision.
- Q3 physics checkpoint: `checkpoint_q3_final_pipeline_v2/`.
- `src/q3_final_pipeline.py` content SHA256 must remain:
  `5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb`.
- Raw data in `data/` and the original problem statement are authoritative inputs.

## Historical baselines only
- `results/q2_sensitivity_19_27/` and `q2_sortie_count_sensitivity_19_27.csv`: fixed-K, single-service-route heuristic baselines. They are not the final Q2 optimizer.
- Q2 K18/K19 audit files: useful for regression/comparison only.
- Most `results/q3_K*`, S008 diagnostics, 0.1/1.1 dB experiments: historical diagnostics; do not cite them as the final Q3 schedule unless explicitly re-certified.

## Invalid / superseded conclusions
- The former lazy-certification K19 result was invalid as an 80-box solution because its master reconstructed the box universe from the filtered candidate pool and silently dropped the eight S002 boxes.
- +1.1 dB was based on a delta-accounting mistake. The pointwise global restoration threshold from the strict baseline is about +1.144995 dB; +1.2 dB is the current sensitivity scenario to test, not a replacement for the strict baseline.
- Projected +1.1 dB edge labels are not independent evaluator certifications.

## Current work
- Q2-v2: `src/q2_v2_solver.py`, outputs to `results/q2_v2/`.
- Goal: independently test whether Cmax <= 7000 s is feasible using true multi-point route candidates and joint transport-aircraft/battery scheduling.
- No Q2-v2 result is final until the 80-box, hard-deadline, resource, energy and route audit passes.
