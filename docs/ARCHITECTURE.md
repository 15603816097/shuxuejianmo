# Project architecture (ChatGPT takeover)

## Status
- Q1: frozen; existing result is retained and not re-optimized.
- Q2: legacy single-point/fixed-K heuristics are reference baselines only. The official Q2 requirement permits one sortie to visit one or more service areas, so Q2 is rebuilt around true multi-point route candidates plus joint resource scheduling.
- Q3: keep only `checkpoint_q3_final_pipeline_v2` as the frozen communication/transport evaluator checkpoint. Strict baseline and sensitivity scenarios must remain separate.
- Q4: derived only after a certified Q3 schedule is frozen.

## Single source of transport physics
`src/minimal_pipeline.py` remains the data/DEM/leg-energy source until a later refactor. Q2-v2 must use the same node-to-node straight-line rule, DEM +50 m cruise altitude, 30 m service operating altitude, payload-dependent range, reserve constraint and official battery recharge law.

## Final execution path
```
data -> shared transport physics
     -> Q1 (frozen)
     -> Q2-v2 route-pool + CP-SAT resource scheduler
     -> Q3 route/communication/resource joint solver
     -> Q4 frozen-task partition/resource sizing
```

## Repository policy
Historical CSV outputs may be kept as evidence, but superseded checkpoints and vendored Python environments are removed. New official outputs go under `results/final/`; experimental outputs are named by method/version and never silently overwrite a frozen result.
