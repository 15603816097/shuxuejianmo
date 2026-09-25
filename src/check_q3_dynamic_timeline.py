"""Read-only integrity check for an already generated Q2-001 timeline."""
from __future__ import annotations
import json
from pathlib import Path
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]

def main():
    summary=json.loads((ROOT/'logs/q3_Q2_001_dynamic_timeline.md').read_text())
    timeline=pd.read_csv(ROOT/'results/q3_Q2_001_dynamic_timeline.csv')
    req_path=ROOT/'results/q3_Q2_001_relay_required_intervals.csv'
    try:
        req=pd.read_csv(req_path)
    except pd.errors.EmptyDataError:
        # Zero relay-required intervals is a valid result; pandas cannot
        # infer columns from an intentionally empty CSV.
        req=pd.DataFrame()
    params=pd.read_csv(ROOT/'results/q3_parameter_source_audit.csv')
    chain=summary.get('relay_chain')
    out={'timeline_exists':True,'timeline_rows':int(len(timeline)),'relay_required_file_rows':int(len(req)),
         'parameter_rows':int(len(params)),'hardcoded_true':int(params['hardcoded'].astype(str).str.lower().eq('true').sum()),
         'communication_pass':bool(summary['communication_pass']),'relay_required_intervals':int(len(summary.get('relay_required_intervals',[]))),
         'direct_intervals':int(len(summary.get('direct_available_intervals',[]))),
         'relay_chain_present':chain is not None,
         'result_complete':bool(len(timeline)==int(summary['dynamic_intervals']) and len(req)==len(summary.get('relay_required_intervals',[])) and not params['hardcoded'].astype(str).str.lower().eq('true').any())}
    print(json.dumps(out,ensure_ascii=False,indent=2))
    if not out['result_complete']: raise SystemExit(1)
if __name__=='__main__':main()
