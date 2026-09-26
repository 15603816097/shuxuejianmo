"""Q4 prerequisite gate.

Q4 is only allowed to optimize partitions after Q3 has a frozen globally
certified joint transport/relay schedule. Until then this script reports
BLOCKED_BY_Q3 rather than reusing historical Q3/Q4 numbers.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--q3-dir",default="results/q3_official")
    ap.add_argument("--out",default="results/q4_official")
    a=ap.parse_args()
    q3=Path(a.q3_dir); out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
    p=q3/"q3_global_summary.json"
    if not p.exists():
        z={"status":"BLOCKED_BY_Q3","reason":"q3_global_summary.json not produced yet",
           "q4_method":"freeze Q3 -> shared-relay coupling components -> canonical K=2/K=3 full partition enumeration -> resource/Pareto comparison"}
    else:
        s=json.loads(p.read_text(encoding="utf-8"))
        ready=bool(s.get("q3_global_certified",False))
        z={"status":"READY_FOR_Q4" if ready else "BLOCKED_BY_Q3",
           "q3_status":s.get("status"),"q3_global_certified":ready,
           "q4_method":"freeze Q3 -> shared-relay coupling components -> canonical K=2/K=3 full partition enumeration -> resource/Pareto comparison"}
    (out/"q4_gate_summary.json").write_text(json.dumps(z,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(z,ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
