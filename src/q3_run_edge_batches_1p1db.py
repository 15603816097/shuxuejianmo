import json,subprocess,sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'
nodes=['O01']+[f'S{i:03d}' for i in range(1,16)]
args=[(u,v,'C') for u in nodes for v in nodes if u!=v]
chunks=[args[i:i+8] for i in range(0,len(args),10)]
def run(c):
 p=subprocess.run([sys.executable,str(ROOT/'src/q3_edge_batch_worker_1p1db.py'),json.dumps(c)],cwd=ROOT,text=True,capture_output=True,timeout=500)
 if p.returncode!=0: raise RuntimeError(p.stderr[-1000:])
 return json.loads(p.stdout)
rows=[]
with ThreadPoolExecutor(max_workers=12) as ex:
 fs=[ex.submit(run,c) for c in chunks]
 for i,f in enumerate(as_completed(fs),1):
  rows.extend(f.result()); print(f'completed batch {i}/{len(fs)}',flush=True)
pd.DataFrame(rows).sort_values(['from_node','to_node']).to_csv(RES/'q3_full_edge_graph_1p1db.csv',index=False,encoding='utf-8-sig')
print(json.dumps({'records':len(rows)}))
