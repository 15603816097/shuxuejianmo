import sys,json,hashlib
from pathlib import Path
import q3_build_edge_graph as g
ROOT=Path(__file__).resolve().parents[1]
if hashlib.sha256((ROOT/'src/q3_final_pipeline.py').read_bytes()).hexdigest()!=g.SHA: raise RuntimeError('frozen SHA mismatch')
D=g.pipe.load_inputs(); g._DATA=D
args=json.loads(sys.argv[1]); out=[]
for a in args:
    u,v,t=a
    try:
        z=g.run_edge(D,u,v,t); z['evaluation_status']='COMPLETED'
    except Exception as e:
        z={'communication_possible':False,'worst_margin':float('nan'),'relay_required':None,'relay_required_duration':float('nan'),'candidate_count':0,'pipeline_called':False,'evaluation_status':'ERROR:'+type(e).__name__}
    out.append({'from_node':u,'to_node':v,'drone_type':t,**z,'scenario':'FEASIBILITY_RESTORED_0P1DB','delta_backhaul_db':g.DELTA,'pipeline_sha256':g.SHA})
print(json.dumps(out,allow_nan=True))
