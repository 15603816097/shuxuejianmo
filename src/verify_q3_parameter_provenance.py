"""Independent provenance verifier for the runtime parameter trace."""
from __future__ import annotations
import ast, json, tempfile
from pathlib import Path
import pandas as pd
from openpyxl import load_workbook
ROOT=Path(__file__).resolve().parents[1]

def workbook_values():
    out={}; wb=load_workbook(ROOT/'data/无人机应急物资运输基础数据/通信链路参数.xlsx',data_only=True,read_only=True); ws=wb['数据']
    for row in ws.iter_rows(min_row=3,values_only=True):
        if row[0] is not None and row[1] is not None and row[4] is not None:
            out[f"{row[0]}:{row[1]}"]=float(row[4]); out[row[1]]=float(row[4])
    wb.close(); wb=load_workbook(ROOT/'data/无人机应急物资运输基础数据/中继无人机数据.xlsx',data_only=True,read_only=True); ws=wb['数据']
    hdr=None
    for i,row in enumerate(ws.iter_rows(values_only=True),1):
        if i==2: hdr=row
        if i==3: vals=row
        if i==11: hdr2=row
        if i==12: vals2=row
    for k,v in zip(hdr,vals):
        if k is not None and isinstance(v,(int,float)): out[k]=float(v)
    for k,v in zip(hdr2,vals2):
        if k is not None and isinstance(v,(int,float)): out[k]=float(v)
    wb.close(); return out

def literal_scan():
    keys=('frequency','power','gain','sensitivity','fade','system','obstacle','prepare','link','turn','hover','communication','comm_power')
    rows=[]
    for fn in ['src/q3_official_semantics.py','src/q3_dynamic_timeline.py','src/minimal_pipeline.py']:
        text=(ROOT/fn).read_text(encoding='utf-8').splitlines()
        tree=ast.parse('\n'.join(text),filename=fn)
        for n in ast.walk(tree):
            if isinstance(n,ast.Constant) and isinstance(n.value,(int,float)) and not isinstance(n.value,bool):
                line=text[n.lineno-1].lower()
                # Exclude indexing/boolean literals and the documented two-stage
                # charging constants; only parameter-like literals in formulas
                # remain candidates for an attachment-value hardcode.
                non_parameter_line = any(tok in line for tok in ('samples','all(x[','min(x[','communication_ok','link_ok'))
                allowed=(0.0,1.0,32.45,1000.0,3600.0,0.5,0.65,0.9,0.1,0.35)
                suspicious=any(k in line for k in keys) and not non_parameter_line and float(n.value) not in allowed
                if suspicious: rows.append({'source_file':fn,'line':n.lineno,'literal':n.value,'line_text':text[n.lineno-1].strip(),'suspicious':suspicious})
    return pd.DataFrame(rows, columns=['source_file','line','literal','line_text','suspicious'])

def verify(trace_path):
    tr=pd.read_csv(trace_path); wb=workbook_values(); matches=[]
    relay_map={'prep_s':'工位固定准备时间（s）','link_s':'建链时间（s）','turn_s':'架次周转时间（s）','hover_power_kw':'悬停功率（kW）','comm_power_kw':'通信附加功率（kW）','power_kw':'巡航功率（kW）','speed':'计划巡航速度（m/s）','climb_speed':'最大爬升速度（m/s）','descend_speed':'最大下降速度（m/s）','eta_climb':'爬升能耗效率','eta_descend':'下降能耗效率','energy_kwh':'能源组件可用能量（kWh）','component_inventory':'共享能源组件总数（组）','component_full_charge_s':'等效完全充电时间（s）'}
    for _,r in tr.iterrows():
        source=str(r['source_field']); field=source.split(':')[-1]; lookup=relay_map.get(field,field); val=float(r['runtime_value']); target=wb.get(source,wb.get(lookup))
        matches.append(target is not None and abs(val-target)<=1e-9)
    scan=literal_scan(); return bool(len(tr)>0 and all(matches) and not scan.suspicious.any()), matches, scan

def main():
    p=ROOT/'results/q3_runtime_parameter_trace.csv'; ok,matches,scan=verify(p); out={'runtime_rows':len(matches),'workbook_matches':int(sum(matches)),'suspicious_literals':int(scan.suspicious.sum()),'independent_provenance_pass':ok,'mutation_parameter_fails':False}
    with tempfile.TemporaryDirectory() as td:
        m=pd.read_csv(p); m.loc[0,'runtime_value']=float(m.loc[0,'runtime_value'])+1.0; q=Path(td)/'mut.csv';m.to_csv(q,index=False); out['mutation_parameter_fails']=not verify(q)[0]
    scan.to_csv(ROOT/'results/q3_runtime_literal_scan.csv',index=False); pd.DataFrame([{'metric':k,'value':v} for k,v in out.items()]).to_csv(ROOT/'results/q3_runtime_parameter_provenance.csv',index=False); (ROOT/'results/q3_parameter_provenance_verification.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
