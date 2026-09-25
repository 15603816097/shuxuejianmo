from pathlib import Path
import json
import pandas as pd
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'figures'; out.mkdir(exist_ok=True)
    s=json.loads((ROOT/'results/current_result_summary.json').read_text())
    m=s['R2_metrics']; sens=pd.read_csv(ROOT/'results/resource_sensitivity.csv')
    pd.DataFrame([m]).to_csv(ROOT/'results/current_q2_metrics.csv',index=False)
    pd.DataFrame([{'resource':'transport','peak':m['transport_peak']},{'resource':'relay','peak':m['relay_peak']},{'resource':'battery','peak':m['battery_peak']},{'resource':'service','peak':m['service_peak']}]).to_csv(ROOT/'results/current_resource_peaks.csv',index=False)
    pd.DataFrame([{'方案':'60秒模型','late_boxes':14,'late_sorties':7,'status':'baseline'},{'方案':'连续时间11箱方案','late_boxes':m['best_known_late_boxes'],'late_sorties':m['best_known_late_sorties'],'status':'best-known feasible'}]).to_csv(ROOT/'results/current_vs_grid60.csv',index=False)
    fig,ax=plt.subplots(figsize=(5.0,3.2)); q=sens[sens.status.astype(str).isin(['0','KNOWN_FEASIBLE'])].copy(); q['R']=q.R.astype(int); q['late_boxes']=q.late_boxes.astype(float); ax.plot(q.R,q.late_boxes,'o-',color='#0072B2'); ax.set_xlabel('Relay inventory R'); ax.set_ylabel('Observed feasible late boxes'); ax.set_title('Finite resource sensitivity (best-known observations)'); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(out/'result_q4_resource_sensitivity.png',dpi=300); fig.savefig(out/'result_q4_resource_sensitivity.svg'); plt.close(fig)
    fig,ax=plt.subplots(figsize=(5.0,3.2)); names=list(m for m in ['transport_peak','relay_peak','battery_peak','service_peak']); vals=[s['R2_metrics'][x] for x in names]; ax.bar(['Transport','Relay','Battery','Service'],vals,color=['#0072B2','#D55E00','#009E73','#CC79A7']); ax.set_ylabel('Peak concurrent units'); ax.set_title('R=2 best-known schedule resource peaks'); fig.tight_layout(); fig.savefig(out/'result_q2_resource_peaks.png',dpi=300); fig.savefig(out/'result_q2_resource_peaks.svg'); plt.close(fig)
if __name__=='__main__': main()
