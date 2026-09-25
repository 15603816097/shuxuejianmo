"""Independent CSV-only verifier and mutation checks for the strict certificate."""
from __future__ import annotations
import json, tempfile
from pathlib import Path
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]

def verify(cert_path, los_path):
    c=pd.read_csv(cert_path); l=pd.read_csv(los_path)
    if len(c)==0: return False
    # Recompute from numeric inequalities; producer pass flags are ignored.
    budget=(c['path_loss_upper_bound'] <= c['threshold'] + 1e-9) & (c['margin_lower_bound'] >= -1e-9)
    # The LOS table is a geometry consistency certificate.  Each stored
    # clearance must equal minimum_los_height - DEM height; a mutated DEM
    # height therefore cannot pass silently.  Communication itself uses the
    # worst-case obstacle penalty in `budget`, so no sampled LOS label is used.
    if not len(l): return False
    rel=(l['clearance_lower_bound'] - (l['minimum_los_height']-l['dem_height'])).abs() <= 1e-8
    geom=l.groupby('primitive_interval').apply(lambda g: bool(rel.loc[g.index].all()), include_groups=False)
    geom_ok=c['interval_id'].map(geom).fillna(False).astype(bool)
    return bool((budget & geom_ok).all())

def main():
    c=ROOT/'results/q3_Q2_001_continuous_certification.csv'; l=ROOT/'results/q3_Q2_001_los_certificate.csv'; result=verify(c,l)
    out={'independent_certificate_pass':result,'primitive_intervals':int(len(pd.read_csv(c))),'los_rows':int(len(pd.read_csv(l)))}
    (ROOT/'results/q3_Q2_001_continuous_certificate_verification.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    # Mutations must fail the independent verifier.
    cc=pd.read_csv(c); ll=pd.read_csv(l); cc.loc[0,'margin_lower_bound']=-1.0; ll.loc[0,'dem_height']+=1000.0
    with tempfile.TemporaryDirectory() as td:
        cp=Path(td)/'c.csv'; lp=Path(td)/'l.csv'; cc.to_csv(cp,index=False); ll.to_csv(lp,index=False); out['mutation_los_or_margin_fails']=not verify(cp,lp)
        cc2=pd.read_csv(c); cc2.loc[0,'margin_lower_bound']=-1.0; cc2.to_csv(cp,index=False); pd.read_csv(l).to_csv(lp,index=False); out['mutation_margin_fails']=not verify(cp,lp)
        ll2=pd.read_csv(l); ll2.loc[0,'dem_height']+=1000.0; pd.read_csv(c).to_csv(cp,index=False); ll2.to_csv(lp,index=False); out['mutation_los_fails']=not verify(cp,lp)
    (ROOT/'results/q3_continuous_certificate_mutation_test.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
