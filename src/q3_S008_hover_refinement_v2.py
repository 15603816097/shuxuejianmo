from __future__ import annotations
import json, math
from pathlib import Path
import numpy as np, pandas as pd
from minimal_pipeline import load_inputs, terrain_profile
from q3_semantics_core import build_authoritative_timeline, direct_and_relay_blocks, xyz, link_obstructed

ROOT=Path(__file__).resolve().parents[1]; RES=ROOT/'results'
TOL=1e-9

def thresholds(data):
    c=data['comm']; s=float(c[('接收参数','接收灵敏度（dBm）')]); m=float(c[('接收参数','衰落裕量（dB）')]); l=float(c[('传播参数','系统损耗（dB）')])
    access=min(20+3+6-s-m-l,20+6+3-s-m-l)
    back=min(19+8+12-s-m-l,27+12+8-s-m-l)
    return access,back

def ground(data,p):
    vals=[v for _,v in terrain_profile(p,p,data['dem']) if np.isfinite(v)]
    return max(vals) if vals else float('nan')

def eval_hover(data, blocks, p, agl, ta, tb):
    g=ground(data,p)
    if not np.isfinite(g) or agl<50-TOL or agl>data['relay']['max_height_m']+TOL: return None
    h=g+agl; freq=float(data['comm'][('传播参数','载波频率（MHz）')]); obs=float(data['comm'][('传播参数','地形遮挡附加损耗（dB）')]); acc=[]
    for b in blocks:
        for q in b['items']:
            for ep,eh in ((q['a'],q['ha']),(q['b'],q['hb'])):
                d=float(np.linalg.norm(xyz(data,ep,eh)-xyz(data,p,h)))
                ol=obs if link_obstructed(data,ep,p,eh,h) else 0.0
                acc.append(ta-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+ol))
    c=data['center']; gh=float(c['海拔（m）'])+float(data['comm'][('固定网关 G01','天线离地高度（m）')]); d=float(np.linalg.norm(xyz(data,p,h)-xyz(data,c,gh))); ol=obs if link_obstructed(data,p,c,h,gh) else 0.0
    back=tb-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+ol)
    return {'x':float(p['经度（°）']),'y':float(p['纬度（°）']),'altitude':float(h),'AGL':float(agl),'terrain_elevation_m':float(g),'access_margin':float(min(acc)),'backhaul_margin':float(back),'joint_margin':float(min(min(acc),back))}

def eval_hover_fast(data, blocks, p, agl, ta, tb):
    """Coarse ranking only; finalists are re-evaluated by eval_hover."""
    lons=np.asarray(data['dem']['longitude']).ravel(); lats=np.asarray(data['dem']['latitude']).ravel(); zdem=np.asarray(data['dem']['dem'])
    def gfast(q):
        ix=int(np.abs(lons-float(q['经度（°）'])).argmin()); iy=int(np.abs(lats-float(q['纬度（°）'])).argmin()); v=float(zdem[iy,ix]); return v if np.isfinite(v) and v>-30000 else float('nan')
    g=gfast(p)
    if not np.isfinite(g) or agl<50-TOL or agl>data['relay']['max_height_m']+TOL:return None
    h=g+agl; freq=float(data['comm'][('传播参数','载波频率（MHz）')]); obs=float(data['comm'][('传播参数','地形遮挡附加损耗（dB）')]);
    def blocked(a,b,ha,hb):
        # conservative 100-point coarse ranking test, never used as final evidence
        ts=np.linspace(0.,1.,25); vals=[]
        for t in ts:
            lon=float(a['经度（°）'])+t*(float(b['经度（°）'])-float(a['经度（°）']))
            lat=float(a['纬度（°）'])+t*(float(b['纬度（°）'])-float(a['纬度（°）']))
            gg=gfast({'经度（°）':lon,'纬度（°）':lat})
            if np.isfinite(gg) and float(ha)+t*(float(hb)-float(ha))-gg<=0:return True
        return False
    acc=[]
    for b in blocks:
        for q in b['items']:
            for ep,eh in ((q['a'],q['ha']),(q['b'],q['hb'])):
                d=float(np.linalg.norm(xyz(data,ep,eh)-xyz(data,p,h))); ol=obs if blocked(ep,p,eh,h) else 0.; acc.append(ta-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+ol))
    c=data['center']; gh=float(c['海拔（m）'])+float(data['comm'][('固定网关 G01','天线离地高度（m）')]); d=float(np.linalg.norm(xyz(data,p,h)-xyz(data,c,gh))); ol=obs if blocked(p,c,h,gh) else 0.; back=tb-(32.45+20*math.log10(freq)+20*math.log10(max(d/1000,1e-12))+ol)
    return {'x':float(p['经度（°）']),'y':float(p['纬度（°）']),'altitude':float(h),'AGL':float(agl),'terrain_elevation_m':float(g),'access_margin':float(min(acc)),'backhaul_margin':float(back),'joint_margin':float(min(min(acc),back))}

def main():
    data=load_inputs(); ids=['S008-MED-01']; tl=build_authoritative_timeline(data,('S008',),'B',ids); blocks=direct_and_relay_blocks(data,tl); ta,tb=thresholds(data)
    lons=np.asarray(data['dem']['longitude']).ravel(); lats=np.asarray(data['dem']['latitude']).ravel();
    # Coarse global legal domain, then local polishing.
    rows=[]; seen=set()
    def add_fast(lon,lat,agl,stage):
        key=(round(lon,9),round(lat,9),round(agl,3))
        if key in seen:return
        seen.add(key); z=eval_hover(data,blocks,{'经度（°）':lon,'纬度（°）':lat},agl,ta,tb)
        if z: z.update({'stage':stage}); rows.append(z)
    coarse=[]
    for lon in np.linspace(float(lons.min()),float(lons.max()),15):
        for lat in np.linspace(float(lats.min()),float(lats.max()),20):
            for agl in (50.,100.,150.,200.,250.,300.):
                p={'经度（°）':float(lon),'纬度（°）':float(lat)}; z=eval_hover_fast(data,blocks,p,agl,ta,tb)
                if z: coarse.append(z)
    rows=[]
    # Local refinement remains a search-layer ranking pass; strict evaluation
    # is applied to its finalists below.
    for z in sorted(coarse,key=lambda x:x['joint_margin'],reverse=True)[:5]:
        for step_m in (5.,2.,1.):
            dlon=step_m/(6371008.8*math.cos(math.radians(float(z['y']))))*180/math.pi; dlat=step_m/6371008.8*180/math.pi
            for dx in np.linspace(-dlon,dlon,5):
                for dy in np.linspace(-dlat,dlat,5):
                    for da in (-2.,-1.,0.,1.,2.):
                        p={'经度（°）':z['x']+dx,'纬度（°）':z['y']+dy}; zz=eval_hover_fast(data,blocks,p,max(50.,min(300.,z['AGL']+da)),ta,tb)
                        if zz: zz['stage']=f'local_{step_m:g}m'; rows.append(zz)
    # Strictly re-evaluate the best search-layer candidates.
    strict=[]; strict_seen=set()
    # Audited pointwise binding-region seed, re-evaluated against the complete
    # four-block trajectory under the corrected obstacle rule.
    for sx,sy,sa in [(109.22252777777778,23.06485555555556,50.0),(109.21731067380951,23.06126800571429,92.5)]:
        zz=eval_hover(data,blocks,{'经度（°）':sx,'纬度（°）':sy},sa,ta,tb)
        if zz: zz['stage']='binding_region_seed_strict'; strict.append(zz)
    # Include a fresh re-evaluation of the archived legal-domain seed set;
    # coordinates are only seeds, all margins are recomputed with the fixed
    # v2 DEM-conditional evaluator.
    archived=RES/'q3_S008_final_refinement.csv'
    if archived.exists():
        old=pd.read_csv(archived).sort_values('backhaul_margin',ascending=False).head(40)
        for _,z in old.iterrows():
            zz=eval_hover(data,blocks,{'经度（°）':float(z['x']),'纬度（°）':float(z['y'])},float(z['height_offset']),ta,tb)
            if zz: zz['stage']='archived_seed_strict'; strict.append(zz)
    for z in sorted(rows,key=lambda x:x['joint_margin'],reverse=True)[:40]:
        key=(round(z['x'],9),round(z['y'],9),round(z['AGL'],3))
        if key in strict_seen: continue
        strict_seen.add(key); zz=eval_hover(data,blocks,{'经度（°）':z['x'],'纬度（°）':z['y']},z['AGL'],ta,tb)
        if zz: zz['stage']='strict_final'; strict.append(zz)
    out=pd.DataFrame(strict).sort_values('joint_margin',ascending=False); out.to_csv(RES/'q3_S008_hover_refinement_v2.csv',index=False,encoding='utf-8-sig')
    best=out.iloc[0].to_dict(); summary={'route':'O01->S008->O01','box_set':ids,'drone_type':'B','relay_required_block_count':len(blocks),'candidate_count':len(out),'best':best,'old_best_joint_margin':-0.0771202430435664,'improvement_db':float(best['joint_margin']+0.0771202430435664),'positive':bool(best['joint_margin']>0),'pipeline_sha256':'v2_pending'}
    (RES/'q3_S008_hover_refinement_v2_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,default=str),encoding='utf-8'); print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
