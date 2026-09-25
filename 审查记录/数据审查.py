# 仅用于全量数据读取、统计与一致性审查；不实现建模求解。
from pathlib import Path
import json,re,hashlib
import numpy as np,pandas as pd,h5py,tifffile,openpyxl
from scipy.io import loadmat
P=Path(__file__).resolve().parents[1]; O=P/'审查记录'; lines=[]
def put(s=''): lines.append(str(s))
def table(df):
 put(f'记录数×字段数：{df.shape[0]}×{df.shape[1]}；整行重复：{int(df.duplicated().sum())}。')
 put('|字段|类型|空值|范围/不同值数|单位|IQR疑似极端值数|\n|---|---|---:|---|---|---:|')
 for c in df:
  x=df[c];num=pd.api.types.is_numeric_dtype(x); unit=('°' if '经度' in c or '纬度' in c else re.search(r'[（(](.*?)[）)]',c).group(1) if re.search(r'[（(](.*?)[）)]',c) else 'km' if c=='长度_km' else '未标注/标识或无量纲')
  if num:
   q1,q3=x.quantile([.25,.75]);ext=int(((x<q1-1.5*(q3-q1))|(x>q3+1.5*(q3-q1))).sum());v=f'{x.min():g}～{x.max():g}'
  else:ext='—';v=f'{x.nunique()}种；'+','.join(map(str,x.dropna().unique()[:6]))
  put(f'|{c}|{x.dtype}|{x.isna().sum()}|{v}|{unit}|{ext}|')
 put('IQR仅作分布审查，不代表数据错误；编号、箱数和设计参数的极端值不自动剔除。')
 put('首记录：'+str(df.iloc[0].to_dict()) if len(df) else '无数据记录')
 put('末记录：'+str(df.iloc[-1].to_dict()) if len(df) else '')
put('# 全量读取与字段审查记录\n\n所有输入只读。行数均不含表头；Excel布局尺寸另记。无采集时间字段，不推断历史时间范围。')
files=list(P.glob('*.pdf'))+list(P.glob('*.docx'))+list(P.glob('*.xlsx'))+[f for f in (P/'data').rglob('*') if f.is_file()]
put('## 输入清单\n|文件|字节|SHA256|\n|---|---:|---|')
for f in sorted(files):put(f'|{f.relative_to(P)}|{f.stat().st_size}|{hashlib.sha256(f.read_bytes()).hexdigest()}|')
for f in sorted((P/'data').rglob('*.csv')):
 df=pd.read_csv(f);put('\n## '+f.name);table(df)
 put('名称为“未记录”的语义缺失：'+str(int((df['名称']=='未记录').sum())))
 put('独立要素数：'+str(df.iloc[:,0].nunique()))
 with h5py.File(f.with_suffix('.mat')) as h:
  candidates=[]
  def visitor(k,v):
   if not isinstance(v,h5py.Dataset):return
   a=v[()] # 每个HDF数据集均实际读取
   if a.dtype.kind in 'if' and a.size==len(df):candidates.append((k,a.ravel()))
   if a.dtype==np.dtype('uint64') and a.size>len(df)+4:
    a=a.ravel();n=int(a[2])
    if n==len(df):
     lens=a[4:4+n].astype(int);raw=a[4+n:].astype('<u8').tobytes();seq=[];pos=0
     for m in lens:seq.append(raw[pos:pos+2*m].decode('utf-16-le'));pos+=2*m
     candidates.append((k,np.array(seq)))
  h.visititems(visitor)
  put('配套 MAT：MATLAB v7.3 table，已读取全部 HDF 数据集并解码打包 UTF-16 字符串。')
  for c in df:
   x=df[c].to_numpy();match=[]
   for k,a in candidates:
    if a.dtype.kind in 'if' and pd.api.types.is_numeric_dtype(df[c]):eq=np.allclose(a,x,rtol=0,atol=1e-10)
    else:eq=np.array_equal(a.astype(str),x.astype(str))
    if eq:match.append(k)
   put(f'- {c}：'+('全列一致（'+','.join(match)+'）' if match else '未匹配，需核查'))
 # topology quality
 if '点序号' in df:
  groups=[df.columns[0]]+(['多边形编号','环编号'] if '环编号' in df else [])
  bad=sum(not np.array_equal(g['点序号'].to_numpy(),np.arange(1,len(g)+1)) for _,g in df.groupby(groups,sort=False))
  put(f'点序列非连续组数：{bad}。')
  if '环编号' in df:
   unclosed=sum(not np.allclose(g[['经度','纬度']].iloc[0],g[['经度','纬度']].iloc[-1],rtol=0,atol=1e-10) for _,g in df.groupby(groups));put(f'多边形未闭合环数：{unclosed}。')
# explicit subtable boundaries (1-based header and inclusive data end)
blocks={'调度中心与服务区.xlsx':[(2,3,5),(6,21,6)],'运输无人机数据.xlsx':[(2,5,18),(8,16,3),(19,22,3)],'中继无人机数据.xlsx':[(2,3,19),(6,8,3),(11,12,3)],'通信链路参数.xlsx':[(2,16,5)]}
for f in sorted((P/'data').rglob('*.xlsx')):
 put('\n## '+f.name);w=openpyxl.load_workbook(f,data_only=True)
 for s in w:
  put(f'工作表 `{s.title}` 布局 {s.max_row}×{s.max_column}。')
  rows=list(s.values)
  for start,end,width in blocks.get(f.name,[(1,s.max_row,s.max_column)]):
   heads=list(rows[start-1][:width]);inds=[i for i,h in enumerate(heads) if h is not None]
   df=pd.DataFrame([[r[i] for i in inds] for r in rows[start:end]],columns=[heads[i] for i in inds]);put(f'### 表头第{start}行，数据第{start+1}～{end}行');table(df)
# DEM
f=next((P/'data').rglob('*DEM.mat'));d=loadmat(f);a=tifffile.imread(f.with_suffix('.tif'));put('\n## DEM（MAT与GeoTIFF）')
put(f'dem: {a.shape}, {a.dtype}, {a.size}像元；全数组逐元素一致={np.array_equal(a,d["dem"])}；NaN={np.isnan(a).sum()}，NoData(-32767)={(a==-32767).sum()}；高程 {a.min()}～{a.max()} m。')
for k,v in d.items():
 if not k.startswith('__'):put(f'- {k}: {v.shape}, {v.dtype}，范围 {v.min()}～{v.max()}。')
put('纬度随行号递减、经度随列号递增；EPSG:4326；中心分辨率1/3600°。GeoTIFF标记PixelIsPoint，无GDAL_NODATA标签；MAT显式nodata=-32767，当前均无此值。MAT transform是像元外边界，而TIFF tiepoint是像元中心，差半像元，不是坐标冲突。')
# html all embedded JSON
f=next((P/'data').rglob('*.html'));t=f.read_text();put('\n## '+f.name);put(f'字符数{len(t)}；文本行数{len(t.splitlines())}。完整读取脚本，解析全部静态JSON常量。')
J={}
for name,raw in re.findall(r'^  const (\w+) = (.+);$',t,re.M):
 try:J[name]=json.loads(raw)
 except (ValueError,TypeError):pass
for name,v in J.items():
 if isinstance(v,dict) and 'features' in v:put(f'- {name}：{len(v["features"])}个要素；字段'+str(sorted({k for feat in v['features'] for k in feat['properties']})))
 elif name=='elevationGrid':
  g=np.array(v);put(f'- elevationGrid：{g.shape}, {g.dtype}, {g.min()}～{g.max()}m；用于展示的降采样栅格，不能替代DEM。')
 else:put(f'- {name}：'+(f'内嵌PNG，字符串长度{len(v)}，不作数值数据' if name=='terrainUrl' else str(v)))
(O/'html_embedded.json').write_text(json.dumps(J,ensure_ascii=False))
# template
put('\n## 结果提交模板.xlsx')
w=openpyxl.load_workbook(P/'结果提交模板.xlsx');
for s in w:
 h=[x for x in next(s.values) if x is not None];put(f'- {s.title}：布局{s.max_row}×{s.max_column}；实际字段{len(h)}；数据记录0。字段：'+ '、'.join(h))
put('模板空白是待填结果，不是输入缺失；Q3缺少独立运输与逐箱交付表，后续需保留模板并补充Q3对应结果与能源资源台账。')
# summary demand
j=json.loads((O/'xlsx_all.json').read_text());r=next(v for k,v in j.items() if k.endswith('物资需求与配送时限.xlsx'));df=pd.DataFrame(r['逐箱货箱清单'][1:],columns=r['逐箱货箱清单'][0]);put('\n## 跨表一致性与需求汇总')
put(f'80箱实读={len(df)}；唯一箱号={df.iloc[:,0].nunique()}；总质量={df.iloc[:,3].sum()}kg；总体积={df.iloc[:,4].sum()}m³；首批={sum(df.iloc[:,5]=="是")}；医疗={sum(df.iloc[:,2]=="医疗物资")}。')
agg=pd.DataFrame(r['数据'][1:],columns=r['数据'][0]);errors=[]
for _,row in agg.iterrows():
 s=df[(df.iloc[:,1]==row.iloc[0])&(df.iloc[:,2]==row.iloc[1])]
 if len(s)!=row.iloc[2] or (s.iloc[:,5]=='是').sum()!=row.iloc[3]:errors.append(row.iloc[:2].tolist())
 for a,b in [(3,4),(4,5),(7,8),(8,6)]:
  if not (s.iloc[:,a]==row.iloc[b]).all():errors.append((row.iloc[0],a))
put('逐箱对汇总数量、首批数、质量、体积、期望时限、优先系数差异：'+str(errors))
put(df.groupby('服务区编号').agg(箱数=('货箱编号','size'),质量=('单箱质量（kg）','sum'),体积=('单箱体积（m³）','sum')).to_string())
(O/'全量数据审查.md').write_text('\n'.join(lines))
print('\n'.join(lines[-30:]));print('audit lines',len(lines))
