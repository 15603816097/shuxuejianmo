# 官方 DOCX 公式对象 → 当前代码实现审计

审计日期：2026-09-25

## 审计边界与证据

用户指定的路径 `/mnt/data/山区洪涝灾害下无人机运输与通信协同优化.docx` 在当前容器中不存在（文件检查退出码为 1）。项目目录中的同名副本 `/home/chenkeming/shuxuejianmo/山区洪涝灾害下无人机运输与通信协同优化.docx` 存在，本次以该副本为审计输入；若两份文件并非同一版本，结论需要重新核验。

未采用普通文本抽取作为唯一依据。已直接解压并读取 `word/document.xml` 中的 OMML 数学对象：`m:oMath=72`，`m:oMathPara=12`。已核验的 OMML 原式包括 P47、P50、P53、P56、P63、P74、P77、P80、P83、P86、P89、P93。DOCX 的可视化 PDF 渲染在本环境中未成功产生输出文件（LibreOffice/soffice helper 均未生成 PDF），因此“页面视觉复核”仍是环境阻塞项；公式对象审计已完成，但不能宣称完成了视觉页面审计。

代码证据主要来自：

- `src/minimal_pipeline.py:155-198`（`energy_leg`、等效航程、飞行时间、能耗）；
- `src/minimal_pipeline.py:245-281`（Q2 送达、返航、充电和硬截止后评估）；
- `src/minimal_pipeline.py:284-371`（`_loss_db`、`_segment_cert`、静态 Q3 认证）；
- `src/continuous_joint.py:15-18,19-97`（充电和连续时间 MILP 语义）；
- `src/q3_stageC1_corrected.py:13-34`（中继分阶段事件与动态通信占位）；
- `src/q3_reoptimize_from_q2.py:40-105`（中继排队、能耗和资源事件）。

## A. EXACT_MATCH（11 项）

| # | 官方公式/规则（OMML 或正文） | 官方变量/单位 | 当前代码 | 判定 |
|---|---|---|---|---|
| A1 | `L_g(q)=L_g^0-(L_g^0-L_g^F)(q/Q_g)^(3/2)`（P47） | q、Q_g 为 kg；航程 m | `minimal_pipeline.energy_leg` 使用 `range_empty - (range_empty-range_full)*frac**1.5` | 等价 |
| A2 | `t_gij=h_ij^+/v_g^↑+d_ij/v_g^c+h_ij^-/v_g^↓`（P50） | h、d 为 m；速度 m/s；时间 s | `energy_leg` 分别累加爬升、巡航、下降时间 | 等价 |
| A3 | `E_gij=E_gij^hor+E_gij^up`，下降附加能耗效率为 0（P53、P55） | 能耗 kWh | `energy_leg` 水平航程能耗 + 爬升附加能耗，没有下降附加项 | 等价 |
| A4 | `E_p^T=ΣE_gij(q_pij)≤(1−ρ_g)E_g^use`（P56） | ρ 为比例；E kWh | `route_stats.safe` 与 `q1` 用 `energy <= (1-reserve)*energy` | 等价 |
| A5 | `t_chg(s)=T_full[0.65(0.90-s)/0.90+0.35]`（s<0.90）；`T_full·0.35(1-s)/0.10`（s≥0.90）（P63） | s 无量纲；时间 s | `minimal_pipeline.py:265-271`、`continuous_joint.charge_time` 对运输电池实现同式 | 等价（仅运输电池） |
| A6 | 有效接收门限 `P_th^b=P_sens^b+M_b`（P77） | dBm/dB | `q3` 从通信参数读取 `P_sens` 与 `M`，构造方向门限 | 等价 |
| A7 | `L_max^{a↔b}=min(L_max^{a→b},L_max^{b→a})`（P83） | dB | `q3` 对直连和回传使用双向最小门限 | 等价 |
| A8 | `L_FSPL=32.45+20log10 f+20log10 D`（P86） | f MHz、D km、损耗 dB | `_loss_db` 将 m 转 km，使用 2400 MHz 和同一公式 | 等价 |
| A9 | `L_path=L_FSPL+L_obs^{b}`（P89） | L_obs dB | `_loss_db` 对遮挡加 10 dB，参数值与通信 xlsx 一致 | 等价（当前参数值） |
| A10 | 单链路 `A=1` 当 `L_path≤L_max`，否则 0（P93） | 无量纲状态 | `_segment_cert` 同时检查损耗阈值和净空 | 公式判定等价 |
| A11 | 货箱送达应使用到达服务区并完成交接的时刻；Q2 完成时间取最后返航（题目正文） | 时间 s | `q2` 用 `deliver=depart+out_flight+handoff`，`ret` 单独用于资源释放/ makespan | 时间点分离正确 |

## B. IMPLEMENTATION_DIFFERENCE（11 项）

| # | 官方要求 | 当前代码证据 | 差异及影响 |
|---|---|---|---|
| B1 | 中继飞行时间也按爬升、巡航、下降阶段计算（P59） | `q3`/`strict_q3` 只用候选段水平/三维距离除巡航速度；没有中继起点/悬停点作业高度的完整阶段时间 | 中继占用和联合 makespan 被低估或错配，直接影响 Q3 排时 |
| B2 | 中继服务能耗由悬停功率与通信附加功率共同确定（P59） | `minimal_pipeline.q3`、`q3_reoptimize_from_q2` 用 `power_kw*distance/speed/3600`，未加入悬停功率和通信附加功率 | 中继总能耗被低估，能源可行性和能耗目标错误 |
| B3 | 中继飞行能耗按巡航功率/巡航时间和爬升附加能耗，质量取计划起飞总质量（P59） | 代码把中继路程直接乘巡航功率；没有 `energy_leg` 式爬升附加项 | 中继能源/SOC 与返航余量不一致 |
| B4 | `L_obs`、f、L_sys、阈值应统一由通信参数和实际方向确定 | `_segment_cert` 的 `link` 参数未使用，内部硬编码 `2400、3、10`；`q3` 门限计算读取参数 | 当前附件数值恰好相同，但实现不能随参数变化；敏感性实验可能不代表合同公式 |
| B5 | 地形遮挡按同一时刻通信端点三维位置和 30 m DEM 判定（P70、P73） | `_segment_cert` 是静态线段证书；移动运输机接入链路在 `minimal_pipeline.q3` 用 33 个 route 点，`final_joint.strict_q3` 直接按固定线段 | 未形成全飞行时域的动态链路判定，可能错误地把静态通过当作连续通信通过 |
| B6 | 中继服务必须在到位、建链后覆盖真实服务时段，准备/建链/服务/返航/周转应分开 | `continuous_joint.py:84` 将中继区间压成 `[-prep, return+turn)`；`q3_reoptimize_from_q2.py:104` 同样合并 | 中继排队时间可能被夸大或缩小；不能从一个区间区分本体、通信服务和周转 |
| B7 | 能源组件是独立资源，任务占用和充电不可重叠（P61-P64） | `q3_stageC1_corrected.py` 只产生 `relay_energy_component/use`，`energy_charge_end=np.nan`，报告明确 `energy_component_charge_defined=False` | 中继能源组件库存/周转没有真正进入 Q3 可行性判定 |
| B8 | 中继本体与能源组件可以分别建账；更换组件不应锁死本体 | C1 虽生成两个资源名，但本体区间仍按运输任务近似，能源组件充电未接入 | 资源峰值“2”只代表当前占用事件，不代表合同下完整组件周转可行 |
| B9 | 通信状态按每一时刻：直连可用→直连；否则两段同时可用→中继；否则中断（P95-P100） | `q3_stageC1_corrected.py:31-33` 将 `communication_ok=False` 并标记 `DYNAMIC_UNVERIFIED`；没有动态时间片状态机 | 不能把当前 Q3 结果称为动态通信 PASS |
| B10 | 直连可用时不应占用中继；只有 relay-required 区间占用中继 | C1 对需要中继的任务把 service 区间按 `start_s` 至 `return_s` 处理，早期 `q3_reoptimize_from_q2.py` 也按整架次 | 中继库存和排队会被过度占用，正是当前 Q3 固定迟到结构的直接风险来源 |
| B11 | 硬截止应作为 Q3 硬约束；普通物资期望时间才用于软及时性 | `continuous_joint.py:56-65` 使用 `arrival≤deadline+M*tardy_box`；常规目标允许 `tardy_box=1`，仅 `violation_cap=0` 时才强制为 0 | 默认 Stage C1/优化语义允许硬截止违约，事后统计的 24 箱不能证明硬约束已满足 |

## C. MISSING_IN_CODE（5 项）

| # | 官方要求 | 当前代码缺失 | 影响 |
|---|---|---|---|
| C1 | 中继的完整 `prepare → outbound flight → link setup → service → return → turnaround` 时间链 | 没有以官方飞行阶段公式计算中继 outbound/return 的独立开始/结束时间 | 无法严格计算中继本体忙闲和联合完成时间 |
| C2 | 中继通信服务持续时间及其悬停/通信附加能耗 | 没有独立的服务开始/结束变量与由通信需求确定的服务功率累计 | 无法严格计算中继服务能耗 |
| C3 | 中继能源组件的 `SOC → t_chg(s) → available_again` 资源链 | 无 `R` 能源组件数量/逐组件充电结束时间的完整约束；当前 C1 直接留下 NaN | 中继能源组件资源可行性未闭合 |
| C4 | 全飞行过程的动态通信状态表 | `dynamic_communication_audit` 目前明确写入 `DYNAMIC_UNVERIFIED` / `communication_ok=False` | “全部时段通信可行”尚未实现 |
| C5 | 动态直连/中继切换区间的统一事件生成器 | 没有 `direct_available_intervals` 与 `relay_required_intervals` 的正式实现 | 无法验证直连可用时中继是否被错误占用 |

## D. UNVERIFIED_ASSUMPTION（4 项）

| # | 代码假设 | 官方来源状态 | 风险 |
|---|---|---|---|
| D1 | `terrain_profile` 用 DEM 栅格穿越后的分段/像元高程做净空；未实现官方未明确写出的双线性插值 | DOCX 只规定 30 m DEM 和视线遮挡，未规定插值方法 | 净空边界附近的连续认证可能因插值规则改变 |
| D2 | 中继服务区间在 `corrected_events` 中近似为运输架次 `start_s` 至 `return_s` | DOCX 只规定到位建链后服务，未给出“服务必须覆盖整架次”的公式 | 会把不需要中继的直连时段计入资源和能耗 |
| D3 | 中继能源组件 SOC 使用方式和充电功率在当前代码中没有从中继 xlsx 完整接入 | DOCX 明确要求组件 SOC/两阶段充电；附件给出 R 型组件库存 6、`T_full=1800 s`，但当前实现未使用 | 无法确认中继资源是否真实周转可行 |
| D4 | `_segment_cert` 的 `link` 参数代表各链路设备参数，但函数内部忽略它而固定使用参数 | 具体固定值可在通信 xlsx 找到，但“函数参数被忽略”的设计没有官方依据 | 参数替换、阈值敏感性及方向性链路审计可能失真 |

## Q3 卡点直接影响

会直接改变 Q3 可行性或最优性判断的差异为：B1、B2、B3、B5、B6、B7、B9、B10、B11、C1-C5、D2-D3。尤其是 B10/C4：当前中继整架次占用和动态通信未实现，会同时改变中继排队、硬截止、服务状态和资源峰值；B2/B3/C3 会改变中继能耗和能源组件周转；B11 会把“允许迟到的 MILP”误读成“硬截止模型”。

## 结论与门禁

本次共核验 **31 条公式/规则**：**EXACT_MATCH 11，IMPLEMENTATION_DIFFERENCE 11，MISSING_IN_CODE 5，UNVERIFIED_ASSUMPTION 4**。当前不建议继续 Q3 优化。应先修复并单元验证：中继阶段时间、悬停+通信能耗、能源组件 SOC/充电、动态通信状态、relay-required 区间，以及把医疗/首批截止改为真正不可放松的硬约束；修复后再重新生成 Q3 方案和资源审计。Q2/Q3/Q4 既有结果本轮未修改。
