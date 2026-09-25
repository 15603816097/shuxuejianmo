# 离散/连续约束一致性矩阵

审查对象：A=`src/final_joint.py`（60 s MILP）；B=`src/continuous_joint.py`（连续时间析取模型）。

| 项目 | A 60 s 模型 | B 连续模型 | 状态 | 证据 |
|---|---|---|---|---|
| 任务集合 | Q1 批次 18 架次 | `q2()` 固定同一 18 批次 | MATCH | `minimal_pipeline.py:q1,q2` |
| 箱子-任务映射 | `box_ids` 固定 | 同一 `box_ids` | MATCH | 两脚本 `box_ids` |
| 可变组批 | 未开放，继承 Q1 贪心批次 | 未开放，继承 Q1 贪心批次 | MATCH | `q1()` |
| 批次容量 | 质量/体积/能耗已在 Q1 检查 | 继承批次，未重新检查 | DIFFERENT | `q1()` vs `continuous_joint.py:build_model` |
| 起飞时间 | `s=60k` | 连续 `s>=0` | DIFFERENT | `final_joint.py:milp_schedule` vs `continuous_joint.py:sidx` |
| 到达时间 | `delivery=start+out_flight+handoff` | 同一偏移 | MATCH | 两脚本 `delivery_s` |
| 硬截止定义 | 医疗物资期望时间+首批截止 | 所有箱子均使用期限 | DIFFERENT | `minimal_pipeline.q2` vs `continuous_joint` deadlines |
| 迟到箱数 | 逐箱 `hard_ok` | 迟到二元变量 | DIFFERENT | `q2()` vs `tardy_box_*` |
| 运输机资源 | 聚合容量 | 物理单元分配+析取 | DIFFERENT | `final_joint` capacity rows vs `y_air/ord` |
| 中继资源 | 聚合峰值 | R 个单元+准备/周转区间 | DIFFERENT | `final_joint` vs `y_rel` |
| 电池库存 | 聚合/保守充电窗口 | 分型电池单元+充电持续时间 | DIFFERENT | `final_joint` vs `y_bat` |
| 服务资源 | 事件式服务区间 | 同服务任务两两析取 | DIFFERENT | `q4()` vs service order constraints |
| 中继候选 | `strict_q3()`候选 | 读取同一认证表 | MATCH | `final_q3_continuous_certification.csv` |
| DEM认证 | 逐像元路径证书 | 读取证书，不重新认证 | MATCH | `strict_q3()` |
| 通信阈值 | 双向预算 | 读取证书可行性 | MATCH | `strict_q3()` |
| 中继能耗 | 证书阶段未进入调度约束 | 未进入连续调度 | MISSING | 两脚本均无 relay energy constraint |
| 运输能耗 | Q1 批次固定能耗 | 继承固定能耗 | MATCH | `q1()/q2()` |
| 装卸/服务时间 | 进入 `delivery_s/return_s` | 进入偏移和服务析取 | MATCH | `q2()`、`build_model()` |
| 资源释放 | 飞行结束；部分输出含充电/周转 | 电池充电、中继周转区间 | DIFFERENT | `final_joint.py` events vs continuous pairs |
| 区间端点 | 事件扫描半开近似 | Big-M 不重叠 | DIFFERENT | `q4()` vs pair constraints |
| 前后依赖 | Q2 earliest-available 解码 | 由资源析取产生 | DIFFERENT | `q2()` vs `pair()` |
| K 分组 | K=2/K=3 后处理 | 未建模 K 分组 | MISSING | `continuous_joint.py` |

结论：共有 11 项 DIFFERENT、2 项 MISSING、9 项 MATCH。当前两套模型不能直接用结果数值互相验证；必须先统一硬截止口径、服务资源、释放区间和中继能耗。
