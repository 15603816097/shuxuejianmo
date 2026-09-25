# Q3 官方公式与资源语义修复记录

本轮没有运行 Q3 大规模优化，没有进入 C2，没有修改 Q2 结果。

## 已修复

1. **硬截止**：`src/continuous_joint.py` 现在对医疗物资和首批保障箱直接加入 `delivery_time <= hard_deadline`；其 `tardy_box` 上界固定为 0，不能放松约束。普通货物仍作为软及时性变量处理。
2. **中继飞行时间**：新增 `src/q3_official_semantics.py::relay_leg_time_energy`，严格计算爬升、水平巡航、下降三个阶段。
3. **中继能耗**：加入往返飞行巡航能耗、爬升附加能耗、悬停功率与通信附加功率乘以服务时间。
4. **中继时间链**：新增 prepare、outbound flight、link setup、relay service、return flight、turnaround 的独立事件边界。
5. **能源组件**：`load_inputs()` 读取 R 型组件 6 组库存和 `T_full=1800 s`；新增 SOC、返航余量、任务占用、两阶段充电、available_again 计算。充电事件不锁死中继机体。
6. **动态通信接口**：新增 `dynamic_communication_audit`，逐运输轨迹阶段检查 direct，或 access/backhaul 两段同时可用；直连区间不产生 relay requirement。
7. **C1 事件生成**：`q3_stageC1_corrected.py` 改为使用官方中继阶段链，并分别生成中继本体和能源组件 use/charge 事件。

## 自动测试

命令：

```bash
PYTHONPATH=.deps:src python -m unittest discover -s tests -p 'test_q3_official_semantics.py' -v
```

结果：**9/9 PASS，退出码 0**。

覆盖：硬截止不可放松、中继飞行时间公式、中继服务能耗、SOC、两阶段充电、6 组库存、动态通信、直连时不占用中继，以及 `build_model` 的不可放松硬截止约束。

## 代表性任务闭环

命令：

```bash
PYTHONPATH=.deps:src python src/q3_official_representative.py
```

代表任务：`Q2-001`，真实数据和真实 DEM/Q3 认证候选。

- 轨迹阶段数：7（爬升、巡航、下降、交接、返航爬升、返航巡航、返航下降）
- 代表任务最新中继总能耗：`0.406467506407959 kWh`
- 能源组件 SOC/返航余量：PASS
- 代表任务最新组件充电时间：`665.1274244782333 s`
- 动态通信：代表任务按真实端点/中点抽查；程序识别爬升与返航下降接入链路不可用，因此 `all_communication_ok=False`，这是动态判定的真实结果，不强行标记通过
- 硬截止手算与程序：PASS

详细输出：

- `results/q3_official_representative_audit.json`
- `logs/q3_official_representative_audit.md`

## 尚未闭合的假设

原审计中的 4 个 `UNVERIFIED_ASSUMPTION` 已闭合能源组件 SOC/充电项，当前至少保留 4 项：

1. DOCX 规定 30 m DEM，但未明确双线性还是分段常值插值；
2. DOCX 未给出中继服务时长的独立数值，本实现由调用方显式传入，代表任务暂以交接服务时长作为测试输入；
3. `_segment_cert` 的 `link` 参数仍未完全参数化，部分通信常量仍在底层函数内部固定。
4. 当前动态审计在每个飞行阶段取端点和中点，尚未按移动端点穿越的全部 DEM 事件进行严格连续证明。

因此，本轮已具备继续开发 Q3 动态语义的最小代码基础，但尚未具备重新运行全量 Q3 的条件；必须先补齐移动端点连续通信认证、确定 DEM 插值规则和服务时长来源，并完成底层通信参数完全参数化后再运行 Q3。
