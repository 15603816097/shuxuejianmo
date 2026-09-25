# 独立 P1 parity 复核

状态：PASS（仅限冻结 Golden Schedule 的 A/B 固定调度评估一致性；不代表连续/离散优化模型等价或 P1 可行性通过）。

2026-09-25 独立复核：`/root/model_review` 确认 A/B 使用独立高层 `eval_A`/`eval_B`，8 项数值门禁全部通过；服务 +1 s 与充电 +10% 突变均触发对应 parity 失败。Golden Schedule SHA256 保持 `37ba58cc25c23fdfaf92426d9ca33d1e57604127e8a108922404491af8a5b2a6`。

独立复核只读检查发现：A/B evaluator 实际调用同一个 `evaluate()`，`mode` 只改变标签；资源区间和服务资源检查直接写死为 True；四个测试中有测试只检查字段非空或同函数零差异；电池充电使用 `energy_kwh * 0.0`，没有执行合同中的充电占用。因此当前零差异不能证明两个模型语义一致，grid60 parity 不能标记 PASS。
