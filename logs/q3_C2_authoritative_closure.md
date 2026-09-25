# Q3 C2 代表路线语义闭环审计

路线：O01 → S011 → S008 → O01。

- authoritative timeline：`results/q3_C2_authoritative_timeline.csv`，包含装载、三段飞行阶段、逐箱交接和返航结束；每个箱子的 delivery_time 是实际交接结束时刻。
- deadline：医疗物资与首批保障箱按硬时限逐箱比较，普通物资只记录 soft 口径。
- relay ledger：`results/q3_C2_S008_relay_component_ledger.csv`，调用正式两阶段充电和 SOC/reserve 检查。
- hover candidates：按每个 relay-required block 的三维轨迹点（含 climb/descent）及局部 XY 偏移生成，未使用旧端点单线候选。
- 代表路线结果：4 个 relay-required blocks，0 个 block 找到正 joint margin；因此 communication 和 seamless handoff 均 FAIL，route_feasible=False。
- 这只是该代表路线的语义闭环结果，不是对全部双点路线的不可行证明。
