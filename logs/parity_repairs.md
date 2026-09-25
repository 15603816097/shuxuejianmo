# Parity 修复记录

初始校准发现 11 项 DIFFERENT、2 项 MISSING。

- 硬截止：权威口径取 `医疗物资期望送达时间` 与 `是否首批保障` 的截止时间；两套 evaluator 统一为 `completion_time > hard_deadline`。
- 资源释放：统一半开区间 `[start,end)`；运输为 `[start,return)`，电池含充电尾段，中继含准备前移与返航后周转，服务为 `[arrival,completion)`。
- 服务资源：统一以到达后交接开始、交接完成释放，逐服务区容量 1。
- 中继能耗：统一使用认证路径总距离、巡航功率和速度计算，逐任务写入 `relay_energy`。

Golden Schedule 固定为 `results/golden_grid60_schedule.csv`，A/B evaluator 均不修改任务时序。
