# 模块 M3 决策：负责人 C

> 目标：提升跟踪稳定性、预测精度和开火判断质量，最终提高命中率。
> 总览与公共规则见 [team-division-orin-nano-super.md](team-division-orin-nano-super.md)，协作规范见 [docs/collaboration-spec.md](../docs/collaboration-spec.md)，接口见 [docs/module-contracts.md](../docs/module-contracts.md)。
> 本文只列要改什么和怎样算完成，不排时间。任务编号表示建议顺序，"前置"一列写明依赖。

## 1. 模块边界

| 项目 | 内容 |
|------|------|
| 输入 | `ThreadDataPack` 中的 `bboxes`、`time`、`attitude`、`robotstatus`；接口 I2（调用 B 的 `pnp_get_measurement` 和坐标变换函数） |
| 输出 | `ThreadDataPack` 中的 `target`、`has_fixed_target`、`robotcommand`（经消息桥交给 TimedSerial） |
| 可以修改 | `predict/`、`planner/`（`planner/tinympc/` 是第三方代码，改动需在 MR 中说明）、`mathutils/` 中的滤波器（`Kalman.hpp`、`ExtendedKalman.hpp`、`IESEKF.hpp`、`IESEKF_Double_Armor.hpp`）、`asset/plannerParam/`、`tools/predictor_simulation.py` |
| 不能修改 | `common/`、`main.cpp`、根 `CMakeLists.txt`、`launch.cfg`、`mathutils/math_tools.hpp`（需要三人批准）；`CoordTransformer.*` 及 A、B 的其他文件；`timedserial/`（电控，只读，`RobotCommand` 字段顺序不能变） |
| 对外交付 | 命中率测试记录；比赛用的 planner 参数文件 |

## 2. 现状问题（源码静态检查，未在 Orin 上实测）

| # | 问题 | 位置 |
|---|------|------|
| C-F1 | `armor_jump_interval` 声明为 `const int`，初值 `0.1f` 被截断为 0；`dt_since_jump > armor_jump_interval` 永远成立，装甲板切换后的禁射窗口不起作用 | `planner/Planner.hpp:80`、`planner/Planner.cpp:261-262` |
| C-F2 | `generate_command_array` 用 `yaw_angle + i * yaw_speed * dt` 外推 2 ms 一步的指令，`yaw_angle` 是度、`yaw_speed` 是 rad/s，外推量只有应有值的约 1/57；pitch 同理 | `planner/planner_submodule.cpp:24,27` |
| C-F3 | 延迟补偿 `total_delay = process_latency + comm_latency + fly_time`。`process_latency` 在线测量"取图完成 → Planner"，`comm_latency` 来自参数文件（哨兵配置为 5 ms，注释里的旧值是 35 ms），**没有包含曝光到取图完成的延迟** | `planner/Planner.cpp:164-166`、`asset/plannerParam/planner_param_sentry.yml:6` |
| C-F4 | 弹速来自 `robotstatus.robot_speed_mps`，Uart 路径固定为 Mock 值 24.5 m/s。实时弹速接入属于电控协议，不在本模块范围 | `timedserial/UartIMU/uart_driver.cpp:79` |
| C-F5 | MPC 时域 `HORIZON = 100`、`DT = 0.01`（即 1 s），每个求解器 `max_iter = 10`；yaw、pitch 两个求解器在 Stage2 每帧串行运行，与 B 的角点精修共用一个线程 | `planner/Planner.hpp:44-50`、`Planner.cpp:640,669` |
| C-F6 | `predict/`、`planner/` 中至少有 24 处未注释的 `std::cout`，其中一部分在每帧路径上（如 PnP 失败时），违反规范 §4.3.5 | 例：`predict/MultiPolicyPredictor_submodule.cpp:211` |
| C-F7 | 前哨站半径钳位范围 0.12–0.4 m 等经验常数直接写在代码里 | `predict/Tracker.cpp:987-997` |
| C-F8 | 开火判断只比较参考轨迹与 MPC 轨迹的偏差（`same_trace_threshold`），没有使用滤波器的不确定度 | `planner/Planner.cpp:274-275` |

## 3. 任务清单

### C-1 修复禁射窗口（C-F1）

| 项目 | 内容 |
|------|------|
| 要改的 | 把 `armor_jump_interval` 改为 `double`，单位秒；取值移到 planner 参数文件，并写明取值依据（例如装甲板切换后云台稳定所需时间的实测值）。单独一个 MR：`fix(planner): ...` |
| 验收标准 | ① 编译开启 `-Wconversion` 时这一处不再报警；② 在旋转靶录像或仿真上，统计装甲板切换后窗口时间内的开火次数为 0；③ MR 附修复前后的开火时序对比图 |
| 前置 | 无 |

### C-2 M3 离线评测

| 项目 | 内容 |
|------|------|
| 要改的 | 写评测程序：读取 R2 或 `predictor_simulation.py` 生成的仿真数据，运行与在线版本相同的 `Tracker` 和 `Planner` 代码（不复制算法代码），输出预测前视误差（例如 100、200、300 ms 前视）、目标切换次数、Predictor + Planner 耗时 |
| 验收标准 | ① 同一输入跑两次，结果完全一致；② 仿真数据有真值时，能输出各前视时间的位置误差和朝向误差；③ 基线报告入库（`docs/experiments/`） |
| 前置 | 仿真数据：无；R2 数据：B-2 |

### C-3 修复指令外推单位（C-F2）

| 项目 | 内容 |
|------|------|
| 要改的 | 在 `generate_command_array` 中把角速度换算成度/秒后再外推，单位换算写成具名常量或辅助函数；pitch 同样处理。单独一个 MR：`fix(planner): ...` |
| 验收标准 | ① 单元测试：给定 `yaw_angle = 10°`、`yaw_speed = 1 rad/s`，第 i 条指令的角度等于 `10 + i × 0.002 × 57.2958`（误差 < 1e-4）；② 实机或下位机日志中，指令序列的角度变化与角速度一致；③ MR 中说明对下位机行为的影响，由电控同学确认 |
| 前置 | 无 |

### C-4 延迟补偿重新标定（C-F3）

| 项目 | 内容 |
|------|------|
| 要改的 | 在 `total_delay` 中加入 M1 的"曝光 → 取图完成"延迟（读 planner 参数文件中的新字段，数值来自 A-8）；重新测量 `comm_latency`（指令发出到云台响应），更新参数文件，注释写明测量方法和日期 |
| 验收标准 | ① 参数文件里每个延迟都有来源说明；② 平移靶（已知速度）实测：弹着点或瞄准点的横向滞后误差小于修改前；③ 在 C-2 评测中，加入延迟偏移后预测误差不变差 |
| 前置 | A-8 |

### C-5 跟踪器调优

| 项目 | 内容 |
|------|------|
| 要改的 | 调整 `Tracker` 的整车模型和装甲板模型噪声参数、模型切换阈值、目标选择策略；经验常数（C-F7）移到参数文件。使用 C-2 的评测和 `predictor_simulation.py` 仿真对照 |
| 验收标准 | ① 在 C-2 的平移靶和旋转靶数据上，200 ms 前视位置误差低于基线；② 目标切换次数不增加；③ 整车模型半径估计在旋转靶上收敛到真值 ± 2 cm 以内（建议阈值） |
| 前置 | C-2 |

### C-6 开火判断

| 项目 | 内容 |
|------|------|
| 要改的 | 在现有轨迹一致性判断（C-F8）之外，加入滤波器协方差（预测不确定度）和装甲板朝向角门限，只有预测足够可信、装甲板朝向足够正时才开火 |
| 验收标准 | ① 实机测试：在静止、平移、旋转靶的固定距离档位，每档 ≥ 50 发，命中率不低于修改前；② 无效开火率（开火但未命中的比例）低于修改前；③ 门限值写在参数文件里 |
| 前置 | C-1、C-5 |

### C-7 求解器耗时与热路径清理

| 项目 | 内容 |
|------|------|
| 要改的 | 评估 `HORIZON`、`DT`、`max_iter` 对耗时和跟踪误差的影响（C-F5）；每帧路径上的 `std::cout` 改为受 `launch.cfg` 日志开关控制的日志（C-F6） |
| 验收标准 | ① 按 A-5 的方法测量，Predictor + Planner 的 P99 在耗时预算以内；② C-2 评测中跟踪误差不变差；③ 每帧路径上不再有无条件的 `std::cout` |
| 前置 | C-2、A-5 |

### C-8 弹速参数化

| 项目 | 内容 |
|------|------|
| 要改的 | 在 planner 参数文件中增加可选的弹速覆盖值；有覆盖值时使用覆盖值，没有时使用 `robotstatus.robot_speed_mps`。不修改 `timedserial/` 和串口协议 |
| 验收标准 | ① 不配置覆盖值时行为与修改前完全一致；② 配置覆盖值后，弹道计算使用该值（日志可见）；③ 参数文件注释写明该值的测量来源 |
| 前置 | 无 |

## 4. 模块指标

| 指标 | 测量方式 |
|------|----------|
| 100/200/300 ms 前视位置误差、朝向误差 | C-2 评测（仿真 + R2） |
| 目标切换次数 | C-2 评测 |
| Predictor + Planner P50/P99 耗时 | A-5 的统计脚本 |
| 无效开火率 | 实机测试 |
| 命中率（团队指标，C 主持） | 静止/平移/旋转靶，固定距离档，每档 ≥ 50 发 |

## 5. 合入前自查

- [ ] 只改了第 1 节"可以修改"范围内的文件
- [ ] 没有修改 `RobotCommand` 字段顺序和单位
- [ ] 只通过 I2 调用 PnP 和坐标变换，没有修改 `CoordTransformer`
- [ ] Orin 上编译通过，`test.avi` + `port=None` 跑完无崩溃
- [ ] MR 描述里有 C-2 评测或实机测试的指标对比表
- [ ] 参数文件是新建的，或改动处写明了来源，没有直接覆盖比赛版本
