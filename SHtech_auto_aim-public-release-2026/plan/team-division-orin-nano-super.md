# Auto-Aim 二次开发：Orin Nano Super 三模块分工总览

> 范围：仅视觉算法链路。不含 MCU 固件、串口协议、云台控制器、发射机构（电控侧）。`timedserial/` 只读，不改协议字段。
> 版本：v3（2026-10-18）。每人的任务和验收标准拆到三个模块文档里，不排时间。
> 协作规则以 [docs/collaboration-spec.md](../docs/collaboration-spec.md) 为准，接口以 [docs/module-contracts.md](../docs/module-contracts.md) 为准。冲突时以这两份文档为准。

## 模块文档

| 模块 | 负责人 | 文档 | 一句话 |
|------|--------|------|--------|
| M1 感知前端 | A | [module-A-perception.md](module-A-perception.md) | 图像 → 检测框：TensorRT 10 迁移、GPU 预处理、FP16/INT8、模型与数据集、R1 录制 |
| M2 几何精度 | B | [module-B-geometry.md](module-B-geometry.md) | 检测框 → 3D 测量：接口文档、回放工具、相机标定、角点精修、PnP 稳定性 |
| M3 决策 | C | [module-C-decision.md](module-C-decision.md) | 3D 测量 → 云台指令：两处单位/类型 bug、离线评测、延迟补偿、跟踪器、开火判断 |

每份模块文档包含：模块边界（能改哪些文件、不能改哪些）、现状问题（带文件和行号）、任务清单（每项写明要改的内容、验收标准、前置依赖）、模块指标、合入前自查清单。

## 1. 三模块划分

```text
 相机图像 ──► [M1 感知前端 · A] ──bbox_t──► [M2 几何精度 · B] ──装甲板测量──► [M3 决策 · C] ──► 云台指令
                                  接口 I1                        接口 I2
```

**隔离规则**：

1. 三个模块改的文件不重叠，所有权写在根目录 `CODEOWNERS`，由平台强制执行 Owner 批准。
2. 模块之间只通过 `common/datatype.hpp`（v1）和接口 I1、I2 交换数据。每个 `ThreadDataPack` 字段只有一个写入方。
3. 模块内部怎么改由 Owner 决定，只要不改变接口、不超出耗时预算即可。
4. 开发时使用上游模块的录制数据（R1、R2）或仿真数据，不等上游模块完成。

**共享冻结区（修改需三人全部批准，规范 §1.3）**：

| 文件 | 原因 |
|------|------|
| `common/`（尤其 `datatype.hpp`、`pipeline.hpp`） | 三个模块都通过它传数据；线程和流水线骨架 |
| `main.cpp`、`main.hpp`、根 `CMakeLists.txt` | 影响所有模块的线程编排和构建 |
| `mathutils/math_tools.hpp` | B 和 C 都在用 |
| `launch.cfg` | 字段名和含义是对外配置 |
| `docs/module-contracts.md` | 接口 I1、I2、录制格式、耗时预算 |
| `timedserial/` | 电控接口，算法组只读 |

## 2. 无法完全隔离的耦合点

| 耦合 | 为什么隔离不了 | 处理方式 |
|------|----------------|----------|
| 共用数据结构 | 流水线用同一个 `ThreadDataPack` 在阶段间传数据 | `datatype.hpp` v1 已写明每个字段的写入方和单位；三人批准后冻结，之后按 `BREAKING CHANGE` 流程修改 |
| PnP 调用 | PnP 由 C 的代码调用（`MultiPolicyPredictor_submodule.cpp:130,206,242,396`），实现在 B 的代码里 | 冻结 `pnp_get_measurement`、`pc_to_pw`、`pw_to_pc`、`pw_to_pi`、`pc_to_pu` 的签名和语义；B 只改实现，C 只调用 |
| 延迟 | Planner 已在线测量"取图完成 → Planner"的延迟，但没有包含曝光到取图完成这一段 | A 测出这段延迟（A-8）写进接口文档；C 把它加入延迟补偿（C-4） |
| 算力 | 三个模块共用一块 Orin，B 的角点精修和 C 的预测规划还共用 Stage2 线程 | A-5 测出基线后，三人一起确定各模块的单帧耗时预算，超出预算的 Owner 负责优化 |
| 标签表 | 模型类别由 A 决定，大小装甲判定在 B 的 PnP 里 | A-10 整理对照表，B-7 按表修改 |
| 命中率 | 命中率是整条链路的结果 | 命中率只作为团队指标；个人用各自的模块指标 |

## 3. 跨模块依赖

只列出一个模块的任务依赖另一个模块产出的情况。模块内部的依赖见各自文档。

| 需要方 | 任务 | 依赖 | 提供方 | 没有依赖时怎么办 |
|--------|------|------|--------|------------------|
| A | A-4 R1 录制 | R1 格式定稿 | B-1 | 无，需要先定格式 |
| B | B-2 回放工具 | 真实 R1 数据 | A-4 | 用 `test.avi` 生成的 R1 先开发 |
| B | B-7 大小装甲判定 | 标签对照表 | A-10 | 保持现有判定 |
| C | C-2 离线评测 | R2 数据 | B-2 | 用 `predictor_simulation.py` 的仿真数据 |
| C | C-4 延迟补偿 | 曝光延迟数值 | A-8 | 保持现有参数 |
| C | C-7 耗时评估 | 统计脚本 | A-5 | 用 C-2 评测程序里的计时 |
| 全部 | 所有在 Orin 上的验收 | Orin 能编译运行 | A-2 | 在 WSL/x86 上离线开发，Orin 上补验收 |

## 4. 协作流程

具体条款见规范对应章节，本节只列出本项目的落地动作。

| 事项 | 本项目落地 | 规范条款 |
|------|------------|----------|
| 仓库初始化 | 当前目录不是 git 仓库。由 A 执行 `git init`，把原始公开版本提交并打 tag `v0.0.0-baseline`，推送到队内 GitLab/GitHub，把 `main` 设为保护分支，创建 `dev` | §2.1、§2.5 |
| CODEOWNERS | 把 `@member-a/b/c` 替换为真实用户名，并在平台上开启"需要 Code Owner 批准" | §1 |
| 分支 | 每个任务一个分支，名称带任务编号，例如 `feat/a-2-trt10`、`feat/b-2-replay`、`fix/c-1-armor-jump-interval` | §2.2 |
| 提交 | `<type>(<scope>): <subject>`，scope 用目录名，例如 `perf(detect): 预处理迁移到 CUDA` | §3 |
| 格式 | 只格式化改动的行：`git clang-format dev` | §4.1 |
| 评审 | 至少 1 人评审；涉及冻结区需要三人批准 | §5 |
| 合入 `dev` | 1 人批准 + Orin 编译通过 + `test.avi` 离线回放通过 + 模块指标对比 + 耗时不超预算 | §5.3 |
| 合入 `main` | 3 人批准 + 实机记录 + SemVer tag | §5.3、§7 |
| 模型与参数 | 每个模型一份 `asset/models/<名>.md` 溯源记录；参数文件只新增，不覆盖 | §6.3、§6.4 |
| 实验记录 | `docs/experiments/YYYYMMDD-<角色>-<主题>.md`，结论为负的实验也要记录 | §6.6 |
| 例会 | 每周一次，30 分钟以内，纪要写在 `plan/weekly/YYYY-WW.md` | §8 |
| 赛前冻结 | 比赛前一周只接受 `fix:` 和参数调整，且需要 3 人批准 | §7.3 |

## 5. 指标总览

| 层级 | 指标 | 负责人 |
|------|------|--------|
| M1 | 端到端 FPS、Sensor/Preprocess/Detect P50/P99、曝光到 bbox 延迟、检测 mAP@0.5、漏检率 | A |
| M2 | 精修角点像素误差、PnP 距离/朝向角误差、朝向角帧间抖动、CornerRefine P50/P99 | B |
| M3 | 前视预测误差、目标切换次数、无效开火率、Predictor+Planner P50/P99 | C |
| 团队 | 命中率（静止/平移/旋转靶，固定距离档，每档 ≥ 50 发） | C 主持，三人共同 |

FPS、耗时预算和命中率的目标值，在 A-5 得到 Orin 上的实测基线后由三人一起确定，写进 `docs/module-contracts.md`。拿到基线之前不承诺具体数字。各模块文档中标为"建议阈值"的数值，三人可以一起调整。
