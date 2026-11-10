# 模块 M2 几何精度：负责人 B

> 目标：让角点和 PnP 位姿更准、更稳，并提供录制/回放工具，使三个模块可以各自离线开发。
> 总览与公共规则见 [team-division-orin-nano-super.md](team-division-orin-nano-super.md)，协作规范见 [docs/collaboration-spec.md](../docs/collaboration-spec.md)，接口见 [docs/module-contracts.md](../docs/module-contracts.md)。
> 本文只列要改什么和怎样算完成，不排时间。任务编号表示建议顺序，"前置"一列写明依赖。

## 1. 模块边界

| 项目 | 内容 |
|------|------|
| 输入（接口 I1） | `ThreadDataPack` 中的 `frame`、`attitude`、`robotstatus.enemy_color`、`bboxes` |
| 输出 | 改写后的 `bboxes[i].pts` 和 `bboxes[i].source`；接口 I2：`CoordTransformer` 的 `pnp_get_measurement` 及坐标变换函数（由 C 调用） |
| 可以修改 | `detect/corner_refine_submodule.*`、`detect/armor_corner_optimizer.*`、`mathutils/CoordTransformer.*`（只改实现，不改 I2 签名和语义）、`asset/camParam/`、`tools/handeye_calibration.py`、`tools/replay/`（新建） |
| 不能修改 | `common/`、`main.cpp`、根 `CMakeLists.txt`、`launch.cfg`、`docs/module-contracts.md` 定稿后（需要三人批准）；A、C 的目录；`timedserial/`（电控，只读） |
| 对外交付 | R1/R2 录制格式与回放工具（给 A、C）；R2 数据（给 C）；新标定的相机参数文件 |

## 2. 现状问题（源码静态检查，未在 Orin 上实测）

| # | 问题 | 位置 |
|---|------|------|
| B-F1 | PnP 已经使用 `SOLVEPNP_IPPE`。平面四点 IPPE 有两个候选解，远距离时容易在两个解之间跳变，表现为装甲板朝向角抖动。当前代码直接采用 OpenCV 返回的解，没有做歧义处理 | `mathutils/CoordTransformer.cpp:177` |
| B-F2 | `measurement = [pw_y, pw_x, pw_z, limit_rad(yaw_in_camera - attitude_yaw)]`：前三项是世界系位置（m），第四项是弧度。顺序是 y、x、z，不是 x、y、z，头文件注释只写了 "[y, x, z, absolute_yaw]"，没有写单位和坐标系 | `CoordTransformer.cpp:232`、`CoordTransformer.hpp:183` |
| B-F3 | 大装甲板判定写死为 `armor_number` 为 0、1、8；`tag_id=0` 在 `datatype.hpp` 中表示无目标。需要和 A 的标签表核对 | `CoordTransformer.cpp:146,152` |
| B-F4 | 角点精修的二值化阈值按 `enemy_color` 选择；颜色未知时默认用蓝色阈值，只打一条 warning | `detect/corner_refine_submodule.cpp:35-40` |
| B-F5 | 角点精修失败时保留神经网络角点，`source` 设为 `NEURAL_NETWORK`；成功时设为 `TRADITIONAL`。精修前后的精度差异没有任何统计 | `corner_refine_submodule.cpp:84-94` |
| B-F6 | `tools/handeye_calibration.py` 里写死了一组相机内参和畸变系数，换相机后必须改脚本 | `tools/handeye_calibration.py:88-94` |
| B-F7 | 现有相机参数文件对应 AX650 平台的若干台车，Orin 上使用的相机和镜头需要重新标定（README："不能跨设备直接复用"） | `asset/camParam/` |

## 3. 任务清单

### B-1 接口文档补全

| 项目 | 内容 |
|------|------|
| 要改的 | 补全 `docs/module-contracts.md`：I2 中 `measurement`、`yaw_in_camera` 的单位、坐标系、取值范围和失败条件；R1、R2 的文件格式（建议每帧一条记录，图像单独存文件；记录 `kDataTypeVersion`）。同时给 `CoordTransformer.hpp` 的 I2 函数补齐 Doxygen 注释（单位、坐标系） |
| 验收标准 | ① 文档中 I1、I2、R1、R2 不再有"待定"项（A-8 的延迟数值除外）；② A、C 都批准；③ 头文件注释与文档一致 |
| 前置 | 无 |

### B-2 录制/回放工具 `tools/replay/`

| 项目 | 内容 |
|------|------|
| 要改的 | 新建 `tools/replay/`：读取 R1，调用与在线版本**相同**的 `ArmorCornerOptimizer` 和 `CoordTransformer` 代码（编译成独立可执行文件，不复制粘贴算法代码），输出 R2 和 M2 指标。提供一个由 `test.avi` 生成 R1 的入口，在 A-4 完成前也能使用 |
| 验收标准 | ① 同一份 R1 输入跑两次，R2 输出逐字节一致；② 对同一帧，回放输出的 `measurement` 与在线流水线的输出一致（最大差 < 1e-6）；③ C 能用 R2 跑通 C-2 的评测；④ 有 README 说明用法 |
| 前置 | B-1 |

### B-3 几何评测集

| 项目 | 内容 |
|------|------|
| 要改的 | 采集已知距离（如 1–8 m 多个档位）、已知朝向角（如 0°、±15°、±30°、±45°）的大、小装甲板图像，人工标注角点真值，记录真实距离和角度。数据放共享存储，接口文档登记版本和 SHA-256 |
| 验收标准 | ① 每个距离 × 角度组合 ≥ 30 帧；② 角点标注由两人交叉检查；③ 评测脚本能输出角点误差、距离误差、朝向角误差三项指标 |
| 前置 | B-4 完成后采集（需要准确的相机参数） |

### B-4 相机内参与相机-IMU 外参标定

| 项目 | 内容 |
|------|------|
| 要改的 | 对 Orin 上实际使用的相机和镜头重新标定内参；用 `handeye_calibration.py` 标定相机到 IMU 的外参。脚本改为从文件读取内参，不再写死（B-F6）。新建带设备编号和日期的 `cam_param_*.yml`，不覆盖旧文件 |
| 验收标准 | ① 内参标定的平均重投影误差 < 0.3 像素（建议阈值）；② 标定报告（图像数量、覆盖范围、误差）入库；③ 把已知距离的靶板放在画面中心和四角，PnP 距离误差 < 2%（建议阈值） |
| 前置 | 无 |

### B-5 角点精修

| 项目 | 内容 |
|------|------|
| 要改的 | 调整 `armor_corner_optimizer` 在运动模糊、远距离、灯条过曝、颜色未知时的处理；颜色未知时的阈值策略写清楚（B-F4）；统计精修成功率 |
| 验收标准 | ① 在 B-3 评测集上，精修后的角点像素误差中位数低于神经网络原始角点；② 精修后的 PnP 距离误差、朝向角误差不差于不精修；③ 在 R1 回放上统计精修成功率，并写进实验记录；④ CornerRefine 的 P99 耗时在预算以内 |
| 前置 | B-2、B-3 |

### B-6 PnP 精度与朝向角稳定性

| 项目 | 内容 |
|------|------|
| 要改的 | 在 `pnp_get_measurement` 内部处理 IPPE 双解歧义（例如用 `solvePnPGeneric` 取出两个解，结合上一帧朝向或装甲板倾角约束选择）；评估 `solvePnPRefineLM` 等精修步骤；评估远距离下的朝向角稳定性。只改实现，不改函数签名和输出含义 |
| 验收标准 | ① 在 B-3 评测集上，距离误差和朝向角误差不差于当前实现；② 静止靶 R1 录像中，朝向角的帧间标准差下降；③ 双解跳变（相邻帧朝向角差 > 20°）的次数下降；④ C 的 C-2 评测在新 R2 上不变差 |
| 前置 | B-2、B-3 |

### B-7 大小装甲判定对齐

| 项目 | 内容 |
|------|------|
| 要改的 | 按 A-10 给出的标签对照表，修改 `CoordTransformer.cpp` 中的大装甲板判定，判定集合改为可配置或集中定义的常量 |
| 验收标准 | ① 判定与接口文档中的对照表一致；② B-3 评测集中大装甲板样本的 PnP 距离误差没有系统性偏差 |
| 前置 | A-10 |

## 4. 模块指标

| 指标 | 测量方式 |
|------|----------|
| 精修后角点像素误差（中位数、P90） | B-3 评测集 |
| PnP 距离误差（%）、朝向角误差（°） | B-3 评测集 |
| 静止靶朝向角帧间标准差、双解跳变次数 | R1 回放 |
| 内参重投影误差 | B-4 标定报告 |
| CornerRefine P50/P99 耗时 | A-5 的统计脚本 |

## 5. 合入前自查

- [ ] 只改了第 1 节"可以修改"范围内的文件
- [ ] I2 函数签名和输出含义没有变化；有变化时走三人批准和 `BREAKING CHANGE` 流程
- [ ] 只改写了 `bboxes[i].pts` 和 `bboxes[i].source`，没有增删 `bboxes` 元素
- [ ] Orin 上编译通过，`test.avi` + `port=None` 跑完无崩溃
- [ ] MR 描述里有 B-3 评测集上的指标对比表
- [ ] 新的相机参数文件是新建的，没有覆盖旧文件
