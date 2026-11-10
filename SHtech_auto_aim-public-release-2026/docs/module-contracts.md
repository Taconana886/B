# 模块接口约定

> 状态：**B-1 初稿完成（2026-10-30），征求意见中**。A、C 48 小时无异议视为通过（单人执行规则，见 `plan/module-B-geometry-scheduled.md` B-1）。冻结后的修改按 `docs/collaboration-spec.md` §1.3 和 §3.3（`BREAKING CHANGE`）执行。
> 下文"当前代码"部分摘自源码，I2、R1、R2 的单位、坐标系、取值范围由 B 根据 `CoordTransformer.cpp` 实现补全并核对。

## 共享数据结构 `common/datatype.hpp`（v1，2026-10-18 定稿，待三人批准后冻结）

版本号：`kDataTypeVersion = 1`。R1/R2 录制文件必须记录这个版本号。

### ThreadDataPack 字段写入权

每个字段只有一个写入方，其他模块只读。

| 字段 | 类型 | 单位/格式 | 写入方 | 读取方 |
|------|------|-----------|--------|--------|
| `index` | `int` | 帧序号 | EntryStage | 全部 |
| `submodule_results` | `array<SubModuleResult>` | — | pipeline 框架 | pipeline 框架 |
| `submodule_timestamps` | `array<pair<steady_clock::time_point>>` | — | pipeline 框架 | EntryStage |
| `frame` | `cv::Mat` | BGR 原图 | M1 Sensor | M1、M2 |
| `time` | `FrameClock::time_point` | 取图完成时刻，单调时钟 | M1 Sensor | M3 |
| `attitude` | `Attitude` | yaw/pitch/roll 为**度**，旋转顺序 Z-X-Y | M1 Sensor | M2、M3 |
| `robotstatus` | `RobotStatus` | 弹速 m/s | M1 Sensor | M2、M3 |
| `detect_input` | `cv::Mat` | RGB 网络输入 | M1 Preprocess | M1 Detect |
| `detect_input_map` | `DetectInputMap` | 像素 | M1 Preprocess | M1 Detect |
| `bboxes` | `vector<bbox_t>` | 见 I1 | M1 Detect；M2 只改 `pts`、`source` | M2、M3 |
| `target` | `Target` | m、m/s、rad | M3 Predictor | M3 Planner |
| `has_fixed_target` | `bool` | — | M3 Predictor | M3 Planner |
| `robotcommand` | `RobotCommand` | 角度为度（相对当前姿态），角速度 rad/s | M3 Planner | TimedSerial |

### v1 相对原代码的修改

| 修改 | 原因 | 影响文件 |
|------|------|----------|
| 新增 `FrameClock = std::chrono::steady_clock`，`time` 改用它 | libstdc++ 的 `high_resolution_clock` 就是 `system_clock`，Orin 联网做 NTP 校时时会跳变，延迟补偿和禁射计时会出错 | `sensor/sensor_submodule.cpp`、`predict/Tracker.*`、`predict/MultiPolicyPredictor_submodule.hpp`、`planner/Planner.*`（只替换了类型名） |
| 删除 `ThreadDataPack::target_state` | 全仓库没有任何读写 | — |
| `bbox_t`、`RobotStatus::enemy`、`Target` 的成员加默认值 | 原来未初始化，读到的是随机值 | — |
| `Target::tracked_state` 注释改为 11 维 | 原注释写 9 维，与实际的 `Matrix<double, 11, 1>` 和 `Tracker.cpp` 用法不一致 | — |
| 每个字段补充写入方、读取方、单位 | 冻结接口需要明确的写入权 | — |
| 新增 `kDataTypeVersion` | 录制文件需要记录数据结构版本 | — |

未改动：所有枚举值、`RobotCommand` 字段顺序（串口协议相关）、`Attitude` 计算方法、`Plan`。

验证范围：在 WSL Ubuntu 22.04（g++ 11.4、OpenCV 4.5.4）上，对 entrystage、sensor、detect（除 TRT/AXCL 后端外）、predict、planner、mathutils、common 共 14 个源文件做了 `-fsyntax-only` 检查，全部通过。没有做完整构建，也没有在 Orin 上运行。

## I1：M1 → M2（A 提供）

**定义**：`common/datatype.hpp` 中的 `bbox_t`，通过 `ThreadDataPack::bboxes` 传递，帧时间戳为 `ThreadDataPack::time`。

| 字段 | 类型 | 约定 | 备注 |
|------|------|------|------|
| `pts[4]` | `cv::Point2f` | 原图像素坐标；顺序为 [0]左上、[1]左下、[2]右下、[3]右上，与 PnP 物体点一致 | — |
| `confidence` | `float` | 检测置信度，默认 0 | M1 内部使用，过滤阈值由 A 在 M1 内部决定；M2 不读 |
| `color_id` | `int` | 0 红、1 蓝、2 灰，默认 2 | — |
| `tag_id` | `int` | 0 无目标、1–7 机器人、8 前哨站、9 基地，默认 0；大装甲板判定见 I2 节 | A-10 确认标签对照表后如有出入，按 §3.3 BREAKING CHANGE 更新 |
| `source` | `DetectionSource` | M1 写 `NEURAL_NETWORK`；M2 精修成功时改为 `TRADITIONAL` | — |
| `time` | `FrameClock::time_point` | 取图完成时刻（不是曝光时刻），单调时钟 | 是否改为曝光时刻由 A 评估海康 SDK 帧时间戳后另行提案（§1.3） |

**M1 延迟（A 发布，C 读取）**：

| 日期 | 版本/tag | 条件 | 曝光到 bbox 输出 P50 | P99 |
|------|----------|------|----------------------|-----|
| 待测 | | | | |

## I2：M2 → M3（B 提供）

**当前代码**：`mathutils/CoordTransformer.hpp`，单例 `CoordTransformer::Get()`。C 只调用，不修改。

**坐标系约定（I2 通用）**：世界系原点在云台旋转点，x 向右、y 向前、z 向上（右手系）。长度单位一律为米（m）。角度单位在每处单独注明。yaw 正方向：绕世界系 z 轴负方向旋转为正（等价于沿 +z 向下看顺时针为正；"正对装甲板为 0"）。

| 函数 | 当前语义 | 约定（B-1 补全） |
|------|----------|------------------|
| `bool pnp_get_measurement(const cv::Point2f (&p)[4], const int& armor_number, const int& color_id, const float& attitude_yaw, const Eigen::Matrix3d& R_world2imu, float& yaw_in_camera, Eigen::Vector4d& measurement)` | 输出 `measurement = [y, x, z, absolute_yaw]`，`attitude_yaw` 为弧度 | 详见下方"pnp_get_measurement 详细约定" |
| `Pos3D pc_to_pw(const Pos3D& pc, const Eigen::Matrix3d& R_world2imu)` | 相机系 → 世界系 | 输入输出均为 m；变换链 Camera → IMU → World |
| `Pos3D pw_to_pc(const Pos3D& pw, const Eigen::Matrix3d& R_world2imu)` | 世界系 → 相机系 | 输入输出均为 m；变换链 World → IMU → Camera |
| `Pos3D pw_to_pi(const Pos3D& pw, const Eigen::Matrix3d& R_world2imu)` | 世界系 → IMU 系（只有旋转） | 输入输出均为 m，只做旋转不含平移 |
| `Pos3D pc_to_pu(const Pos3D& pc)` | 相机系 → 像素齐次坐标 `[u, v, 1]` | 输出为齐次像素坐标（像素），需除以第三分量得到真实像素坐标；pc 单位 m |

### pnp_get_measurement 详细约定

- **输入**：`p` 为四个角点原图像素坐标（左上、左下、右下、右上）；`armor_number` 即 `bbox_t::tag_id`；`color_id` 为颜色编号（I1 约定）；`attitude_yaw` 为机器人当前姿态 yaw，单位**弧度**；`R_world2imu` 为世界系到 IMU 系的旋转矩阵（由 `Attitude` 提供，Z-X-Y 顺序）。
- **输出 `measurement`（顺序 `[y, x, z, absolute_yaw]`，注意不是 x,y,z 顺序）**：
  - `measurement(0)` = 装甲板中心在世界系的 **y** 坐标，单位 m；
  - `measurement(1)` = 装甲板中心在世界系的 **x** 坐标，单位 m；
  - `measurement(2)` = 装甲板中心在世界系的 **z** 坐标，单位 m；
  - `measurement(3)` = `absolute_yaw` = `limit_rad(yaw_in_camera - attitude_yaw)`，单位**弧度**，取值范围 `[-pi, pi]`（`limit_rad` 归一化到该区间）。
- **输出 `yaw_in_camera`**：装甲板法向量在相机系的朝向角，单位**弧度**，取值范围 `[-pi, pi]`。正对装甲板（法向量指向相机光轴反方向）时为 0；绕世界系 z 轴负方向旋转为正（绕 z 轴正方向旋转为负），与 `CoordTransformer.cpp` 实现和 `Target::tracked_state` 注释一致。
- **失败条件**：`cv::solvePnP` 返回 false 时函数返回 `false`，此时 `yaw_in_camera` 与 `measurement` 保持调用前的值、**内容未定义**，调用方必须检查返回值（C 的现有代码已全部检查）。
- **装甲板模型**（PnP 物体点，硬编码于 `CoordTransformer.hpp`）：小装甲板 13.5 cm × 5.6 cm，大装甲板 22.8 cm × 5.6 cm（官方规格；B-7 核对来源后补注释）。

大、小装甲判定：当前 `armor_number`（即 `tag_id`）为 `0`、`1`、`8` 时按大装甲板处理，其余按小装甲板处理；判定集合在 `CoordTransformer.hpp` 中集中定义为 `kLargeArmorTagIds`。最终对照表待 A-10 标签表交付后由 B-7 对齐；对齐后如有出入按 §3.3 流程更新本文档。

## 录制格式（B 起草，2026-10-30 初稿）

两种格式均为 **JSONL**：每帧一条 JSON 记录（`\n` 结尾），图像单独存文件。文件名用 `%08d` 格式化的帧序号。R1/R2 均记录 `kDataTypeVersion`（当前 = 1）。

### R1（M1 录制，A 写入；B 的回放工具读取）

目录结构：

```text
<r1_dir>/
  records.jsonl      # 每帧一条记录，顺序与帧序号一致
  images/
    00000000.png     # 帧序号 %08d.png，BGR，无损
```

`records.jsonl` 每行字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `format_version` | int | R1 格式版本，= 1 |
| `data_type_version` | int | `kDataTypeVersion`，= 1 |
| `index` | int | 帧序号（`ThreadDataPack::index`） |
| `time_ns` | uint64 | `FrameClock::time_point` 自单调时钟纪元起的纳秒数 |
| `attitude_deg` | {yaw, pitch, roll} | 取图时刻云台姿态，单位**度**，旋转顺序 Z-X-Y |
| `enemy_color` | int | 0 红、1 蓝、2 灰（`RobotStatus::enemy_color`） |
| `image` | string | 图像文件名，相对 R1 目录（如 `images/00000000.png`） |
| `image_width` / `image_height` | int | 图像尺寸（像素），用于自检 |
| `bboxes` | array | `bbox_t` 列表，元素见下 |

`bboxes` 元素：`{"pts": [[x0,y0],[x1,y1],[x2,y2],[x3,y3]], "confidence": float, "color_id": int, "tag_id": int, "source": int}`。`pts` 为原图像素坐标，顺序同 I1；`source` 取 `DetectionSource` 枚举值。

图像采用 PNG 无损存储：M2 的角点精修与 PnP 要求回放输入与在线原图逐字节一致（B-2 验收②）。

### R2（B 的回放工具输出，C 读取）

目录结构：

```text
<r2_dir>/
  records.jsonl      # 每帧一条记录，顺序与帧序号一致（确定性输出）
  metrics.json       # M2 确定性统计（计数/成功率）
  timing.json        # 耗时统计 P50/P99（依赖时钟，非确定性，仅供性能参考，不参与逐字节一致验收）
```

`records.jsonl` 每行字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `format_version` | int | R2 格式版本，= 1 |
| `data_type_version` | int | `kDataTypeVersion`，= 1 |
| `index` | int | 帧序号，与 R1 一致 |
| `time_ns` | uint64 | 与 R1 一致 |
| `attitude_deg` | {yaw, pitch, roll} | 与 R1 一致 |
| `enemy_color` | int | 与 R1 一致 |
| `R_world2imu` | float[9] | 世界系 → IMU 系旋转矩阵（行主序），由 `Attitude` 计算 |
| `measurements` | array | 每块装甲板一条，元素见下 |

`measurements` 元素：

| 字段 | 类型 | 说明 |
|------|------|------|
| `tag_id` | int | 与 I1 一致 |
| `color_id` | int | 与 I1 一致 |
| `source` | int | 角点来源（精修成功 = 1 TRADITIONAL，否则 = 0 NEURAL_NETWORK） |
| `pts` | [[x,y]×4] | 精修后的角点（M2 输出） |
| `pnp_success` | bool | `pnp_get_measurement` 是否成功 |
| `measurement` | [y, x, z, absolute_yaw] | 见 I2；pnp_success=false 时四个值为 `null` |
| `yaw_in_camera` | float | 单位 rad，范围 [-pi, pi]；pnp_success=false 时为 `null` |

`metrics.json`（确定性统计，参与逐字节一致验收）：

| 字段 | 说明 |
|------|------|
| `frame_count` | 总帧数 |
| `bbox_count` | 总装甲板数 |
| `refine_success_count` / `refine_fail_count` | 角点精修成功/失败数（B-5 成功率统计口径：`optimizeCorners` 返回值的帧内板数） |
| `pnp_success_count` / `pnp_fail_count` | PnP 成功/失败数 |

`timing.json`（非确定性，仅供性能参考）：

| 字段 | 说明 |
|------|------|
| `refine_ms_p50` / `refine_ms_p99` | 单块装甲板精修耗时（ms） |
| `pnp_ms_p50` / `pnp_ms_p99` | 单块装甲板 PnP 耗时（ms） |

**确定性要求**：回放工具单线程执行，不引入 OpenMP/多线程非确定性；同一 R1 输入跑两次，R2 输出逐字节一致（B-2 验收①）。

录像数据不进 git，放在共享存储，这里只登记路径和 SHA-256：

| 数据集 | 版本 | 路径 | SHA-256 | 采集日期 |
|--------|------|------|---------|----------|
| （B-3 几何评测集） | 待采集 | 待定 | 待登记 | 待采集 |
| （A-4 实机 R1） | 待 A 交付 | 待定 | 待登记 | 待采集 |

## 单帧耗时预算（A-5 得到基线后由三人确定）

| 模块 | 线程 | P99 预算 | 基线实测 |
|------|------|----------|----------|
| M1 | Stage0 + Stage1 | 待定 | 待测 |
| M2 | Stage2（CornerRefine） | 待定 | 待测 |
| M3 | Stage2（Predictor + Planner） | 待定 | 待测 |
