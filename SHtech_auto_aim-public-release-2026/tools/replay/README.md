# tools/replay —— M2 录制/回放工具（模块 B，任务 B-2）

离线回放 M2 几何链路（角点精修 + PnP），使模块 A/B/C 可以各自离线开发。
与在线版本**共用同一份算法源码**（`detect/armor_corner_optimizer.cpp`、
`detect/corner_refine_submodule.hpp` 的精修策略、`mathutils/CoordTransformer.cpp`），
不复制粘贴算法代码。

## 产物

| 可执行文件 | 功能 |
|------------|------|
| `avi_to_r1` | `test.avi` 等视频 → R1（A-4 完成前验证格式用；bboxes 为空） |
| `replay_tool` | R1 → R2 + M2 指标 |

## 构建

```bash
cmake -S tools/replay -B build-replay
cmake --build build-replay -j
```

依赖：OpenCV、Eigen3（与主工程相同）。不链接 OpenMP，保证单线程确定性。

## 用法

### 1. 视频生成 R1（A-4 的实机 R1 到位前验证用）

```bash
./build-replay/avi_to_r1 test.avi /path/to/r1 --enemy-color 0   # 0 红 1 蓝 2 灰
```

输出：`<r1>/records.jsonl` + `<r1>/images/%08d.png`。姿态填 0、bboxes 为空。

### 2. R1 → R2

```bash
./build-replay/replay_tool /path/to/r1 /path/to/r2 asset/camParam/<相机参数>.yml [--no-refine]
```

输出：

- `<r2>/records.jsonl`：每帧一条记录（格式见 `docs/module-contracts.md` R2 节）
- `<r2>/metrics.json`：确定性统计（帧数、bbox 数、精修成功/失败数、PnP 成功/失败数）
- `<r2>/timing.json`：耗时 P50/P99（**非确定性**，仅供性能参考，不参与逐字节一致验收）

### 3. 确定性验证（B-2 验收①）

同一 R1 跑两次，`records.jsonl` 与 `metrics.json` 应逐字节一致：

```bash
./build-replay/replay_tool r1 r2_a cam.yml
./build-replay/replay_tool r1 r2_b cam.yml
diff -r --exclude=timing.json r2_a r2_b   # 无输出即通过
```

## 与在线流水线的一致性（B-2 验收②）

- 角点精修：调用 `detect::optimizeCornersWithEnemyColor`，与在线
  `CornerRefineSubModule::process` 是**同一个函数**（含颜色未知时红蓝双阈值策略，B-5）。
- PnP：调用 `CoordTransformer::pnp_get_measurement`，`attitude_yaw` 与 `R_world2imu`
  的取法与在线 `MultiPolicyPredictorSubModule::predict` 相同
  （`yaw/180*M_PI`、`Attitude::R_world2imu()`）。
- 姿态输入：R1 记录 yaw/pitch/roll（度），回放用 `Attitude` 构造函数重建，与在线一致。
- 帧序处理：单线程顺序处理，与 stage2 的串行执行一致。

## 已知限制

- `avi_to_r1` 生成的 R1 不含 bboxes（没有检测器），只能验证工具链与格式；
  实机 R1 由 A-4 的录制开关写入（格式见接口文档）。
- 图像为 PNG 无损存储；如 A 的录制开关写 JPEG，回放与在线的一致性（验收②）
  需要 A 改用无损格式或重新协商格式（B-1 定稿）。

## pnp_check —— 单图距离验证（B-4 验收③）

```bash
./build-replay/pnp_check 照片.png 相机参数.yml --tag 3 --true-distance 2.0 [--points pts.txt]
```

不传 `--points` 时打开窗口用鼠标点选四个角点（顺序：左上、左下、右下、右上；
r 重选，q 退出）；`--points` 文件为 4 行 "x y"（像素坐标，无显示环境用）。
`--tag` 为大/小装甲判定（3=小板，0/1/8=大板）。距离从云台旋转点量起（米），
输出水平距离误差，< 2% 达标。用与在线完全相同的 `CoordTransformer::pnp_get_measurement`。

配套的内参标定工具见 `tools/intrinsics_calibration.py`（B-4 第一步），
产出 yml 后可直接喂给 `handeye_calibration.py` 和本工具。
