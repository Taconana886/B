# 2026-10-30 B-2/B-4 工具链开发记录（含负结果）

> 负责人：B。任务：B-1 文档补全、B-2 回放工具、B-4 手眼标定脚本改造（B-F6/B-F8）、
> B-5 颜色未知策略、B-6 PnP 双解消歧、B-7 常量集中化。

## 结论

1. **B-2 回放工具**（`tools/replay/`）完成并自测：`test.avi → R1 → R2` 全链路跑通，
   同一 R1 跑两次 `records.jsonl`/`metrics.json` 逐字节一致（1688 帧 + 200 帧合成 bbox 数据）。
2. **B-4 手眼标定脚本**（`tools/handeye_calibration.py`）改造完成，并用合成棋盘格数据
   端到端验证：旋转误差 0.85°、平移误差 15.3 mm（含角点检测噪声；纯数学无噪声验证
   收敛到 2.5e-13 mm）。**发现并修正旧脚本的 `calibrateHandEye` 输入约定错误**（见下）。
3. B-5/B-6/B-7 代码改动完成，编译与确定性自测通过；精度指标需 B-3 评测集验证（待采集）。

## 关键发现（负结果/坑）

### 旧 handeye 脚本的 `calibrateHandEye` 约定错误

- 旧脚本把 euler 文件的旋转矩阵（语义为"世界系 → 云台系"）**未转置**直接作为
  `R_gripper2base` 传入。OpenCV 要求的是"云台系 → 世界系"（gripper → base），
  且返回的 `R_cam2gripper`/`t_cam2gripper` 是字面意义的"相机系 → 云台系"
  （与 yml 的 `R_c2i`/`T_c2i` 定义 `p_imu = R_c2i @ p_cam + T_c2i` 一致，直接写出）。
- 验证方法：合成 12 组姿态（euler → R_W2G → R_obs = R_G2C R_W2G、
  t_obs = R_G2C R_W2G t_board0 + T_G2C），分别测试正/反约定：
  - 未转置传入：R 误差 32.8°（坏解）
  - 转置传入（正确）：R 误差 0.0°、T 误差 2.5e-13 mm
- 旧脚本结果只 print 不写回 yml，所以历史结果大概率从未被直接使用；
  **重跑标定后必须用新约定**，旧 print 输出不要手工搬运。

### 合成数据生成中的两个自误（已修正，留给后人参考）

- 靶板固定在世界系，云台转动后靶板在云台系的位置必须随姿态旋转
  （`t_board_i = R_W2G_i @ t_board0`），不能把云台系坐标当常量。
- 棋盘格近正对相机（约 90° 姿态 + 小扰动）才容易被 `findChessboardCorners` 检测；
  强透视/斜视的合成棋盘格经常检测失败（真实照片无此问题）。

### 确定性输出的做法（B-2 验收①）

- 浮点序列化用 `std::to_chars`（最短往返、locale 无关），不用 `std::ostringstream <<`；
- 单线程、不链 OpenMP；耗时统计单独放 `timing.json`（非确定性），确定性文件只有
  `records.jsonl` + `metrics.json`。

### B-6 合成数据验证记录（新增 2026-10-30 下午）

用投影合成（无噪声 + 0.3 px 噪声，多个距离/偏航组合）验证 `pnp_get_measurement`：

1. **角点约定确认**：检测器输出（`detect/AXCL/AXCL.cpp:436`）为图像序
   TL, BL, BR, TR（左上/左下/右下/右上），与 `CoordTransformer` 模型点一致。
   在此约定下：正对板 `yaw_in_camera ≈ 0`，绕相机 y 轴 +10° 旋转 → yaw = -10°，
   与接口文档一致。**合成数据必须用此约定**（把 rvec 设为小角度/0，而不是
   用 `[0,π,0]` 表示"朝向相机"，后者是镜像约定，会产生 yaw≈±180° 的假象）。
2. **IPPE 主解**：`solvePnP(IPPE)` 单解与 `solvePnPGeneric` 的 sol0 一致；
   大多数构型下 sol0=真解；但存在镜像构型（如 t=[0.5,0.2,3]、偏航 45° 时
   sol0=镜像解、真解在 sol1）。这是 4 点共面 PnP 的固有前/后歧义（两解平移几乎
   相同、仅朝向互为镜像，单帧无法从几何区分），旧实现同样存在；
   B-6 的上一帧锚定保证帧间稳定，但首帧若落进镜像槽会一直保持（与旧实现行为
   一致，不构成回归）。未来可考虑用灯条侧信息或 EKF 融合破除此歧义。
3. **LM 精修护栏**：`solvePnPRefineLM` 在近退化构型（对称角点、接近零空间）
   会沿退化方向漂移到另一等误差解（实测 2m → 3.25m、朝向翻转 180°）。已加护栏：
   精修后重投影误差不显著变差（≤1.05×）且朝向跳变 < 20° 才接受，否则保留原解。
4. **朝向对噪声极其敏感**：2 m 处 20° 偏航只体现为左右灯条 0.53 px 的高度差，
   0.3 px 角点噪声即可把 yaw 洗偏 17°（实测 -20° → -3°）；距离不受影响
   （1.6% 误差）。3 m 处 45° 偏航信号 6.2 px，yaw 才稳定。B-3 评测"朝向角误差"
   指标要考虑此物理下限，重点看静止靶帧间标准差（B-6 验收②）而非单帧绝对值。
5. **pnp_check 工具**（B-4 验收③配套，`tools/replay/pnp_check.cpp`）用同一套
   合成数据验证：正对 2m 距离误差 0.03%、偏航 45°@3m 距离误差 0.51%、偏置 2m
   误差 1.64%，均 < 2% 达标。

## 涉及文件

- `docs/module-contracts.md`：I1/I2 待定项补全、R1/R2 格式定稿（B-1 初稿）
- `mathutils/CoordTransformer.{hpp,cpp}`：Doxygen 补全；B-6 `solvePnPGeneric`
  双解消歧（上一帧朝向最近邻选解）+ `solvePnPRefineLM` 精修（yml 开关
  `pnp_disambiguate`/`pnp_refine_lm`）；B-7 `kLargeArmorTagIds` 常量集中化
- `detect/armor_corner_optimizer.{hpp,cpp}`：`CornerRefineCallStats` 增加
  `match_cost`/`final_check_passed`；`selectBestLightBar` 输出得分
- `detect/corner_refine_submodule.{hpp,cpp}`：B-5 颜色未知红蓝双阈值策略，
  在线与回放共用 `optimizeCornersWithEnemyColor`
- `tools/handeye_calibration.py`：B-F6/B-F8 改造 + 约定修正 + 残差报告与门槛
- `tools/intrinsics_calibration.py`：新建，棋盘格内参标定（合成数据验证 K 误差
  <0.5%、RMS 0.13 px；重投影误差 >0.3 px 默认拒绝写文件）
- `tools/replay/`：新建（`replay_main.cpp`、`avi_to_r1.cpp`、`pnp_check.cpp`、
  `json_min.hpp`、`CMakeLists.txt`、`README.md`）

## 待办

- B-4 实机标定（10-31/11-01）：采集 ≥25 张标定板图 + 手眼数据，用新脚本跑；
  内参重投影误差 < 0.3 px，PnP 距离误差 < 2%。
- B-2 验收②（回放 vs 在线一致性 < 1e-6）等 A-4 的实机 R1（可顺延至 11-15）。
- B-5/B-6 精度指标等 B-3 评测集（11-02 起）。
- B-7 等 A-10 标签对照表。
