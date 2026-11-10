# 模块 M1 感知前端：负责人 A

> 目标：在 Orin Nano Super 上跑通，提升 FPS，同时保证检测质量不下降。
> 总览与公共规则见 [team-division-orin-nano-super.md](team-division-orin-nano-super.md)，协作规范见 [docs/collaboration-spec.md](../docs/collaboration-spec.md)，接口见 [docs/module-contracts.md](../docs/module-contracts.md)。
> 本文只列要改什么和怎样算完成，不排时间。任务编号表示建议顺序，"前置"一列写明依赖。

## 1. 模块边界

| 项目 | 内容 |
|------|------|
| 输入 | 海康相机帧或视频文件；消息桥上的 IMU 姿态和机器人状态 |
| 输出（接口 I1） | `ThreadDataPack` 中的 `frame`、`time`、`attitude`、`robotstatus`、`bboxes`（字段约定见 `common/datatype.hpp` v1） |
| 可以修改 | `sensor/`、`detect/` 中除 `corner_refine_submodule.*`、`armor_corner_optimizer.*` 外的全部文件、`asset/models/`、模型训练与导出脚本 |
| 不能修改 | `common/`、`main.cpp`、根 `CMakeLists.txt`、`launch.cfg`（需要三人批准）；B、C 的目录；`timedserial/`（电控，只读） |
| 对外交付 | R1 录制数据（给 B）；曝光到取图的延迟数值（给 C）；性能基线报告 |

## 2. 现状问题（源码静态检查，未在 Orin 上实测）

| # | 问题 | 位置 |
|---|------|------|
| A-F1 | Orin 只能走 `TRT` 后端，README 标注该后端"维护与验证不充分"，主线是 AX650 | `README.md`、`detect/CMakeLists.txt` |
| A-F2 | `TRTModule` 使用 TensorRT 10 已移除的 API：`getBindingIndex`、`getBindingDimensions`（117–119 行）、`setMaxWorkspaceSize`（179 行）、`enqueue(1, ...)`（232 行）。JetPack 6.x 自带 TensorRT 10，现有代码在 Orin 上无法编译 | `detect/TensorRT/TRTModule.cpp` |
| A-F3 | TRT 路径按输出张量名 `output-topk`、`TOPK_NUM=128`、每候选 21 个值解析；`docs/dependencies-and-assets.md` 写的是 6720 个候选。两者是否一致没有验证 | `TRTModule.cpp:118,238`、`TRTModule.hpp` |
| A-F4 | 预处理全部在 CPU 上做：`cv::resize`、`cvtColor`，然后在 TRT 模块里 `convertTo(CV_32F)`、`blobFromImage`，再拷贝到显存。每帧都新建 `cv::Mat` | `detect/preprocess_submodule.cpp:30-31`、`TRTModule.cpp:224-231` |
| A-F5 | `data->time` 记录的是 `video->read()` 返回之后的时刻，不包含曝光、传输和 SDK 缓冲的延迟 | `sensor/sensor_submodule.cpp:101-102` |
| A-F6 | PnP 按 `armor_number` 为 0、1、8 判定大装甲板，而 `datatype.hpp` 中 `tag_id=0` 表示无目标。模型标签表与这个判定是否一致没有记录 | `mathutils/CoordTransformer.cpp:146,152` |

## 3. 任务清单

### A-1 Orin 环境记录

| 项目 | 内容 |
|------|------|
| 要改的 | 新建 `docs/orin-env.md`：模组内存（4GB/8GB）、JetPack、L4T、TensorRT、CUDA、cuDNN、OpenCV（是否带 CUDA）、Eigen、MVS 的版本；`nvpmodel` 模式；是否执行 `jetson_clocks`；风扇策略 |
| 验收标准 | ① 文档入库；② 另一名成员按文档在同型号 Orin 上配出一样的环境，并能完成 A-2 的编译 |
| 前置 | 无 |

### A-2 TensorRT 10 迁移

| 项目 | 内容 |
|------|------|
| 要改的 | `detect/TensorRT/TRTModule.cpp/.hpp`：改用 `getNbIOTensors`/`getIOTensorName`/`getTensorShape` 按名称取张量；`setTensorAddress` + `enqueueV3`；`setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, ...)`；按 TRT 版本处理 `createNetworkV2` 的显式 batch 标志；CUDA/TRT 资源改用 RAII 管理（规范 §4.3.6）。如有需要，修改 `detect/CMakeLists.txt` 的 TRT 分支（根 `CMakeLists.txt` 不动） |
| 验收标准 | ① 在 Orin 上执行 `cmake -S . -B build -DINFERENCE_BACKEND=TRT && cmake --build build` 无错误；② `launch.cfg` 设 `source=test.avi`、`port=None`，`test.avi` 完整跑完无崩溃、无 TRT 报错；③ 第一次运行生成引擎缓存，第二次运行从缓存加载 |
| 前置 | A-1 |

### A-3 模型输出与解码核对

| 项目 | 内容 |
|------|------|
| 要改的 | 用 Netron 或 `polygraphy inspect model` 确认 `SKD250526.onnx` 的输入/输出张量名、形状和数据类型；按实际输出修正 `TRTModule` 的解码（候选数、每候选字段、颜色和类别字段位置、NMS）。结论写进 `asset/models/SKD250526.md` |
| 验收标准 | ① 在 `test.avi` 中抽 ≥ 200 帧，用 ONNX Runtime（CPU，FP32）跑同一模型作为参考，TRT FP32 结果与参考结果的检测框一一匹配（IoU ≥ 0.9 的匹配率 ≥ 99%），类别和颜色一致；② 对比脚本入库（`tools/` 下由 A 新建） |
| 前置 | A-2 |

### A-4 R1 录制开关

| 项目 | 内容 |
|------|------|
| 要改的 | 在 Detect 子模块输出处（角点精修之前）按 `docs/module-contracts.md` 的 R1 格式写盘：原始帧、`index`、`time`、`attitude`、`robotstatus.enemy_color`、`bboxes`、`kDataTypeVersion`。写盘放在独立线程，不阻塞流水线。`launch.cfg` 新增开关字段（需三人批准） |
| 验收标准 | ① 开关关闭时，各 stage 耗时与关闭前相比无变化（P50 差异 < 2%）；② 开关打开时，B 的回放工具能完整读取录制文件，帧数与 `index` 连续性检查通过；③ 录制格式字段与接口文档一致 |
| 前置 | A-2；R1 格式定稿（B-1） |

### A-5 性能基线

| 项目 | 内容 |
|------|------|
| 要改的 | 利用 `EntryStage` 的计时日志（`entrystage_log_file=true`）统计 Sensor、Preprocess、Detect、CornerRefine、Predict、Planner 各阶段耗时；写一个统计脚本，输出 P50/P99 和端到端 FPS。报告写到 `docs/experiments/` |
| 验收标准 | ① 报告包含测量条件（规范 §6.5）：功耗模式、`jetson_clocks`、输入源、帧数、预热帧数；② `test.avi` 和实机相机两种输入各测一次，每次 ≥ 3000 帧，去掉前 100 帧预热；③ 同一条件重复 3 次，P50 波动 < 5%；④ 统计脚本入库 |
| 前置 | A-2 |

### A-6 预处理移到 GPU

| 项目 | 内容 |
|------|------|
| 要改的 | 写一个 CUDA kernel，一次完成 resize + BGR→RGB + 归一化 + HWC→CHW，直接写入 TRT 输入显存；利用 Orin 的统一内存（`cudaHostAlloc` 映射内存或 `cudaMallocManaged`）省掉 H2D 拷贝；去掉每帧新建的 `cv::Mat`。`detect_input` 字段仍然保留，供调试显示 |
| 验收标准 | ① GPU 预处理输出与原 CPU 版本逐元素最大绝对误差 < 1e-3（在 ≥ 200 帧上验证）；② A-3 的检测一致性测试仍然通过；③ 按 A-5 的方法测量，Preprocess + Detect 的 P50 低于基线 |
| 前置 | A-3、A-5 |

### A-7 FP16 / INT8 引擎

| 项目 | 内容 |
|------|------|
| 要改的 | 引擎构建时开启 FP16；评估 INT8（校准集从 A-9 的检测数据集中抽取，≥ 500 张，覆盖各种光照）。引擎缓存文件名包含 TRT 版本、精度和 GPU 型号，避免误用旧缓存 |
| 验收标准 | ① FP16：检测 test 集 mAP@0.5 相比 FP32 下降 ≤ 0.5 个百分点（建议阈值，三人可调整）；② INT8：同样的阈值，否则不启用，并把结论写进实验记录；③ 按 A-5 方法测量的 Detect P50 低于 FP32 |
| 前置 | A-3、A-9 的 test 集 |

### A-8 曝光时间戳与延迟数值

| 项目 | 内容 |
|------|------|
| 要改的 | 评估海康 SDK 的帧时间戳（如设备时间戳、主机时间戳）能否换算到 `FrameClock`。可以的话，让 `data->time` 改记曝光时刻；不可以的话，测出"曝光 → `read()` 返回"的平均延迟（例如拍摄毫秒计时器或 LED），写进 `docs/module-contracts.md` 的 M1 延迟表 |
| 验收标准 | ① 接口文档中的 M1 延迟表填有数值、测量方法和测量日期；② 如果改了 `time` 的含义，`datatype.hpp` 注释和接口文档同步更新，并经三人批准 |
| 前置 | A-2 |

### A-9 检测数据集与模型迭代

| 项目 | 内容 |
|------|------|
| 要改的 | 建检测数据集：从比赛和训练录像抽帧标注，覆盖远距离、运动模糊、灯条熄灭（灰色）、过曝、反光，划分 train/val/test，test 集固定版本并记录 SHA-256；做数据增强；按耗时预算选择输入尺寸或剪枝；每个模型写 `asset/models/<名>.md` 溯源记录（规范 §6.3） |
| 验收标准 | ① 数据集版本、路径、SHA-256 登记在接口文档；② 新模型在 test 集上的 mAP@0.5 和漏检率不差于当前模型；③ 角点像素误差（与标注真值比较）不差于当前模型；④ 按 A-5 方法测量的 Detect 耗时在预算以内；⑤ 新模型通过 A-3 同样的一致性测试 |
| 前置 | A-3 |

### A-10 标签表与装甲尺寸判定

| 项目 | 内容 |
|------|------|
| 要改的 | 整理模型原始类别到 `tag_id` 的对照表，确认哪些编号是大装甲板，结果写进接口文档。若与 `CoordTransformer.cpp` 中"0、1、8 为大装甲板"的判定不一致，把结论交给 B 修改（A 不改 B 的文件） |
| 验收标准 | ① 对照表入库；② 与 B 确认后，PnP 的大小装甲判定与对照表一致 |
| 前置 | A-3 |

### A-11 相机参数调优

| 项目 | 内容 |
|------|------|
| 要改的 | 通过海康 SDK 调整曝光、增益、ROI、取流方式（如抓图策略、缓冲区数量），减少 Sensor 等待时间和运动模糊 |
| 验收标准 | ① 检测 test 集或现场录像上的 mAP、漏检率不下降；② 按 A-5 方法测量的 Sensor P50 或端到端 FPS 有提升；③ 相机参数写在配置里，不硬编码 |
| 前置 | A-5 |

## 4. 模块指标

| 指标 | 测量方式 |
|------|----------|
| 端到端 FPS、Sensor/Preprocess/Detect 的 P50/P99 | A-5 的统计脚本 |
| 曝光到 bbox 输出的延迟 | A-8 |
| 检测 mAP@0.5、漏检率、角点像素误差 | A-9 的 test 集 |
| TRT 与 ONNX 参考结果的一致率 | A-3 的对比脚本 |

FPS 和耗时的目标值在 A-5 得到基线后，由三人一起确定，写进 `docs/module-contracts.md` 的耗时预算表。

## 5. 合入前自查

- [ ] 只改了第 1 节"可以修改"范围内的文件
- [ ] `bboxes` 的字段含义、角点顺序和坐标系没有变化（接口 I1）
- [ ] Orin 上编译通过，`test.avi` + `port=None` 跑完无崩溃
- [ ] MR 描述里有指标对比表（基线 → 本次），测量条件写全
- [ ] 模型或引擎有变化时，更新了 `asset/models/<名>.md`
