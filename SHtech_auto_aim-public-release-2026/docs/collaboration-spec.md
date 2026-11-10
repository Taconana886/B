# 算法团队开发合作规范 v1.0

> 适用范围：Auto-Aim 视觉算法链路（`sensor/`、`detect/`、`predict/`、`planner/`、`mathutils/`、`common/`、`asset/`、`tools/`）。
> 不适用：MCU 固件、串口协议、云台控制等电控工作。`timedserial/` 对算法组只读。
> 生效日期：2026-10-18。修订本规范本身也要走第 5 节的评审流程，并且需要三人全部同意。

## 0. 参考来源与条款等级

本规范没有照抄任何一份大厂规范，而是从以下公开规范中挑出适合 3 人竞赛算法组的部分，再按本仓库现状做了调整：

| 来源 | 借鉴内容 |
|------|----------|
| Google C++ Style Guide | 命名、头文件、作用域、所有权、异常与断言 |
| Google Engineering Practices（Code Review Developer Guide） | 小 CL、评审标准、评审时限、作者与评审者职责 |
| 阿里巴巴《Java 开发手册》 | 【强制】/【推荐】/【参考】三级条款写法 |
| Conventional Commits 1.0.0 | 提交信息格式 |
| Semantic Versioning 2.0.0 | 发布 tag 命名 |
| GitHub / GitLab CODEOWNERS | 目录所有权与强制评审 |
| ML 工程实践（实验可复现） | 数据集版本、指标对比、随机种子、模型溯源 |

条款等级：

- **【强制】**：违反即不能合入 `main`。评审者必须拦截。
- **【推荐】**：默认要做。不做时要在 MR/PR 描述里写理由。
- **【参考】**：好的做法，自行判断。

---

## 1. 角色与所有权

1. 【强制】每个目录只有一个 Owner，记录在根目录 `CODEOWNERS`。Owner 对该目录的代码质量、指标和文档负责。
2. 【强制】修改别人拥有的目录时，必须由该目录的 Owner 批准。
3. 【强制】以下"冻结接口"的修改需要三人全部批准：
   - `common/`，其中最关键的是 `common/datatype.hpp`（`ThreadDataPack`、`bbox_t`、`color_id`/`tag_id` 编号）和 `common/pipeline.hpp`
   - `main.cpp`、`main.hpp`、根 `CMakeLists.txt`
   - `mathutils/math_tools.hpp`
   - 模块接口与录制格式（`docs/module-contracts.md`）
   - `launch.cfg` 字段名与含义
   - 完整清单以 `CODEOWNERS` 的共享冻结区为准
4. 【推荐】Owner 离队或长期缺席时，在例会上书面移交，并同时更新 `CODEOWNERS`。

## 2. 版本管理与分支

1. 【强制】仓库托管在队内 GitLab/GitHub。`main` 设为保护分支，禁止直接 push 和 force push。
2. 【强制】分支模型（简化版 GitHub Flow + 集成分支）：

   ```text
   main  ← 只放上车验证过的版本，每次合入都打 tag
    └── dev  ← 日常集成，必须能编译并通过离线回放
         ├── feat/<角色>-<主题>      例：feat/a-trt10-migration
         ├── fix/<角色>-<主题>       例：fix/c-armor-jump-interval
         ├── perf/<角色>-<主题>      例：perf/a-cuda-preprocess
         └── exp/<角色>-<主题>       实验分支，不合入，结论写进实验记录
   ```

   角色代号固定为 `a`/`b`/`c`。
3. 【强制】功能分支从 `dev` 拉出，合入前先 rebase 到最新 `dev`。合入时使用 squash merge，保持 `dev` 历史线性。
4. 【推荐】功能分支寿命不超过 5 个工作日。超过时拆分，或者用配置开关先合入未启用的代码。
5. 【强制】发布 tag 采用 `vMAJOR.MINOR.PATCH`（SemVer）：
   - MAJOR：冻结接口不兼容变更
   - MINOR：新功能或新模型
   - PATCH：参数调整、缺陷修复
   比赛前定版另外打 `match-<赛事>-<日期>` 标签，例如 `match-rmuc-20270510`。

## 3. 提交规范

1. 【强制】提交信息遵循 Conventional Commits：

   ```text
   <type>(<scope>): <subject>

   <body：为什么改、改了什么行为、怎么验证的>

   <footer：Refs/BREAKING CHANGE>
   ```

   - `type`：`feat`、`fix`、`perf`、`refactor`、`test`、`docs`、`build`、`chore`、`exp`
   - `scope`：目录名，即 `detect`、`predict`、`planner`、`sensor`、`common`、`mathutils`、`asset`、`tools` 之一
   - `subject`：中文或英文均可，不超过 50 个字符，不加句号
   - 例：`fix(planner): 修正 armor_jump_interval 被截断为 0 的问题`
2. 【强制】一个提交只做一件事。不要把格式化、重构和功能改动混在同一个提交里。
3. 【强制】改动冻结接口时，footer 写 `BREAKING CHANGE: <说明>`。
4. 【强制】禁止提交：`build/`、日志、录像原始数据、`*.cache`/`*.engine` 等 TensorRT 引擎缓存、凭据、个人 IDE 配置。
5. 【推荐】单个文件超过 10 MB 的模型或数据使用 Git LFS，或者放到固定的共享存储，仓库里只记录路径和 SHA-256。

## 4. 代码规范（C++）

以 Google C++ Style Guide 为基础。为了和现有代码保持一致，有以下本地例外。

### 4.1 格式

1. 【强制】使用根目录 `.clang-format`：4 空格缩进，行宽 120，namespace 内缩进。
2. 【强制】**只格式化你改动的行**（`git clang-format dev`）。禁止对整个文件或整个仓库批量重排，否则 diff 无法评审。如果需要全量格式化，单独发一个 `style:` MR，三人同意后再合入。

### 4.2 命名（与现有代码保持一致）

| 对象 | 规则 | 例 |
|------|------|----|
| 类型、类、结构体 | PascalCase | `PlannerSubModule`、`ThreadDataPack` |
| 函数、变量 | snake_case | `generate_command_array`、`dt_since_jump` |
| 类私有成员 | snake_case + 后缀 `_` | `config_`、`resized_bgr_` |
| 常量、constexpr | `kPascalCase`（新代码）；旧代码的全大写常量保留 | `kTopkNum`；旧 `TOPK_NUM` |
| 宏 | 全大写，尽量不用 | `TRT_ASSERT` |
| 命名空间 | 小写 | `predict`、`plan` |

### 4.3 语言用法

1. 【强制】头文件里不写 `using namespace`。现存的 `using namespace mathutils;` 等不新增，修改该文件时顺手清理。
2. 【强制】物理量的变量名或注释必须写明单位与坐标系，例如 `yaw_rad`、`dist_m`、`latency_ms`，或者写注释 `// 世界系, m`。角度一律注明是度还是弧度。（README 已指出 Planner 中度和弧度混用，这是命中率问题的常见来源。）
3. 【强制】数值类型要匹配：时间、角度、距离不得声明为整型（反例：`const int armor_jump_interval = 0.1f;`）。开启 `-Wconversion` 编译新代码。
4. 【强制】跨线程共享的数据必须加锁，或使用 `std::atomic`。读取被锁保护的对象时同样要加锁。
5. 【强制】热路径（每帧执行的 `process()`）中禁止：动态分配大块内存、`std::cout`/同步日志、`cv::imshow`、阻塞式 sleep。调试输出受 `launch.cfg` 中的 `*_log_*` 开关控制。
6. 【推荐】资源用 RAII 管理（`std::unique_ptr` 或自定义 deleter 管理 CUDA、TensorRT、相机句柄），不手写 `new`/`delete`。
7. 【推荐】可调参数从 yml 或 `launch.cfg` 读取，不硬编码魔数。必须写死的常量要加注释说明来源。
8. 【参考】新代码优先使用 `Eigen` 固定尺寸类型，避免在热路径中创建动态尺寸矩阵。

### 4.4 注释与文档

1. 【强制】对外类和函数写 Doxygen 注释（`@brief`、`@param`、`@return`），中英文均可。
2. 【强制】注释说明"为什么"，不复述代码。经验参数要写出处，例如"2026-10-25 旋转靶标定"。
3. 【强制】行为变化要同步更新 README 或 `docs/` 中对应的段落。修复 README"已查到但未修正的问题"中的条目时，要把该行移除或改写。

### 4.5 Python（tools/、训练脚本）

1. 【强制】PEP 8，使用 `ruff` 格式化和检查。函数要有类型标注。
2. 【推荐】依赖写进 `tools/requirements.txt`，或使用 `uv` 的 `pyproject.toml`。

## 5. 代码评审（参照 Google Code Review Developer Guide）

### 5.1 作者职责

1. 【强制】MR/PR 小而聚焦：改动一般不超过 400 行（不含自动生成文件和参数文件）。超过时拆分。
2. 【强制】提交评审前自查：本地编译通过，离线回放（`test.avi` + `port=None`）不崩溃，`git clang-format` 已执行。
3. 【强制】MR 描述使用以下模板：

   ```markdown
   ## 改了什么
   ## 为什么改
   ## 影响范围
   - 涉及冻结接口：是/否
   - 影响模块：
   ## 验证
   - 平台：Orin Nano Super / JetPack x.y / TensorRT x.y
   - 评测集与版本：
   - 指标对比（基线 → 本次）：
   | 指标 | 基线 | 本次 |
   ## 风险与回滚
   ```

### 5.2 评审者职责

1. 【强制】评审标准："这个改动是否让代码库整体变得更好"，而不是"是否完美"。不要因为个人偏好卡住合入。
2. 【强制】评审重点按以下顺序：正确性（单位、坐标系、线程安全、边界条件）> 指标证据 > 设计 > 可读性 > 风格。
3. 【强制】1 个工作日内给出首轮回复。比赛前两周内缩短到当天。
4. 【推荐】评论分级：`必须:` 不改不能合；`建议:` 作者自行决定；`问题:` 只是提问。
5. 【强制】作者不能批准自己的 MR。修改他人目录时，Owner 必须是批准人之一。

### 5.3 合入门槛

| 目标分支 | 条件 |
|----------|------|
| `dev` | 1 人批准（Owner 优先）+ 编译通过 + 离线回放通过 + 附指标对比 |
| `main` | 3 人批准 + 实机验证记录（第 7 节）+ 打 tag |

## 6. 实验与数据规范（算法组特有）

1. 【强制】**没有数据就不合入**。凡是声称提升 FPS、精度或命中率的改动，必须在同一评测集、同一硬件和功耗模式下给出基线与改动后的对比。
2. 【强制】评测集固定版本：`<名称>-v<版本>`，记录 SHA-256 与采集日期。评测集冻结后不再修改，只发布新版本。
3. 【强制】模型溯源：每个模型文件都要有对应的 `asset/models/<模型名>.md`，写明训练数据版本、代码 commit、超参数、导出 opset、输入输出约定、验证指标、SHA-256。
4. 【强制】参数文件（`camParam`、`plannerParam`）不覆盖旧文件，新建带日期或设备编号的文件。标定要附重投影误差。
5. 【强制】性能测试条件写明：`nvpmodel` 模式、是否执行了 `jetson_clocks`、输入源（相机/录像）、测试时长、P50/P99。
6. 【推荐】实验记录写在 `docs/experiments/YYYYMMDD-<角色>-<主题>.md`，结论为负的实验也要记录。
7. 【推荐】训练脚本固定随机种子，并记录环境（`pip freeze`、CUDA 版本）。

## 7. 上车与发布

1. 【强制】只用 `main` 上的 tag 构建上车，禁止在车上直接改代码后比赛。
2. 【强制】上车记录：tag、`launch.cfg` 快照、相机参数文件名、场景（距离/靶型/光照）、发弹数与命中数。每档至少 50 发。
3. 【强制】赛前一周冻结代码：只允许 `fix:` 和参数调整，并且需要三人批准。
4. 【推荐】每次发布保留上一稳定 tag 的构建产物，便于现场快速回滚。

## 8. 沟通

1. 【强制】每周一次例会，30 分钟以内。每人汇报：本周指标变化、阻塞项、下周计划。纪要写在 `plan/weekly/YYYY-WW.md`。
2. 【强制】接口变更、模型 I/O 变更要提前在群里和例会上通知，不能只在 MR 里说明。
3. 【推荐】阻塞超过半天，主动找相关 Owner 当面或语音沟通。
