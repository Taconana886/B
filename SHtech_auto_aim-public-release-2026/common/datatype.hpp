//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refactored the framework
// Modified by Haoran Jiang on 21-10-21: Updated RobotStatus structure and file layout
// Classes of Common Data Type
//
// ============================================================================
// 冻结接口 v1（2026-10-18）
//
// 本文件是三个模块之间传递数据的唯一载体，属于共享冻结区：
//   - 任何修改都需要 A、B、C 三人批准（docs/collaboration-spec.md §1.3）
//   - 改变字段含义、单位、类型或删除字段时，kDataTypeVersion 加 1，
//     提交信息 footer 写 BREAKING CHANGE（§3.3），并同步 docs/module-contracts.md
//
// 模块与写入权（每个字段只有一个写入方，其他模块只读）：
//   M1 感知前端（A）：EntryStage、Sensor、Preprocess、Detect
//   M2 几何精度（B）：CornerRefine（只允许改写 bboxes[i].pts 和 bboxes[i].source）
//   M3 决策（C）：MultiPolicyPredictor、Planner
//
// 单位约定：长度 m，时间 s，角度在字段注释中逐一写明度或弧度。
// ============================================================================

#ifndef COMMON_ROBOT_H
#define COMMON_ROBOT_H


// packages
#include <cassert>
#include <cstdint>
#include <array>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

/// @brief 数据结构版本号。不兼容修改时加 1，录制文件（R1/R2）中同时记录该值
constexpr int kDataTypeVersion = 1;

/**
 * @brief 帧时间戳使用的时钟
 * @details 使用单调时钟。libstdc++ 中 high_resolution_clock 等同于 system_clock，
 *          会被 NTP 校时改变，导致延迟补偿计算出错。
 */
using FrameClock = std::chrono::steady_clock;

enum class EnemyColor : uint8_t
{
    // 敌方颜色
    RED = 0,
    BLUE = 1,
    GRAY = 2,
};

enum class ProgramMode : uint8_t
{
    // 视觉模式
    IDLE = 0,     // 空闲
    AUTO_AIM = 1,     // 自瞄
    ANTIMISSLE = 2,   // 反导
    SMALL_ENERGY = 4, // 小能量机关
    BIG_ENERGY = 8,   // 大能量机关
};

// 低5位发射标志位，高3位状态标识位
enum class ShootMode : uint8_t
{
    // 射击标志位
    COMMON = 0,      // 普通模式
    DISTANT = 1,     // 远距离击打
    ANTITOP = 2,     // 反陀螺
    SWITCH = 4,      // 快速切换装甲板
    FOLLOW = 8,      // 跟随不发弹
    CRUISE = 16,     // 巡航
    EXIST_HERO = 32, // 英雄存在
};

enum class GameState : uint8_t
{
    // 比赛模式
    SHOOT_NEAR_ONLY = 0, // 仅射击近处
    SHOOT_FAR = 1,       // 允许远处射击
    COMMON = 255,        // 巡航
};

enum class SubModuleResult : uint8_t
{
    // 子模块处理结果
    SUCCESS,    // 成功，数据有效
    SKIP,       // 模块处理被跳过
    FAILURE,    // 失败
    NOTYET,     // 尚未处理
};

enum class SubModuleName : uint8_t
{
    // 子模块名称
    ENTRYSTAGE,
    SENSOR,
    PREPROCESS,
    DETECT,
    CORNER_REFINE,
    MULTI_POLICY_PREDICTOR,
    PLANNER,

    COUNT, // 仅用于计数子模块数量
};

enum class DetectionSource : uint8_t
{
    // 角点来源
    NEURAL_NETWORK = 0, // 神经网络输出的角点（M1 写入，或 M2 精修失败时保留）
    TRADITIONAL = 1,    // 传统视觉精修后的角点（M2 精修成功时写入）
};

// 根据 SubModuleName 自动生成子模块数量常量
constexpr size_t SUBMODULE_COUNT = static_cast<size_t>(SubModuleName::COUNT);

// 获取子模块名称字符串
inline const char* getSubModuleName(SubModuleName module) {
    switch (module) {
        case SubModuleName::ENTRYSTAGE: return "Entry";
        case SubModuleName::SENSOR: return "Sensor";
        case SubModuleName::PREPROCESS: return "Preprocess";
        case SubModuleName::DETECT: return "Detect";
        case SubModuleName::CORNER_REFINE: return "CornerRefine";
        case SubModuleName::MULTI_POLICY_PREDICTOR: return "Predict";
        case SubModuleName::PLANNER: return "Planner";
        default: return "Unknown";
    }
}

constexpr float INF_BALL_SPEED = 30.0f; // 裁判系统默认值，用于状态合并，不是临时 Mock 射速

/**
 * @brief 原图到网络输入的映射（M1 内部使用）
 */
struct DetectInputMap
{
    cv::Rect src_roi{};   ///< 原图中参与推理的区域，像素
    cv::Size dst_size{};  ///< 网络输入尺寸，像素
    bool valid = false;
};

/**
 * @brief 下位机上报的机器人状态（Sensor 从消息桥读取，M1 写入）
 */
struct RobotStatus
{
    ProgramMode program_mode = ProgramMode::AUTO_AIM;
    float robot_speed_mps = INF_BALL_SPEED;    ///< 弹速，m/s（当前 Uart 路径为临时 Mock 值，见 README）
    uint16_t enemy[6]{};                       ///< 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
    GameState game_state = GameState::COMMON; ///< 是否射击远处
    EnemyColor enemy_color = EnemyColor::RED;
};

/**
 * @brief 发往 TimedSerial 的云台指令（M3 写入）
 * @details 字段顺序与串口协议相关，属于电控接口，不得调整顺序。
 *          角度为相对当前云台姿态的增量。
 */
struct RobotCommand
{
    float distance = 0.0f;    ///< 目标距离，m
    float yaw_angle = 0.0f;   ///< 偏航角增量，度
    float yaw_speed = 0.0f;   ///< 偏航角速度，rad/s
    float yaw_acc = 0.0f;     ///< 偏航角加速度，rad/s²
    float pitch_angle = 0.0f; ///< 俯仰角增量，度
    float pitch_speed = 0.0f; ///< 俯仰角速度，rad/s
    float pitch_acc = 0.0f;   ///< 俯仰角加速度，rad/s²
    int fire_enable = 0;      ///< 0 is disable, 1 is enable, 2 is self-determined, 3 is single shoot
    int target_id = 0;        ///< 1-7: robot, 8: outpost, 9: base, 0: none
};


// 射击指令线性插值，对target_id和shoot_mode采纳时间上更接近的指令
inline RobotCommand command_linear_interpolation(const RobotCommand& cmd1, const RobotCommand& cmd2, float cmdTwoWeight){
    assert(cmdTwoWeight >= 0.0f && cmdTwoWeight <= 1.0f&&"command_linear_interpolation: cmdTwoWeight out of range [0,1]");
    if(cmdTwoWeight < 0.5f)
        return RobotCommand{
            cmd1.distance * (1-cmdTwoWeight) + cmd2.distance * cmdTwoWeight,
            cmd1.yaw_angle * (1-cmdTwoWeight) + cmd2.yaw_angle * cmdTwoWeight,
            cmd1.yaw_speed * (1-cmdTwoWeight) + cmd2.yaw_speed * cmdTwoWeight,
            cmd1.yaw_acc * (1-cmdTwoWeight) + cmd2.yaw_acc * cmdTwoWeight,
            cmd1.pitch_angle * (1-cmdTwoWeight) + cmd2.pitch_angle * cmdTwoWeight,
            cmd1.pitch_speed * (1-cmdTwoWeight) + cmd2.pitch_speed * cmdTwoWeight,
            cmd1.pitch_acc * (1-cmdTwoWeight) + cmd2.pitch_acc * cmdTwoWeight,
            cmd1.fire_enable,
            cmd1.target_id
        };
    return RobotCommand{
            cmd1.distance * (1-cmdTwoWeight) + cmd2.distance * cmdTwoWeight,
            cmd1.yaw_angle * (1-cmdTwoWeight) + cmd2.yaw_angle * cmdTwoWeight,
            cmd1.yaw_speed * (1-cmdTwoWeight) + cmd2.yaw_speed * cmdTwoWeight,
            cmd1.yaw_acc * (1-cmdTwoWeight) + cmd2.yaw_acc * cmdTwoWeight,
            cmd1.pitch_angle * (1-cmdTwoWeight) + cmd2.pitch_angle * cmdTwoWeight,
            cmd1.pitch_speed * (1-cmdTwoWeight) + cmd2.pitch_speed * cmdTwoWeight,
            cmd1.pitch_acc * (1-cmdTwoWeight) + cmd2.pitch_acc * cmdTwoWeight,
            cmd2.fire_enable,
            cmd2.target_id,
        };
}


/**
 * @brief 装甲板检测结果，接口 I1 的数据单元
 * @details M1 写入全部字段；M2 只允许改写 pts 和 source。
 */
struct bbox_t
{
    /// 四个角点，原图像素坐标（M1 已从网络输入尺寸映射回原图）。
    /// 顺序与 PnP 物体点一致：[0]左上、[1]左下、[2]右下、[3]右上（装甲板正视时）
    cv::Point2f pts[4];
    float confidence = 0.0f;  ///< 检测置信度
    int color_id = 2;         ///< 0: red, 1: blue, 2: gray
    int tag_id = 0;           ///< 0: none, 1-7: robot, 8: outpost, 9: base
    DetectionSource source = DetectionSource::NEURAL_NETWORK; ///< 角点来源

    bool operator==(const bbox_t &a) const
    {
        return pts[0] == a.pts[0] && pts[1] == a.pts[1] && pts[2] == a.pts[2] && pts[3] == a.pts[3];
    }
    bool operator!=(const bbox_t &a) const
    {
        return !(*this == a);
    }

    bool operator < (const bbox_t &a) const
    {
        return confidence < a.confidence;
    }

    bool operator > (const bbox_t &a) const
    {
        return confidence > a.confidence;
    }
};

/**
 * @brief 云台 IMU 姿态（Sensor 从消息桥读取，M1 写入）
 * @details yaw/pitch/roll 单位为度，旋转顺序 Z(yaw)-X(pitch)-Y(roll)。
 *          使用方需要弧度时自行乘 π/180。
 */
class Attitude
{
public:
    // 1. 默认构造函数：ypr等于0，即水平姿态
    Attitude() : Attitude(0.0f, 0.0f, 0.0f) {}

    Attitude(float yaw, float pitch, float roll)
        : yaw_(yaw), pitch_(pitch), roll_(roll)
    {
        const double deg2rad = M_PI / 180.0;

        Eigen::Quaterniond q =
            Eigen::AngleAxisd(static_cast<double>(yaw)   * deg2rad, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(static_cast<double>(pitch) * deg2rad, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(static_cast<double>(roll)  * deg2rad, Eigen::Vector3d::UnitY());

        // 计算 World -> IMU 旋转
        // R_world2imu_ = R_initial * R_relative.transpose()
        R_world2imu_ = get_initial_transform() * q.toRotationMatrix().transpose();
    }

    float yaw() const { return yaw_; }       ///< 度
    float pitch() const { return pitch_; }   ///< 度
    float roll() const { return roll_; }     ///< 度
    const Eigen::Matrix3d& R_world2imu() const { return R_world2imu_; }

private:
    float yaw_;
    float pitch_;
    float roll_;
    Eigen::Matrix3d R_world2imu_;

    /**
     * @brief 自定义坐标系到PnP坐标系的转换矩阵
     * @details 该矩阵用于将自定义的世界坐标系转换为PnP算法使用的标准坐标系
     *          实现坐标系的翻转和轴交换
     */
    static const Eigen::Matrix3d& get_initial_transform()
    {
        static const Eigen::Matrix3d R = (Eigen::Matrix3d() <<
            -1.,  0.,  0.,
             0.,  0.,  1.,
             0.,  1.,  0. ).finished();
        return R;
    }
};

/**
 * @enum AimedTargetType
 * @brief 瞄准目标类型枚举 - 定义不同的预测策略模式
 * @details 根据目标运动状态选择不同的瞄准策略
 */
enum class AimedTargetType : uint8_t
{
    /// @brief 无目标
    NONE,

    /// @brief 无模型跟踪装甲板
    ARMOR_WITH_NO_MODEL,

    /// @brief 装甲板模型跟踪装甲板
    ARMOR_WITH_ARMOR_MODEL,

    /// @brief 整车模型跟踪装甲板
    ARMOR_WITH_VEHICLE_MODEL,

    /// @brief 整车模型瞄准车辆中心
    VEHICLE_CENTER_WITH_VEHICLE_MODEL,
};

/**
 * @struct Plan
 * @brief 预测计划结构体 - 包含云台控制和射击决策的完整信息
 * @details 由Planner模块生成，包含目标预测位置、云台角度控制参数和射击使能（M3 内部使用）
 */
struct Plan
{
    /// @brief 当前瞄准目标类型
    AimedTargetType aimed_target_type = AimedTargetType::NONE;

    /// @brief 预测的装甲板瞄准位置 [x, y, z] (米)，世界坐标系
    Eigen::Matrix<double, 3, 1> aimed_armor_pos = Eigen::Matrix<double, 3, 1>::Zero();

    /// @brief 目标偏航角 (弧度)，云台应达到的偏航角度
    double target_yaw = 0.0;

    /// @brief 目标偏航角速度 (弧度/秒)，云台偏航轴应达到的角速度
    double target_yaw_speed = 0.0;

    /// @brief 目标偏航角加速度 (弧度/秒2)，云台偏航轴应达到的角加速度
    double target_yaw_acc = 0.0;

    /// @brief 目标俯仰角 (弧度)，云台应达到的俯仰角度
    double target_pitch = 0.0;

    /// @brief 目标俯仰角速度 (弧度/秒)，云台俯仰轴应达到的角速度
    double target_pitch_speed = 0.0;

    /// @brief 目标俯仰角加速度 (弧度/秒2)，云台俯仰轴应达到的角加速度
    double target_pitch_acc = 0.0;

    /// @brief 射击使能标志 (0=禁止, 1=允许, 2=下位机决策)
    int fire_enable = 0;

    /// @brief 目标距离 (米)，目标的直线距离
    double target_distance = 0.0;
};

/**
 * @enum TrackingState
 * @brief 跟踪状态枚举 - 描述目标跟踪器的当前工作状态
 * @details 跟踪器状态机的四个状态，控制不同阶段的跟踪行为
 */
enum class TrackingState : uint8_t
{
    /// @brief 空闲状态 - 未发现目标，等待检测
    IDLE,

    /// @brief 检测状态 - 发现目标但尚未稳定跟踪
    DETECTING,

    /// @brief 跟踪状态 - 正在稳定跟踪目标
    TRACKING,

    /// @brief 暂时丢失状态 - 目标暂时消失，保持预测
    TEMP_LOST,
};

/**
 * @enum UpdatingModelType
 * @brief 模型更新类型枚举 - 指定当前使用的滤波模型类型
 * @details 根据目标运动速度动态选择使用的滤波器类型：
 *          - 低速时使用装甲板模型
 *          - 高速时使用整车模型
 *          - 中速时两个模型同时使用
 */
enum class UpdatingModelType : uint8_t
{
    /// @brief 仅更新装甲板运动模型（适用于低速运动）
    ARMOR_MODEL,

    /// @brief 仅更新整车运动模型（适用于高速旋转）
    VEHICLE_MODEL,

    /// @brief 同时更新两种模型（适用于中速过渡阶段）
    BOTH,
};

/**
 * @struct Target
 * @brief 目标跟踪结构体 - 包含目标的完整状态信息和滤波器状态
 * @details 存储跟踪器的所有状态变量，包括EKF和KF的状态估计。Predictor 写入，Planner 读取（M3 内部使用）
 */
struct Target
{
    /// @brief 当前跟踪器状态
    TrackingState predictor_state = TrackingState::IDLE;

    /// @brief 当前模型更新类型
    UpdatingModelType updating_model_type = UpdatingModelType::ARMOR_MODEL;

    /// @brief 装甲板切换计数器（0或1，标识当前跟踪的装甲板）
    int ab_counter = 0;

    /// @brief 整车模型可信标志
    bool vehicle_model_trust = false;

    /// @brief 整车状态向量 [y, vy, x, vx, z, vz, yaw, vyaw, r, dr, dz] (11x1)
    /// @details y,x,z: 车辆中心位置 (m)；vy,vx,vz: 速度 (m/s)；yaw,vyaw: 装甲板偏航角 (rad) 和角速度 (rad/s)；
    ///          r: 当前一对装甲板的旋转半径 (m)；dr: 另一对装甲板半径与 r 之差 (m)；dz: 另一对装甲板的高度差 (m)
    /// 坐标使用世界坐标系，装甲板偏航角坐标系为：正对世界坐标系y轴为0度，以世界坐标系z轴负半轴为正方向旋转
    Eigen::Matrix<double, 11, 1> tracked_state = Eigen::Matrix<double, 11, 1>::Zero();

    /// @brief 整车模型观测向量 [y, x, z, yaw] (4x1)，m 和 rad
    Eigen::Matrix<double, 4, 1> tracked_measurement = Eigen::Matrix<double, 4, 1>::Zero();

    /// @brief 当前跟踪的装甲板对象
    bbox_t tracked_armor;

    /// @brief 当前跟踪的装甲板的估计观测向量
    Eigen::Matrix<double, 4, 1> estimated_armor_m = Eigen::Matrix<double, 4, 1>::Zero();

    /// @brief 另一对装甲板的旋转半径 (米)
    double another_r = 0.0;

    /// @brief 装甲板高度差 (米)，用于处理上下装甲板
    double dz = 0.0;

    // === 装甲板模型的卡尔曼滤波器状态 ===
    /// @brief 偏航角KF状态 [yaw, yaw_velocity] (2x1)
    Eigen::Matrix<double, 2, 1> yaw_state = Eigen::Matrix<double, 2, 1>::Zero();

    /// @brief 偏航角KF观测 [yaw] (1x1)
    Eigen::Matrix<double, 1, 1> yaw_measurement = Eigen::Matrix<double, 1, 1>::Zero();

    /// @brief X坐标KF状态 [x, vx] (2x1)
    Eigen::Matrix<double, 2, 1> armor_x_state = Eigen::Matrix<double, 2, 1>::Zero();

    /// @brief X坐标KF观测 [x] (1x1)
    Eigen::Matrix<double, 1, 1> armor_x_measurement = Eigen::Matrix<double, 1, 1>::Zero();

    /// @brief Y坐标KF状态 [y, vy] (2x1)
    Eigen::Matrix<double, 2, 1> armor_y_state = Eigen::Matrix<double, 2, 1>::Zero();

    /// @brief Y坐标KF观测 [y] (1x1)
    Eigen::Matrix<double, 1, 1> armor_y_measurement = Eigen::Matrix<double, 1, 1>::Zero();

    /// @brief Z坐标KF状态 [z, vz] (2x1)
    Eigen::Matrix<double, 2, 1> armor_z_state = Eigen::Matrix<double, 2, 1>::Zero();

    /// @brief Z坐标KF观测 [z] (1x1)
    Eigen::Matrix<double, 1, 1> armor_z_measurement = Eigen::Matrix<double, 1, 1>::Zero();
};

/**
 * @brief   标准报文类，流水线各阶段之间传递的唯一数据包
 * @details 每个字段标注了写入方和读取方。只有写入方可以修改该字段。
 */
struct ThreadDataPack
{
    // ---- 流水线框架（common/、EntryStage 写入）----
    int index = 0;  ///< 报文序号。写：EntryStage；读：全部

    /// 各子模块处理结果。写：pipeline 框架（EntryStage 每帧重置为 NOTYET）；读：pipeline 框架
    std::array<SubModuleResult, SUBMODULE_COUNT> submodule_results;

    /// 各子模块开始/结束时间。写：pipeline 框架；读：EntryStage（耗时日志）
    std::array<std::pair<std::chrono::steady_clock::time_point, std::chrono::steady_clock::time_point>, SUBMODULE_COUNT> submodule_timestamps{};

    // ---- M1 感知前端（A）写入 ----
    cv::Mat frame;  ///< 原始图像，BGR。写：Sensor；读：M1、M2（角点精修）、调试显示

    /// 取图完成时刻（不是曝光时刻），单调时钟。写：Sensor；读：M3（延迟补偿）
    FrameClock::time_point time{};

    Attitude attitude;          ///< 取图时刻的云台 IMU 姿态。写：Sensor；读：M2（PnP）、M3
    RobotStatus robotstatus;    ///< 下位机状态。写：Sensor；读：M2（敌方颜色）、M3

    cv::Mat detect_input;              ///< 网络输入图像，RGB。写：Preprocess；读：Detect（M1 内部）
    DetectInputMap detect_input_map;   ///< 原图与网络输入的映射。写：Preprocess；读：Detect（M1 内部）

    /// 接口 I1：装甲板检测结果。写：Detect；M2 的 CornerRefine 可改写 pts 和 source；读：M3
    std::vector<bbox_t> bboxes;

    // ---- M3 决策（C）写入 ----
    Target target;                  ///< 跟踪结果。写：Predictor；读：Planner
    bool has_fixed_target = false;  ///< 是否锁定目标。写：Predictor；读：Planner
    RobotCommand robotcommand;      ///< 云台指令。写：Planner；读：TimedSerial（经消息桥）
};

#endif // COMMON_ROBOT_H
