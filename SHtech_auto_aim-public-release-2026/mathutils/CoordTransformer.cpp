/**
 * @file CoordTransformer.cpp
 * @brief 坐标变换模块实现 - 处理相机、IMU和世界坐标系之间的转换
 * @author Cao Jingyan
 * @date 2025/11/15
 */

#include "CoordTransformer.hpp"

#include <algorithm>
#include <limits>

namespace mathutils
{
    // 定义静态成员变量
    std::unique_ptr<CoordTransformer> CoordTransformer::instance_ = nullptr;

    /**
     * @brief 默认构造函数 - 初始化所有变换矩阵为零矩阵
     */
    CoordTransformer::CoordTransformer(bool adjust_)
    {
        // 初始化相机到IMU的平移向量 (3x1)
        T_camera2imu_MAT = cv::Mat::zeros(3, 1, CV_64FC1);
        
        // 初始化相机到IMU的旋转矩阵 (3x3)
        R_camera2imu_MAT = cv::Mat::zeros(3, 3, CV_64FC1);
        
        // 初始化相机内参矩阵 (3x3)
        F_MAT = cv::Mat::zeros(3, 3, CV_64FC1);
        
        // 初始化相机畸变参数 (1x5)
        C_MAT = cv::Mat::zeros(1, 5, CV_64FC1);
        
        // 将OpenCV矩阵转换为Eigen矩阵，便于后续数学运算
        cv::cv2eigen(T_camera2imu_MAT, T_camera2imu);
        cv::cv2eigen(R_camera2imu_MAT, R_camera2imu);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        adjust = adjust_;

        if (adjust) {
            cv::namedWindow("transformer trackbar", cv::WINDOW_AUTOSIZE);
        
            cv::createTrackbar("pw_length", "transformer trackbar", &pw_length, 250, 0);
            cv::createTrackbar("pw_width", "transformer trackbar", &pw_width, 250, 0);
        }
    }

    /**
     * @brief 带参数构造函数 - 从yml文件加载相机参数
     * @param camera_param yml文件路径
     * @details 从YAML文件中读取相机内参、畸变参数和相机-IMU外参
     */
    CoordTransformer::CoordTransformer(const std::string camera_param, bool adjust_)
    {
        // 打开相机参数文件
        cv::FileStorage fin(camera_param, cv::FileStorage::READ);
        
        // 读取相机到IMU的外参（平移向量和旋转矩阵）
        fin["T_c2i"] >> T_camera2imu_MAT;  // 相机到IMU的平移向量
        fin["R_c2i"] >> R_camera2imu_MAT;  // 相机到IMU的旋转矩阵

        // 读取相机内参
        fin["K"] >> F_MAT;  // 相机内参矩阵 (焦距、主点等)
        fin["D"] >> C_MAT;  // 相机畸变参数 (径向畸变、切向畸变)

        // B-6：可选读取 PnP 消歧/精修开关（缺省使用类内默认值 true）
        cv::FileNode node = fin["pnp_disambiguate"];
        if (!node.empty()) pnp_disambiguate_ = (static_cast<int>(node) != 0);
        node = fin["pnp_refine_lm"];
        if (!node.empty()) pnp_refine_lm_ = (static_cast<int>(node) != 0);

        // 将OpenCV格式转换为Eigen格式，便于数学运算
        cv::cv2eigen(T_camera2imu_MAT, T_camera2imu);
        cv::cv2eigen(R_camera2imu_MAT, R_camera2imu);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        adjust = adjust_;

        if (adjust) {
            cv::namedWindow("transformer trackbar", cv::WINDOW_AUTOSIZE);
        
            cv::createTrackbar("pw_length", "transformer trackbar", &pw_length, 250, 0);
            cv::createTrackbar("pw_width", "transformer trackbar", &pw_width, 250, 0);
        }
    }

    /**
     * @brief 显式初始化单例
     */
    void CoordTransformer::Init(const std::string& param_file, bool adjust)
    {
        if (!instance_) {
            instance_ = std::unique_ptr<CoordTransformer>(new CoordTransformer(param_file, adjust));
        }
    }

    /**
     * @brief 获取单例实例
     */
    CoordTransformer& CoordTransformer::Get()
    {
        if (!instance_) {
            // 防御性编程：如果未初始化就调用，直接报错终止
            std::cerr << "[FATAL] CoordTransformer accessed before Init!" << std::endl;
            std::abort();
        }
        return *instance_;
    }

    /**
     * @brief 销毁单例
     */
    void CoordTransformer::Destroy()
    {
        instance_.reset();
    }

    /**
     * @brief 由 PnP 的 rvec/tvec 计算装甲板法向量在相机系的朝向角
     */
    double CoordTransformer::compute_yaw_in_camera(const std::vector<cv::Point3d> &pw_cur,
                                                   const cv::Mat &rvec, const cv::Mat &tvec)
    {
        cv::Mat mat_R;
        Eigen::Matrix3d R;
        cv::Rodrigues(rvec, mat_R);  // 旋转向量转旋转矩阵
        cv::cv2eigen(mat_R, R);

        Pos3D pc;
        cv::cv2eigen(tvec, pc);  // PnP得到的是相机坐标系中的位置

        // 计算每个角点的相机系3D坐标
        Pos3D p_a_c[4];
        for (int i = 0; i < 4; ++i)
        {
            Pos3D temp(pw_cur[i].x, pw_cur[i].y, pw_cur[i].z);  // 装甲板模型坐标
            p_a_c[i] = R * temp + pc;  // 变换到相机坐标系
        }

        // 计算装甲板在相机坐标系中的方向向量
        Pos3D armor_v_x_c, armor_v_y_c, armor_v_z_c;
        armor_v_x_c = (p_a_c[0] - p_a_c[3]) + (p_a_c[1] - p_a_c[2]);
        armor_v_y_c = (p_a_c[1] - p_a_c[0]) + (p_a_c[2] - p_a_c[3]);
        armor_v_z_c = armor_v_x_c.cross(armor_v_y_c);

        // 计算装甲板在相机坐标系中的偏航角
        // 偏航轴是负Z轴，因此使用-armor_v_z_c[2]
        // 正对装甲板的yaw_in_camera=0，绕世界坐标系z轴正方向旋转为负，相反为正
        return atan2(armor_v_z_c[0], -armor_v_z_c[2]);
    }

    /**
     * @brief PnP算法获取装甲板测量值
     */
    bool CoordTransformer::pnp_get_measurement(const cv::Point2f (&p)[4], const int &armor_number, 
                                                const int &color_id, const float &attitude_yaw, 
                                                const Eigen::Matrix3d &R_world2imu,
                                                float &yaw_in_camera, Eigen::Vector4d &measurement)
    {
        // 根据装甲板编号选择对应的3D模型尺寸
        std::vector<cv::Point3d> pw_cur;
        if (adjust) {
            double pw_x = pw_length / 1000.0 / 2.0;
            double pw_y = pw_width / 1000.0 / 2.0;

            std::vector<cv::Point3d> pw_temp = {
                {-pw_x, -pw_y, 0.},  // 左上角点
                {-pw_x, pw_y, 0.},   // 左下角点
                {pw_x, pw_y, 0.},    // 右下角点
                {pw_x, -pw_y, 0.}};  // 右上角点

            pw_cur = pw_temp;
        }
        else {
            // B-7：大装甲板判定集中定义为 kLargeArmorTagIds（与 docs/module-contracts.md 对照表一致）
            const bool is_large = std::find(kLargeArmorTagIds.begin(), kLargeArmorTagIds.end(), armor_number)
                                  != kLargeArmorTagIds.end();
            if (color_id == 0) {
                pw_cur = is_large ? pw_red_big : pw_red_small;
            }
            else {
                pw_cur = is_large ? pw_blue_big : pw_blue_small;
            }
        }

        // for (int i = 0; i < 4; ++i) {
        //     std::cout << p[i] << std::endl;
        // }

        // std::cout << "pw_cur: " << std::endl;
        // for (const auto &pt : pw_cur) {
        //     std::cout << pt << std::endl;
        // }

        // std::cout << "F_MAT: " << std::endl << F_MAT << std::endl;
        // std::cout << "C_MAT: " << std::endl << C_MAT << std::endl;

        // 将图像点转换为PnP算法需要的格式
        std::vector<cv::Point2d> pu(p, p + 4);
        // cv::Mat pu(4, 1, CV_32FC2, const_cast<cv::Point2f*>(p));

        // === PnP求解（B-6：IPPE 双解消歧 + 可选 LM 精修） ===
        cv::Mat rvec, tvec;  // 旋转向量和平移向量
        bool success = false;

        if (pnp_disambiguate_) {
            // solvePnPGeneric 对平面四点返回两个候选解（IPPE 双解歧义）
            std::vector<cv::Mat> rvecs, tvecs;
            cv::solvePnPGeneric(pw_cur, pu, F_MAT, C_MAT, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);

            if (rvecs.size() >= 2 && tvecs.size() >= 2) {
                // 选解策略：有上一帧朝向时选朝向最接近的解；否则取第一个解
                // （OpenCV 返回顺序：第一个解为主解）
                int chosen = 0;
                if (has_prev_yaw_) {
                    double best_diff = std::numeric_limits<double>::max();
                    for (size_t i = 0; i < rvecs.size(); ++i) {
                        const double yaw = compute_yaw_in_camera(pw_cur, rvecs[i], tvecs[i]);
                        const double diff = std::abs(mathutils::limit_rad(yaw - prev_yaw_in_camera_));
                        if (diff < best_diff) {
                            best_diff = diff;
                            chosen = static_cast<int>(i);
                        }
                    }
                }
                rvec = rvecs[chosen];
                tvec = tvecs[chosen];
                success = true;

                // 可选 LM 精修（带护栏）。
                // 合成数据实测（2026-10-30）：在近退化构型（对称角点、接近零空间）
                // 下 solvePnPRefineLM 可能沿退化方向漂移到另一个等误差解
                // （实测距离 2m → 3.25m、朝向翻转 180°）。因此只在精修后
                // ①重投影误差不显著变差 且 ②朝向未突变（<20°，未跳槽到另一解）
                // 时才接受精修结果，否则保留所选解（对应 B-6 验收③）。
                // 注：正常正面朝向的数据（检测器角点约定）下 sol0 即真解、
                // 精修稳定；护栏为退化构型的防御性措施。
                if (pnp_refine_lm_) {
                    const double yaw_before = compute_yaw_in_camera(pw_cur, rvec, tvec);
                    std::vector<cv::Point2d> projected;
                    cv::projectPoints(pw_cur, rvec, tvec, F_MAT, C_MAT, projected);
                    double err_before = 0.0;
                    for (size_t i = 0; i < pu.size(); ++i)
                        err_before += cv::norm(pu[i] - projected[i]);
                    err_before /= static_cast<double>(pu.size());

                    cv::Mat rvec_refined = rvec.clone();
                    cv::Mat tvec_refined = tvec.clone();
                    cv::solvePnPRefineLM(pw_cur, pu, F_MAT, C_MAT, rvec_refined, tvec_refined);

                    cv::projectPoints(pw_cur, rvec_refined, tvec_refined, F_MAT, C_MAT, projected);
                    double err_after = 0.0;
                    for (size_t i = 0; i < pu.size(); ++i)
                        err_after += cv::norm(pu[i] - projected[i]);
                    err_after /= static_cast<double>(pu.size());

                    const double yaw_after = compute_yaw_in_camera(pw_cur, rvec_refined, tvec_refined);
                    const double yaw_jump = std::abs(mathutils::limit_rad(yaw_after - yaw_before));
                    constexpr double kMaxRefineYawJump = 0.3490658503988659;  // 20°

                    if (err_after <= err_before * 1.05 && yaw_jump < kMaxRefineYawJump) {
                        rvec = rvec_refined;
                        tvec = tvec_refined;
                    }
                }
            }
        }

        if (!success) {
            // 回退：单解 IPPE（与旧实现行为一致）
            success = cv::solvePnP(pw_cur, pu, F_MAT, C_MAT, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        }

        if (!success) {
            return false;
        }
        
        // === 坐标变换过程 ===
        // 获取装甲板中心在不同坐标系中的位置
        Pos3D pc, pw, pi;  // 相机坐标系、世界坐标系、IMU坐标系
        
        cv::cv2eigen(tvec, pc);  // PnP得到的是相机坐标系中的位置

        // std::cout << "T_camera2imu: " << T_camera2imu << std::endl;
        // std::cout << "R_camera2imu: " << R_camera2imu << std::endl;
        // std::cout << "R_world2imu: " << R_world2imu << std::endl;
        
        // 相机坐标系 -> IMU坐标系
        pi = R_camera2imu * pc + T_camera2imu;

        // IMU坐标系 -> 世界坐标系
        pw = R_world2imu.transpose() * pi;

        // === 计算装甲板朝向（与选解时使用同一实现，保证一致性） ===
        yaw_in_camera = static_cast<float>(compute_yaw_in_camera(pw_cur, rvec, tvec));

        // 记录本帧朝向供下一帧双解选解
        // （调用方为 stage2 单线程 / 回放工具单线程，无并发访问）
        prev_yaw_in_camera_ = yaw_in_camera;
        has_prev_yaw_ = true;

        if (adjust)
            std::cout << pc[1] << std::endl << pc[0] << std::endl << pc[2] << std::endl << yaw_in_camera << std::endl;

        // === 组装最终测量值 ===
        // 测量值格式：[y坐标, x坐标, z坐标, 绝对偏航角]，该测量值对应于ekf的测量项
        // 绝对偏航角 = 相机坐标系中的偏航角 - 机器人姿态偏航角， [-pi, pi]
        measurement << pw[1], pw[0], pw[2], mathutils::limit_rad(yaw_in_camera - attitude_yaw);
        // measurement << pw[1], pw[0], pw[2], yaw_in_camera - attitude_yaw;

        return true;
    }
}

