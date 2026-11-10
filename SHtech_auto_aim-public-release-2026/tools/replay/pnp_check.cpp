/**
 * @file pnp_check.cpp
 * @brief B-4 验收③辅助工具：单张图片人工点选装甲板四角，用与在线完全相同的
 *        CoordTransformer::pnp_get_measurement 计算距离，与卷尺实测值对比。
 *
 * 用法：
 *   pnp_check <image> <cam_param.yml> --tag 1 --true-distance 2.0 [--color 0] [--points pts.txt]
 *
 * - 角点顺序：左上、左下、右下、右上（I2 约定）；
 * - 不传 --points 时打开窗口鼠标点选（r 重选，q/ESC 退出）；
 * - --points 文件：4 行，每行 "x y"（像素坐标），用于无显示环境；
 * - 距离从云台旋转点量起（与 yml 中 T_c2i 语义一致），单位为米；
 * - 姿态按 yaw=0、R_world2imu=单位阵处理，输出即为 IMU/云台系坐标；
 * - 判定：水平距离误差 = |测量 - 实测| / 实测，B-4 要求 < 2%。
 */

#include "CoordTransformer.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace
{
    struct ClickState
    {
        cv::Mat image;
        std::vector<cv::Point2f> points;
    };

    void on_mouse(int event, int x, int y, int, void *userdata)
    {
        if (event != cv::EVENT_LBUTTONDOWN)
            return;
        auto *state = static_cast<ClickState *>(userdata);
        if (state->points.size() >= 4)
            return;
        state->points.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }

    void draw_state(const ClickState &state)
    {
        cv::Mat canvas = state.image.clone();
        for (size_t i = 0; i < state.points.size(); ++i)
        {
            cv::circle(canvas, state.points[i], 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(canvas, std::to_string(i), state.points[i] + cv::Point2f(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            if (i > 0)
                cv::line(canvas, state.points[i - 1], state.points[i], cv::Scalar(0, 255, 0), 1);
        }
        if (state.points.size() == 4)
            cv::line(canvas, state.points[3], state.points[0], cv::Scalar(0, 255, 0), 1);
        cv::imshow("pnp_check", canvas);
    }

    bool load_points(const std::string &path, std::vector<cv::Point2f> &out)
    {
        std::ifstream fin(path);
        if (!fin)
            return false;
        float x = 0.0f;
        float y = 0.0f;
        while (fin >> x >> y)
            out.emplace_back(x, y);
        return out.size() == 4;
    }

    void perform_pnp_check(mathutils::CoordTransformer &transformer,
                           const std::vector<cv::Point2f> &points,
                           int tag_id, int color_id, double true_distance)
    {
        cv::Point2f p[4] = {points[0], points[1], points[2], points[3]};
        float yaw_in_camera = 0.0f;
        Eigen::Vector4d measurement = Eigen::Vector4d::Zero();
        const Eigen::Matrix3d R_world2imu = Eigen::Matrix3d::Identity();

        const bool ok = transformer.pnp_get_measurement(
            p, tag_id, color_id, 0.0f, R_world2imu, yaw_in_camera, measurement);
        if (!ok)
        {
            std::printf("[pnp_check] PnP failed\n");
            return;
        }

        const double y = measurement(0, 0);
        const double x = measurement(1, 0);
        const double z = measurement(2, 0);
        const double yaw_abs = measurement(3, 0);
        const double horizontal = std::hypot(x, y);
        const double los = measurement.head<3>().norm();

        std::printf("[pnp_check] measurement: y=%.4f x=%.4f z=%.4f (m)\n", y, x, z);
        std::printf("[pnp_check] yaw_in_camera=%.2f deg, absolute_yaw=%.2f deg\n",
                    yaw_in_camera * 180.0f / static_cast<float>(M_PI),
                    yaw_abs * 180.0 / M_PI);
        std::printf("[pnp_check] horizontal distance=%.4f m, line-of-sight=%.4f m\n",
                    horizontal, los);
        if (true_distance > 0.0)
        {
            const double err = std::abs(horizontal - true_distance) / true_distance * 100.0;
            std::printf("[pnp_check] true=%.4f m, horizontal error=%.2f%% (pass if < 2%%)\n",
                        true_distance, err);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        std::printf("usage: %s <image> <cam_param.yml> --tag N --true-distance M"
                    " [--color 0] [--points pts.txt]\n", argv[0]);
        return 1;
    }

    const std::string image_path = argv[1];
    const std::string yml_path = argv[2];
    int tag_id = 1;
    int color_id = 0;
    double true_distance = -1.0;
    std::string points_path;

    for (int i = 3; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--tag" && i + 1 < argc)
            tag_id = std::stoi(argv[++i]);
        else if (arg == "--color" && i + 1 < argc)
            color_id = std::stoi(argv[++i]);
        else if (arg == "--true-distance" && i + 1 < argc)
            true_distance = std::stod(argv[++i]);
        else if (arg == "--points" && i + 1 < argc)
            points_path = argv[++i];
        else
        {
            std::printf("unknown argument: %s\n", arg.c_str());
            return 1;
        }
    }

    const cv::Mat image = cv::imread(image_path);
    if (image.empty())
    {
        std::printf("cannot read image: %s\n", image_path.c_str());
        return 1;
    }

    mathutils::CoordTransformer::Init(yml_path, false);
    mathutils::CoordTransformer &transformer = mathutils::CoordTransformer::Get();

    std::vector<cv::Point2f> points;
    if (!points_path.empty())
    {
        if (!load_points(points_path, points))
        {
            std::printf("cannot load 4 points from %s (4 lines 'x y')\n", points_path.c_str());
            return 1;
        }
        perform_pnp_check(transformer, points, tag_id, color_id, true_distance);
        return 0;
    }

    ClickState state{image, {}};
    cv::namedWindow("pnp_check", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("pnp_check", on_mouse, &state);
    std::printf("click 4 corners: top-left, bottom-left, bottom-right, top-right;"
                " r=reset q=quit\n");

    bool done = false;
    while (true)
    {
        draw_state(state);
        const int key = cv::waitKey(20) & 0xFF;
        if (key == 'q' || key == 27)
            break;
        if (key == 'r')
        {
            state.points.clear();
            done = false;
            continue;
        }
        if (!done && state.points.size() == 4)
        {
            perform_pnp_check(transformer, state.points, tag_id, color_id, true_distance);
            done = true;
        }
    }
    return 0;
}
