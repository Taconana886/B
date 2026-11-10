//
// tools/replay：视频 → R1 入口（模块 B）
//
// 在 A-4 的实机 R1 数据到位之前，用 test.avi 生成结构合法的 R1，
// 用于验证回放工具和格式（bboxes 为空，因为没有检测器）。
//
// 用法：avi_to_r1 <video> <r1_dir> [--enemy-color 0|1|2]
//   - 图像输出为 <r1_dir>/images/%08d.png（BGR，无损）
//   - 记录输出为 <r1_dir>/records.jsonl（每帧一条）
//   - 姿态填 0（无 IMU），时间戳用当前单调时钟
//

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "datatype.hpp"
#include "json_min.hpp"

namespace
{
    namespace fs = std::filesystem;
    using namespace replay_json;
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "usage: avi_to_r1 <video> <r1_dir> [--enemy-color 0|1|2]\n";
        return 1;
    }
    const std::string video_path = argv[1];
    const std::string r1_dir = argv[2];
    int enemy_color = 2;
    for (int i = 3; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--enemy-color" && i + 1 < argc)
        {
            enemy_color = std::stoi(argv[++i]);
        }
    }

    try
    {
        cv::VideoCapture cap(video_path);
        if (!cap.isOpened())
        {
            std::cerr << "[avi_to_r1] cannot open video " << video_path << "\n";
            return 1;
        }

        fs::path images_dir = fs::path(r1_dir) / "images";
        fs::create_directories(images_dir);
        std::ofstream fout(fs::path(r1_dir) / "records.jsonl");
        if (!fout.is_open())
        {
            std::cerr << "[avi_to_r1] cannot create records.jsonl\n";
            return 1;
        }

        cv::Mat frame;
        int index = 0;
        while (cap.read(frame))
        {
            // 时间戳：取图完成时刻（单调时钟）
            const uint64_t time_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    FrameClock::now().time_since_epoch())
                    .count());

            char name_buf[64];
            std::snprintf(name_buf, sizeof(name_buf), "images/%08d.png", index);
            const std::string image_rel = name_buf;

            if (!cv::imwrite((fs::path(r1_dir) / image_rel).string(), frame))
            {
                std::cerr << "[avi_to_r1] cannot write image for frame " << index << "\n";
                return 1;
            }

            Object rec;
            rec.emplace_back("format_version", 1);
            rec.emplace_back("data_type_version", kDataTypeVersion);
            rec.emplace_back("index", index);
            rec.emplace_back("time_ns", time_ns);
            rec.emplace_back("attitude_deg",
                             Value(Object{{"yaw", 0.0}, {"pitch", 0.0}, {"roll", 0.0}}));
            rec.emplace_back("enemy_color", enemy_color);
            rec.emplace_back("image", image_rel);
            rec.emplace_back("image_width", frame.cols);
            rec.emplace_back("image_height", frame.rows);
            rec.emplace_back("bboxes", Value(Array{}));

            fout << Value(std::move(rec)).dump() << '\n';
            ++index;
        }

        fout.close();
        std::cout << "[avi_to_r1] wrote " << index << " frames to " << r1_dir << "\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[avi_to_r1] error: " << e.what() << "\n";
        return 1;
    }
}
