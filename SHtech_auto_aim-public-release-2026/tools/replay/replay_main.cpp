//
// tools/replay：R1 → R2 回放工具（模块 B）
//
// 读取 R1（records.jsonl + 图像），调用与在线版本相同的
//   - detect::optimizeCornersWithEnemyColor（角点精修，含颜色未知双阈值策略）
//   - mathutils::CoordTransformer::pnp_get_measurement（PnP + 双解消歧）
// 输出 R2（records.jsonl + metrics.json + timing.json）。
//
// 确定性保证（B-2 验收①）：
//   - 单线程执行，不引入 OpenMP/多线程非确定性
//   - 浮点数序列化使用 std::to_chars（最短往返、与 locale 无关）
//   - records.jsonl / metrics.json 逐字节一致；timing.json 依赖时钟，仅供性能参考
//
// 用法：replay_tool <r1_dir> <r2_dir> <cam_param.yml> [--no-refine]
//

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "CoordTransformer.hpp"
#include "corner_refine_submodule.hpp"
#include "json_min.hpp"

namespace
{
    namespace fs = std::filesystem;
    using namespace replay_json;

    constexpr int kR1FormatVersion = 1;

    double ms_between(const std::chrono::steady_clock::time_point& a,
                      const std::chrono::steady_clock::time_point& b)
    {
        return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(b - a).count();
    }

    // 最近秩百分位数（nearest-rank），n >= 1
    double percentile(const std::vector<double>& sorted_values, double q)
    {
        if (sorted_values.empty()) return 0.0;
        const size_t idx = static_cast<size_t>((sorted_values.size() - 1) * q);
        return sorted_values[idx];
    }

    struct FrameRecord
    {
        int index = 0;
        uint64_t time_ns = 0;
        double yaw_deg = 0.0, pitch_deg = 0.0, roll_deg = 0.0;
        int enemy_color = 2;
        std::string image;
        struct BBoxRec
        {
            std::array<cv::Point2f, 4> pts{};
            int color_id = 2;
            int tag_id = 0;
        };
        std::vector<BBoxRec> bboxes;
    };

    FrameRecord parse_record(const Value& rec)
    {
        FrameRecord out;
        out.index = rec.at("index").as_int();
        out.time_ns = rec.at("time_ns").as_uint64();
        out.enemy_color = rec.at("enemy_color").as_int();

        const Value& att = rec.at("attitude_deg");
        out.yaw_deg = att.at("yaw").as_double();
        out.pitch_deg = att.at("pitch").as_double();
        out.roll_deg = att.at("roll").as_double();

        out.image = rec.at("image").as_string();

        const Value& bboxes = rec.at("bboxes");
        for (const auto& bv : bboxes.as_array())
        {
            const Value& pts = bv.at("pts");
            const auto& pts_arr = pts.as_array();
            if (pts_arr.size() != 4)
            {
                throw std::runtime_error("bbox pts must have 4 corners");
            }
            FrameRecord::BBoxRec b;
            for (size_t i = 0; i < 4; ++i)
            {
                const auto& c = pts_arr[i].as_array();
                if (c.size() != 2)
                {
                    throw std::runtime_error("corner must be [x, y]");
                }
                b.pts[i] = cv::Point2f(static_cast<float>(c[0].as_double()),
                                       static_cast<float>(c[1].as_double()));
            }
            b.color_id = bv.at("color_id").as_int();
            b.tag_id = bv.at("tag_id").as_int();
            out.bboxes.push_back(b);
        }
        return out;
    }

    Value pts_to_json(const cv::Point2f (&pts)[4])
    {
        Array arr;
        for (const auto& p : pts)
        {
            arr.emplace_back(Array{Value(static_cast<double>(p.x)), Value(static_cast<double>(p.y))});
        }
        return Value(std::move(arr));
    }

    Value mat3_to_json(const Eigen::Matrix3d& m)
    {
        Array arr;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                arr.emplace_back(m(r, c));
            }
        }
        return Value(std::move(arr));
    }

    struct RunStats
    {
        int64_t frame_count = 0;
        int64_t bbox_count = 0;
        int64_t refine_success = 0;
        int64_t refine_fail = 0;
        int64_t pnp_success = 0;
        int64_t pnp_fail = 0;
        std::vector<double> refine_ms;
        std::vector<double> pnp_ms;
    };
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr << "usage: replay_tool <r1_dir> <r2_dir> <cam_param.yml> [--no-refine]\n";
        return 1;
    }
    const std::string r1_dir = argv[1];
    const std::string r2_dir = argv[2];
    const std::string cam_param = argv[3];
    bool enable_refine = true;
    for (int i = 4; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--no-refine") enable_refine = false;
    }

    try
    {
        // ---- 初始化（与在线 main.cpp 相同） ----
        mathutils::CoordTransformer::Init(cam_param, false);
        auto& transformer = mathutils::CoordTransformer::Get();
        detect::ArmorCornerOptimizer optimizer(false);

        const fs::path r1_records = fs::path(r1_dir) / "records.jsonl";
        std::ifstream fin(r1_records);
        if (!fin.is_open())
        {
            std::cerr << "[replay] cannot open " << r1_records << "\n";
            return 1;
        }

        fs::create_directories(r2_dir);
        std::ofstream fout(fs::path(r2_dir) / "records.jsonl");
        if (!fout.is_open())
        {
            std::cerr << "[replay] cannot create output file\n";
            return 1;
        }

        RunStats stats;
        std::string line;
        size_t line_no = 0;

        while (std::getline(fin, line))
        {
            ++line_no;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            const Value rec = Value::parse(line);
            const int data_type_version = rec.at("data_type_version").as_int();
            if (data_type_version != kDataTypeVersion)
            {
                std::cerr << "[replay] line " << line_no
                          << ": data_type_version " << data_type_version
                          << " != " << kDataTypeVersion << "\n";
                return 1;
            }
            const int format_version = rec.at("format_version").as_int();
            if (format_version != kR1FormatVersion)
            {
                std::cerr << "[replay] line " << line_no
                          << ": unsupported R1 format_version " << format_version << "\n";
                return 1;
            }

            const FrameRecord frame = parse_record(rec);
            stats.frame_count++;

            // 读取图像
            const fs::path img_path = fs::path(r1_dir) / frame.image;
            cv::Mat img = cv::imread(img_path.string(), cv::IMREAD_COLOR);
            if (img.empty())
            {
                std::cerr << "[replay] line " << line_no << ": cannot read image " << img_path << "\n";
                return 1;
            }

            // 姿态（与 MultiPolicyPredictor 相同：yaw 度 → 弧度）
            const Attitude attitude(static_cast<float>(frame.yaw_deg),
                                    static_cast<float>(frame.pitch_deg),
                                    static_cast<float>(frame.roll_deg));
            const Eigen::Matrix3d R_world2imu = attitude.R_world2imu();
            const float attitude_yaw = static_cast<float>(frame.yaw_deg / 180.0 * M_PI);

            // 逐块装甲板处理
            Array measurements;
            for (const auto& bbox : frame.bboxes)
            {
                stats.bbox_count++;

                cv::Point2f pts[4];
                std::copy(bbox.pts.begin(), bbox.pts.end(), pts);
                int source = 0;  // NEURAL_NETWORK

                // ---- 角点精修（与在线 CornerRefineSubModule 共用实现） ----
                if (enable_refine)
                {
                    const auto t0 = std::chrono::steady_clock::now();
                    detect::CornerRefineCallStats refine_stats;
                    int used_threshold = -1;
                    auto refined = detect::optimizeCornersWithEnemyColor(
                        optimizer, img, pts,
                        static_cast<EnemyColor>(frame.enemy_color),
                        false, refine_stats, used_threshold);
                    stats.refine_ms.push_back(ms_between(t0, std::chrono::steady_clock::now()));

                    if (refined.has_value())
                    {
                        std::copy(refined->begin(), refined->end(), pts);
                        source = 1;  // TRADITIONAL
                        stats.refine_success++;
                    }
                    else
                    {
                        stats.refine_fail++;
                    }
                }

                // ---- PnP（与在线 MultiPolicyPredictor 共用实现） ----
                float yaw_in_camera = 0.0f;
                Eigen::Vector4d measurement = Eigen::Vector4d::Zero();
                const auto t0 = std::chrono::steady_clock::now();
                const bool pnp_ok = transformer.pnp_get_measurement(
                    pts, bbox.tag_id, bbox.color_id,
                    attitude_yaw, R_world2imu, yaw_in_camera, measurement);
                stats.pnp_ms.push_back(ms_between(t0, std::chrono::steady_clock::now()));

                if (pnp_ok)
                {
                    stats.pnp_success++;
                }
                else
                {
                    stats.pnp_fail++;
                }

                Object m;
                m.emplace_back("tag_id", bbox.tag_id);
                m.emplace_back("color_id", bbox.color_id);
                m.emplace_back("source", source);
                m.emplace_back("pts", pts_to_json(pts));
                m.emplace_back("pnp_success", pnp_ok);
                if (pnp_ok)
                {
                    m.emplace_back("measurement", Array{Value(measurement(0, 0)),
                                                        Value(measurement(1, 0)),
                                                        Value(measurement(2, 0)),
                                                        Value(measurement(3, 0))});
                    m.emplace_back("yaw_in_camera", static_cast<double>(yaw_in_camera));
                }
                else
                {
                    m.emplace_back("measurement", Array{Value(nullptr), Value(nullptr),
                                                        Value(nullptr), Value(nullptr)});
                    m.emplace_back("yaw_in_camera", Value(nullptr));
                }
                measurements.emplace_back(std::move(m));
            }

            // ---- 写 R2 记录（键序固定，保证逐字节一致） ----
            Object frame_json;
            frame_json.emplace_back("format_version", 1);
            frame_json.emplace_back("data_type_version", kDataTypeVersion);
            frame_json.emplace_back("index", frame.index);
            frame_json.emplace_back("time_ns", frame.time_ns);
            frame_json.emplace_back(
                "attitude_deg",
                Value(Object{{"yaw", frame.yaw_deg},
                             {"pitch", frame.pitch_deg},
                             {"roll", frame.roll_deg}}));
            frame_json.emplace_back("enemy_color", frame.enemy_color);
            frame_json.emplace_back("R_world2imu", mat3_to_json(R_world2imu));
            frame_json.emplace_back("measurements", Value(std::move(measurements)));

            fout << Value(std::move(frame_json)).dump() << '\n';
        }

        fin.close();
        fout.close();

        // ---- metrics.json（确定性统计） ----
        Object metrics;
        metrics.emplace_back("format_version", 1);
        metrics.emplace_back("data_type_version", kDataTypeVersion);
        metrics.emplace_back("frame_count", stats.frame_count);
        metrics.emplace_back("bbox_count", stats.bbox_count);
        metrics.emplace_back("refine_success_count", stats.refine_success);
        metrics.emplace_back("refine_fail_count", stats.refine_fail);
        metrics.emplace_back("pnp_success_count", stats.pnp_success);
        metrics.emplace_back("pnp_fail_count", stats.pnp_fail);
        {
            std::ofstream mf(fs::path(r2_dir) / "metrics.json");
            mf << Value(std::move(metrics)).dump() << '\n';
        }

        // ---- timing.json（非确定性，仅供性能参考） ----
        std::sort(stats.refine_ms.begin(), stats.refine_ms.end());
        std::sort(stats.pnp_ms.begin(), stats.pnp_ms.end());
        Object timing;
        timing.emplace_back("format_version", 1);
        timing.emplace_back("refine_ms_p50", percentile(stats.refine_ms, 0.50));
        timing.emplace_back("refine_ms_p99", percentile(stats.refine_ms, 0.99));
        timing.emplace_back("pnp_ms_p50", percentile(stats.pnp_ms, 0.50));
        timing.emplace_back("pnp_ms_p99", percentile(stats.pnp_ms, 0.99));
        {
            std::ofstream tf(fs::path(r2_dir) / "timing.json");
            tf << Value(std::move(timing)).dump() << '\n';
        }

        std::cout << "[replay] done: " << stats.frame_count << " frames, "
                  << stats.bbox_count << " bboxes; refine "
                  << stats.refine_success << "/" << stats.refine_fail
                  << " ok/fail; pnp "
                  << stats.pnp_success << "/" << stats.pnp_fail
                  << " ok/fail\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[replay] error: " << e.what() << "\n";
        return 1;
    }
}
