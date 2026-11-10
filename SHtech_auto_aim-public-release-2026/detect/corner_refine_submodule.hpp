//
// CornerRefineSubModule
// Refines detected armor corners using traditional computer vision.
//

#ifndef DETECT_CORNER_REFINE_SUBMODULE_H
#define DETECT_CORNER_REFINE_SUBMODULE_H

#include <limits>
#include <optional>

#include "armor_corner_optimizer.hpp"
#include "common.hpp"

namespace detect
{
    const int bin_threshold_for_blue = 70;
    const int bin_threshold_for_red = 60;

    struct CornerRefineConfig : pipeline::ModuleConfig
    {
        bool adjust_threshold = false;
    };

    /**
     * @brief 按敌方颜色执行角点精修（在线 CornerRefineSubModule 与 tools/replay 共用同一实现，B-5）
     * @param optimizer 已构造的 ArmorCornerOptimizer（内部阈值会被本函数设置）
     * @param frame 原始 BGR 图像
     * @param yolo_corners 神经网络角点（左上、左下、右下、右上）
     * @param enemy_color 敌方颜色（0 红、1 蓝、2 灰/未知）
     * @param show_image 是否弹出调试窗口（仅在线调试模式使用，回放传 false）
     * @param[out] used_threshold 实际采用的二值化阈值（-1 表示精修失败未采用）
     * @param[out] stats_out 采用尝试的耗时与几何统计（match_cost 越小几何拟合越好）
     * @return 精修后的角点；失败返回 nullopt（调用方保留神经网络角点）
     * @details 颜色未知（GRAY）时的策略：红蓝双阈值各跑一次，选择通过最终几何校验
     *          （灯条长度比、角度一致性）且 match_cost 更小的结果；
     *          两次都失败则返回 nullopt。策略依据见 docs/module-contracts.md I1 与 B-5 实验记录。
     */
    inline std::optional<std::array<cv::Point2f, 4>> optimizeCornersWithEnemyColor(
        ArmorCornerOptimizer &optimizer,
        const cv::Mat &frame,
        const cv::Point2f yolo_corners[4],
        EnemyColor enemy_color,
        bool show_image,
        CornerRefineCallStats &stats_out,
        int &used_threshold)
    {
        std::optional<std::array<cv::Point2f, 4>> best_corners;
        CornerRefineCallStats best_stats;
        double best_cost = std::numeric_limits<double>::infinity();
        int best_threshold = -1;

        auto try_threshold = [&](int threshold)
        {
            optimizer.setBinaryThreshold(threshold);
            CornerRefineCallStats stats;
            auto corners = optimizer.optimizeCorners(frame, yolo_corners, show_image, &stats);
            if (!stats.final_check_passed)
            {
                return;
            }
            if (best_corners.has_value() && stats.match_cost >= best_cost)
            {
                return;
            }
            best_corners = corners;
            best_stats = stats;
            best_cost = stats.match_cost;
            best_threshold = threshold;
        };

        if (enemy_color == EnemyColor::RED)
        {
            try_threshold(bin_threshold_for_red);
        }
        else if (enemy_color == EnemyColor::BLUE)
        {
            try_threshold(bin_threshold_for_blue);
        }
        else
        {
            // 颜色未知：红蓝双阈值各跑一次按几何置信度选择（B-F4/B-5）
            try_threshold(bin_threshold_for_red);
            try_threshold(bin_threshold_for_blue);
        }

        stats_out = best_stats;
        used_threshold = best_threshold;
        return best_corners;
    }

    class CornerRefineSubModule : public pipeline::SubModule
    {
    public:
        explicit CornerRefineSubModule(const CornerRefineConfig& config);
        virtual ~CornerRefineSubModule() = default;

        SubModuleResult process(std::shared_ptr<ThreadDataPack> data,
                                const pipeline::BasicTask* parent) override;

    private:
        CornerRefineConfig config_;
        ArmorCornerOptimizer corner_optimizer;
        int binary_thres = 94;
    };
}

#endif // DETECT_CORNER_REFINE_SUBMODULE_H
