#include "armor_detector/visualizer.hpp"

Visualizer::Visualizer() {}

/**
 * 主调试接口：
 * - 画灯条
 * - 显示灯条数量
 * - 画装甲板
 * - 画装甲中心
 * - 显示装甲数量
 * - 显示二值图
 */
void Visualizer::drawDebug(
    cv::Mat &img,
    const std::vector<Light> &lights,    
    const std::vector<Armor> &armors,
    int armor_count,
    const cv::Mat &binary
) {
    // 画灯条
    drawLights(img, lights);

    // 显示灯条数量
    cv::putText(img,
                "Lights: " + std::to_string(lights.size()),
                cv::Point(30, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.9,
                cv::Scalar(100, 255, 100),
                2);

    // 画装甲板
    drawArmors(img, armors);

    // 显示装甲数量
    cv::putText(img,
                "Armors: " + std::to_string(armor_count),
                cv::Point(30, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 200, 255),
                2);

    // 显示窗口（调试）
    //cv::imshow("Binary", binary);
    //cv::imshow("Detected Armors", img);
    //cv::waitKey(1);
}

/**
 * 绘制灯条（绿色）
 */
void Visualizer::drawLights(cv::Mat &img, const std::vector<Light> &lights)
{
    for (auto &l : lights)
    {
        cv::Point2f pts[4];
        l.rect.points(pts);

        for (int i = 0; i < 4; i++)
            cv::line(img, pts[i], pts[(i+1)%4], cv::Scalar(0,255,0), 2);
    }
}

/**
 * 绘制装甲板（红框 + 中心点）
 */
void Visualizer::drawArmors(cv::Mat &img, const std::vector<Armor> &armors)
{
    for (auto &a : armors)
    {
        cv::Point2f pts[4];
        a.box.points(pts);

        for (int i = 0; i < 4; i++)
            cv::line(img, pts[i], pts[(i+1)%4], cv::Scalar(0,0,255), 2);

        cv::circle(img, a.center, 4, cv::Scalar(0,255,255), -1);
    }
}
