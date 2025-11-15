#include "armor_detector/armor_finder.hpp"
#include <algorithm>
#include <cmath>

ArmorFinder::ArmorFinder()
{
    enemy_color_ = 1;   // 1 = blue, 2 = red（默认红色)
}

/**
 * 主流程：输入原图 → 输出多个装甲板 Armor
 */
std::vector<Armor> ArmorFinder::detect(const cv::Mat &img)
{
    cv::Mat binary;
    preprocess(img, binary);

    auto lights = findLights(binary);

    auto armors = matchArmors(lights);

    return armors;
}

/**
 * Step 1: 二值化预处理
 */
void ArmorFinder::preprocess(const cv::Mat &img, cv::Mat &binary)
{
    cv::Mat blur_img;
    cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);

    // 分离 BGR
    std::vector<cv::Mat> ch;
    cv::split(blur_img, ch);

    // 敌方颜色=红色：R - B
    if (enemy_color_ == 2)
        cv::subtract(ch[2], ch[0], binary);
    else
        cv::subtract(ch[0], ch[2], binary);

    // Threshold
    cv::threshold(binary, binary, 150, 255, cv::THRESH_BINARY);

    // Morphology
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
}

/**
 * Step 2: 提取灯条（fitEllipse）
 */
std::vector<Light> ArmorFinder::findLights(const cv::Mat &binary)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Light> lights;

    for (auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < 10 || contour.size() < 5)
            continue;

        cv::RotatedRect light = cv::fitEllipse(contour);

        float w = light.size.width;
        float h = light.size.height;

        // 灯条基本筛选
        if (w / h > 0.8) continue;
        if (area / (w * h) < 0.5) continue;

        // 灯条角度必须竖直
        if (!(light.angle < 45 || light.angle > 135))
            continue;

        lights.push_back({light});
    }

    return lights;
}

/**
 * Step 3: 灯条两两配对成装甲板
 */
std::vector<Armor> ArmorFinder::matchArmors(const std::vector<Light> &lights)
{
    std::vector<Armor> output;
    if (lights.size() < 2) return output;

    // 按 x 从左到右排序
    auto sorted = lights;
    std::sort(sorted.begin(), sorted.end(),
              [](const Light &a, const Light &b){
                  return a.rect.center.x < b.rect.center.x;
              });

    for (size_t i = 0; i < sorted.size(); i++)
    {
        for (size_t j = i + 1; j < sorted.size(); j++)
        {
            const cv::RotatedRect &L = sorted[i].rect;
            const cv::RotatedRect &R = sorted[j].rect;

            float angleDiff = std::fabs(L.angle - R.angle);
            if (angleDiff > 10) continue;

            float meanH = (L.size.height + R.size.height) / 2.0f;

            // y 列不一致
            float yDiff = std::fabs(L.center.y - R.center.y);
            if (yDiff / meanH > 0.7) continue;

            // x 间距太小
            float xDiff = std::fabs(L.center.x - R.center.x);
            if (xDiff / meanH < 0.5) continue;

            // 两灯条距离
            float d = cv::norm(L.center - R.center);
            float ratio = d / meanH;
            if (ratio < 1 || ratio > 3) continue;

            // =============== 构造装甲板数据 ====================

            Armor armor;

            // 装甲中心
            armor.center = (L.center + R.center) * 0.5f;

            // 宽 = 两灯条中心距，高 = 平均灯条高
            armor.width  = d;
            armor.height = meanH;

            // 旋转矩形
            armor.box.center = armor.center;
            armor.box.size = cv::Size2f(armor.width, armor.height);
            armor.box.angle = (L.angle + R.angle) * 0.5f;

            // 角点
            armor.box.points(armor.corners);

            // edge_vec: PnP 重建需要的 2D projected edges
            armor.edge_vec.clear();
            armor.edge_vec.push_back(R.center - L.center);  // 水平边向量

            float rad = (L.angle + R.angle + 180) / 360.0f * CV_PI;
            armor.edge_vec.push_back(cv::Point2f(meanH * std::cos(rad),
                                                 meanH * std::sin(rad)));

            output.push_back(armor);
        }
    }

    return output;
}
