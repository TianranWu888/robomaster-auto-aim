#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "armor_detector/types.hpp"

class Visualizer
{
public:
    Visualizer();

    // 画出完整调试信息
    void drawDebug(
    cv::Mat &img,
    const std::vector<Light> &lights,   
    const std::vector<Armor> &armors,
    int armor_count,
    const cv::Mat &binary
);


private:
    void drawLights(cv::Mat &img, const std::vector<Light> &lights);
    void drawArmors(cv::Mat &img, const std::vector<Armor> &armors);
};
