#pragma once

#include <opencv2/opencv.hpp>
#include "armor_detector/types.hpp"
#include <vector>

class ArmorFinder {
public:
    ArmorFinder();
    std::vector<Armor> detect(const cv::Mat &img);

    void preprocess(const cv::Mat &img, cv::Mat &binary);

    std::vector<Light> findLights(const cv::Mat &binary);
    std::vector<Armor> matchArmors(const std::vector<Light> &lights);


private:
    int enemy_color_;
};