#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct Light {
    cv::RotatedRect rect;
};

struct Armor {
    cv::Point2f center;
    cv::Point2f corners[4];
    cv::RotatedRect box;          // 旋转矩形
    float width;                  // 宽
    float height;                 // 高
    std::vector<cv::Point2f> edge_vec;  // edge_vectors[0], edge_vectors[1]
};