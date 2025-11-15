#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "armor_detector/armor_finder.hpp"
#include "armor_detector/visualizer.hpp"

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    image_transport::Subscriber img_sub_;
    image_transport::Publisher  debug_img_pub_;

    ArmorFinder armor_finder_;
    Visualizer visualizer_;
};
