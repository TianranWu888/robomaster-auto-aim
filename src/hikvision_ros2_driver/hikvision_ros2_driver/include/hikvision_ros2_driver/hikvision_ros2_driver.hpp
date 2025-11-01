#pragma once
#ifndef HIKVISION_ROS2_DRIVER_HPP
#define HIKVISION_ROS2_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>

namespace hikvision_ros2_driver {

class HikvisionDriver : public rclcpp::Node {
   public:
    HikvisionDriver(const rclcpp::NodeOptions &options);
    ~HikvisionDriver();

   private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace hikvision_ros2_driver

#endif  // HIKVISION_ROS2_DRIVER_HPP