#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class ArmorDetector : public rclcpp::Node {
public:
    ArmorDetector() : Node("armor_detector_node") {
        RCLCPP_INFO(this->get_logger(), "Armor Detector Node Started");

        // 声明参数（enemy_color默认值为 "blue"，可设为"red"）
        this->declare_parameter<std::string>("enemy_color", "blue");
        this->get_parameter("enemy_color", enemy_color_);

        // 订阅图像话题
        image_sub_ = image_transport::create_subscription(
            this,
            "/driver/hikvision/my_hik_camera/raw/image",
            std::bind(&ArmorDetector::imageCallback, this, _1),
            "raw"
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // 转换到HSV
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 根据 enemy_color 选择不同阈值
        cv::Scalar lower, upper;
        if (enemy_color_ == "blue") {
            lower = cv::Scalar(100, 80, 80);
            upper = cv::Scalar(130, 255, 255);
        } else if (enemy_color_ == "red") {
            // 红色分两段，需要两个范围合并
            cv::Mat mask1, mask2;
            cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv, cv::Scalar(160, 80, 80), cv::Scalar(180, 255, 255), mask2);
            cv::bitwise_or(mask1, mask2, mask_);
        }

        // 若是蓝色，直接 inRange
        if (enemy_color_ == "blue") {
            cv::inRange(hsv, lower, upper, mask_);
        }

        // 显示
        cv::imshow("mask", mask_);
        cv::waitKey(1);
    }

    std::string enemy_color_;
    cv::Mat mask_;
    image_transport::Subscriber image_sub_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetector>());
    rclcpp::shutdown();
    return 0;
}
