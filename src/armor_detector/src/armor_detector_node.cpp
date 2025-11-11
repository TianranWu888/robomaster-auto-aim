#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

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
        } else if (enemy_color_ != "red") {
            mask_ = cv::Mat::zeros(frame.size(), CV_8UC1);
        }

        // 使用形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, kernel);

        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat contour_input = mask_.clone();
        cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 筛选轮廓并拟合最小外接矩形
        std::vector<cv::RotatedRect> lightbars;
        lightbars.reserve(contours.size());
        constexpr int kMaxContourPoints = 10;
        constexpr double kMinContourArea = 30.0;
        for (const auto &contour : contours) {
            if (contour.size() > kMaxContourPoints) {
                continue;
            }
            double area = cv::contourArea(contour);
            if (area < kMinContourArea) {
                continue;
            }

            cv::RotatedRect rect = cv::minAreaRect(contour);
            float width = rect.size.width;
            float height = rect.size.height;
            if (width <= 0 || height <= 0) {
                continue;
            }

            if (width > height) {
                std::swap(width, height);
            }

            float aspect_ratio = height / width;
            if (aspect_ratio < 1.5f || aspect_ratio > 15.0f) {
                continue;
            }

            lightbars.emplace_back(rect);
        }

        // 计算候选装甲板
        std::vector<cv::RotatedRect> armor_candidates;
        auto normalized_angle = [](const cv::RotatedRect &rect) {
            float angle = rect.angle;
            if (rect.size.width < rect.size.height) {
                return angle;
            }
            return angle + 90.0f;
        };

        for (size_t i = 0; i < lightbars.size(); ++i) {
            for (size_t j = i + 1; j < lightbars.size(); ++j) {
                const auto &rect1 = lightbars[i];
                const auto &rect2 = lightbars[j];

                const cv::RotatedRect &left = rect1.center.x < rect2.center.x ? rect1 : rect2;
                const cv::RotatedRect &right = rect1.center.x < rect2.center.x ? rect2 : rect1;

                float height1 = std::max(rect1.size.width, rect1.size.height);
                float height2 = std::max(rect2.size.width, rect2.size.height);
                float height_ratio = height1 > height2 ? height1 / height2 : height2 / height1;
                if (height_ratio > 1.5f) {
                    continue;
                }

                float angle_diff = std::fabs(normalized_angle(rect1) - normalized_angle(rect2));
                if (angle_diff > 90.0f) {
                    angle_diff = 180.0f - angle_diff;
                }
                if (angle_diff > 15.0f) {
                    continue;
                }

                float center_distance = static_cast<float>(cv::norm(rect1.center - rect2.center));
                float avg_height = (height1 + height2) / 2.0f;
                float distance_ratio = center_distance / avg_height;
                if (distance_ratio < 0.5f || distance_ratio > 4.0f) {
                    continue;
                }

                float vertical_diff = std::fabs(rect1.center.y - rect2.center.y);
                if (vertical_diff > avg_height * 0.8f) {
                    continue;
                }

                // 组合两个灯条的角点用于拟合装甲板
                cv::Point2f left_pts[4];
                cv::Point2f right_pts[4];
                left.points(left_pts);
                right.points(right_pts);

                std::vector<cv::Point2f> all_points;
                all_points.reserve(8);
                all_points.insert(all_points.end(), left_pts, left_pts + 4);
                all_points.insert(all_points.end(), right_pts, right_pts + 4);

                armor_candidates.emplace_back(cv::minAreaRect(all_points));
            }
        }

        // 可视化灯条与装甲板候选
        cv::Mat visual = frame.clone();
        for (const auto &rect : lightbars) {
            cv::Point2f pts[4];
            rect.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(visual, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
            }
        }

        for (const auto &rect : armor_candidates) {
            cv::Point2f pts[4];
            rect.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(visual, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            }
        }

        cv::imshow("mask", mask_);
        cv::imshow("armor_detection", visual);
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
