#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <chrono>

using std::placeholders::_1;
using namespace cv;
using namespace std;
using namespace std::chrono;

// Armor Detection Class
#define REDENEMY 2
#define BLUEENEMY 0

struct ArmorObj {
    RotatedRect armor;
    double armor_distance;
    vector<Point2f> edge_vec;
};

class ArmorPlate {
public:
    ArmorPlate() {}

    void set_enemy_color(int colour) { enemy_color = colour; }

    int locate_armor(Mat &img) {
        if (img.empty()) return 0;

        vector<Mat> channels;
        split(img, channels);

        Mat color_sub;
        if (enemy_color == REDENEMY)
            color_sub = channels[2] - channels[0];
        else
            color_sub = channels[0] - channels[2];

        threshold(color_sub, binary, 50, 255, THRESH_BINARY);
        GaussianBlur(binary, binary, Size(3, 3), 0);
        dilate(binary, binary, getStructuringElement(MORPH_RECT, Size(3, 3)));

        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        light_infos.clear();
        armors.clear();

        for (auto &c : contours) {
            if (contourArea(c) < 30) continue;
            RotatedRect rect = minAreaRect(c);

            float aspect_ratio = max(rect.size.width, rect.size.height) /
                                 min(rect.size.width, rect.size.height);
            if (aspect_ratio > 2.0 && aspect_ratio < 15.0) {
                light_infos.push_back(rect);
            }
        }

        for (size_t i = 0; i < light_infos.size(); i++) {
            for (size_t j = i + 1; j < light_infos.size(); j++) {
                auto &l1 = light_infos[i];
                auto &l2 = light_infos[j];

                float angle_diff = fabs(l1.angle - l2.angle);
                float height_diff = fabs(l1.center.y - l2.center.y);
                float height_mean = (l1.size.height + l2.size.height) / 2;
                float x_diff = fabs(l1.center.x - l2.center.x);

                if (angle_diff < 10 && height_diff < height_mean * 0.5 &&
                    x_diff > height_mean * 0.5 && x_diff < height_mean * 5.0) {

                    ArmorObj armor;
                    armor.armor = RotatedRect(
                        (l1.center + l2.center) / 2.0,
                        Size2f(x_diff, height_mean * 1.5),
                        (l1.angle + l2.angle) / 2.0
                    );
                    armors.push_back(armor);
                }
            }
        }

        return (int)armors.size();
    }

    void draw_armors(Mat &img) {
        for (auto &armor : armors) {
            Point2f pts[4];
            armor.armor.points(pts);
            for (int i = 0; i < 4; i++) {
                line(img, pts[i], pts[(i + 1) % 4], Scalar(0, 255, 0), 2);
            }
        }
    }

private:
    int enemy_color = REDENEMY;
    Mat binary;
    vector<RotatedRect> light_infos;
    vector<ArmorObj> armors;
};

// ROS2 Node with FPS measurement
class ArmorDetector : public rclcpp::Node {
public:
    ArmorDetector() : Node("armor_detector_node") {
        RCLCPP_INFO(this->get_logger(), "Armor Detector Node Started");

        plate_.set_enemy_color(REDENEMY);

        image_sub_ = image_transport::create_subscription(
            this,
            "/driver/hikvision/my_hik_camera/raw/image",
            std::bind(&ArmorDetector::imageCallback, this, _1),
            "raw"
        );

        last_time_ = steady_clock::now();
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

        // --- Time start ---
        auto start = steady_clock::now();

        int armor_count = plate_.locate_armor(frame);
        plate_.draw_armors(frame);

        // --- Time end ---
        auto end = steady_clock::now();
        double frame_time_ms = duration_cast<microseconds>(end - start).count() / 1000.0;
        double fps = 1000.0 / frame_time_ms;

        string info = "Detected: " + to_string(armor_count) +
                      " | " + to_string(fps).substr(0, 5) + " FPS";
        cv::putText(frame, info, Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2);

        cv::imshow("Armor Detection", frame);
        cv::waitKey(1);

        RCLCPP_INFO(this->get_logger(), "Detected %d armors | %.2f FPS", armor_count, fps);
    }

    image_transport::Subscriber image_sub_;
    ArmorPlate plate_;
    steady_clock::time_point last_time_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
