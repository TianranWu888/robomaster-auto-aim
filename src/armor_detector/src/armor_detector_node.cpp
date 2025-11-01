#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "armor_plate.hpp"   

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() : Node("armor_detector_node") {
        this->declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");
        std::string topic = this->get_parameter("camera_topic").as_string();

        detector = std::make_unique<ArmorPlate>();
        detector->set_enemy_color(2);  

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic, 10, std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        int armor_count = detector->locate_armor(frame);
        detector->append_screent_info(frame);
        YPAngles target = detector->get_target_pitch_yaw();

        RCLCPP_INFO(this->get_logger(), "Armor=%d | Yaw=%.2f°, Pitch=%.2f°, Dist=%.2fm",
                    armor_count, target.yaw, target.pitch, target.distance);

        cv::imshow("Armor Detector", frame);
        cv::waitKey(1);
    }

    std::unique_ptr<ArmorPlate> detector;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
