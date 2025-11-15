#include "armor_detector/armor_detector_node.hpp"

ArmorDetectorNode::ArmorDetectorNode(): Node("armor_detector_node"){
    RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode started.");

    // 订阅相机图像
    img_sub_ = image_transport::create_subscription(
        this,
        "/driver/hikvision/my_hik_camera/raw/image",
        std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1),
        "raw"
    );

    // 发布调试图像
    debug_img_pub_ = image_transport::create_publisher(
        this,
        "/armor_detector/debug_image"
    );
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();

    // 二值图
    cv::Mat binary;
    armor_finder_.preprocess(frame, binary);

    // 找灯条（调用 ArmorFinder 的内部函数）
    auto lights = armor_finder_.findLights(binary);

    // 找装甲板
    auto armors = armor_finder_.matchArmors(lights);

    // 可视化
    visualizer_.drawDebug(frame, lights, armors, armors.size(), binary);

    // 发布调试图像
    auto msg_out = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    debug_img_pub_.publish(msg_out);
}
