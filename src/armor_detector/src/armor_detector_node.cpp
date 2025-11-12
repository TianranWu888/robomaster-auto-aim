#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;
using namespace cv;
using namespace std;

/* ============================================================
   辅助函数：矫正灯条的角度与宽高
   ============================================================ */
RotatedRect& adjustRec(cv::RotatedRect& rec)
{
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    // 规范化角度到 [-90, 90)
    angle = std::fmod(angle, 180.0f);
    if (angle >= 90.0f) angle -= 180.0f;
    if (angle < -90.0f) angle += 180.0f;

    // 如果角度绝对值大于等于 45，则交换宽高并旋转角度 90 度
    if (angle >= 45.0f){
        std::swap(width, height);
        angle -= 90.0f;
    } else if (angle < -45.0f){
        std::swap(width, height);
        angle += 90.0f;
    }
    
    return rec;
}   

/* ============================================================
   数据结构定义
   ============================================================ */
struct YPAngles {
    double yaw;
    double pitch;
    double distance;
};

struct ArmorObj {
    RotatedRect armor;
    vector<Point2f> edge_vec;
    double armor_distance;
};

struct RectParams {
    double width;
    double height;
    Mat rotation;
};

/* ============================================================
   ArmorPlate 类
   ============================================================ */
class ArmorPlate {
public:
    ArmorPlate() {}

    int locate_armor(Mat img);
    void set_enemy_color(int c) { enemy_color = c; }
    YPAngles get_target_pitch_yaw();

    // === Getter 接口 ===
    const Mat& getBinary() const { return binary; }
    size_t getLightCount() const { return light_infos.size(); }

private:
    vector<vector<Point>> light_contours;
    vector<RotatedRect> light_infos;
    vector<Mat> channels;
    Mat binary;
    RotatedRect armor_rect;
    int enemy_color = 2;  // 2 = red, 1 = blue
    vector<ArmorObj> armors;

    RectParams recoverRectangleParameters(double aspect, const Vec2d& v1, const Vec2d& v2);
    double calculateAbsoluteRotationAngle(const Mat& R);
    YPAngles getYawAndPitch(int pixelX, int pixelY);
};

/* ============================================================
   几何恢复（反投影）
   ============================================================ */
RectParams ArmorPlate::recoverRectangleParameters(double aspect, const Vec2d &v1, const Vec2d &v2)
{
    RectParams params;

    double a = norm(v1);
    double b = norm(v2);
    Vec2d u1 = v1 / a;
    Vec2d u2 = v2 / b;
    double D = u1.dot(u2);

    double A = (a * a) / (aspect * aspect * b * b);
    double B = A - 1.0;
    double disc = B * B + 4 * A * (D * D);
    if (disc < 0) disc = 0;
    double sqrt_disc = sqrt(disc);
    double X1 = (-B + sqrt_disc) / (2 * A);
    double X2 = (-B - sqrt_disc) / (2 * A);
    double X = (X1 >= 0 ? X1 : (X2 >= 0 ? X2 : 0));

    double tan_alpha = sqrt(X);
    double alpha = atan(tan_alpha);
    double tan_beta = (fabs(tan_alpha) < 1e-6 ? 0.0 : -D / tan_alpha);
    double beta = atan(tan_beta);

    params.width  = a / cos(alpha);
    params.height = b / cos(beta);

    Vec3d r0(u1[0] * cos(alpha), u1[1] * cos(alpha), sin(alpha));
    Vec3d r1(u2[0] * cos(beta),  u2[1] * cos(beta),  sin(beta));
    Vec3d r2 = r0.cross(r1);
    r2 /= norm(r2);

    Mat R = (Mat_<double>(3, 3) <<
        r0[0], r1[0], r2[0],
        r0[1], r1[1], r2[1],
        r0[2], r1[2], r2[2]
    );
    params.rotation = R;
    return params;
}

/* ============================================================
   计算旋转角度
   ============================================================ */
double ArmorPlate::calculateAbsoluteRotationAngle(const Mat& R)
{
    double traceR = R.at<double>(0,0)
                  + R.at<double>(1,1)
                  + R.at<double>(2,2);
    double theta = acos((traceR - 1) / 2.0) * 180.0 / CV_PI;
    return fabs(theta);
}

/* ============================================================
   像素 → yaw/pitch
   ============================================================ */
YPAngles ArmorPlate::getYawAndPitch(int pixelX, int pixelY)
{
    const double imageWidthPixels  = 1440.0;
    const double physicalWidth     = 0.48;
    const double imageHeightPixels = 1080.0;
    const double physicalHeight    = 0.36;
    const double distance          = 0.6;

    double cx = imageWidthPixels / 2.0;
    double cy = imageHeightPixels / 2.0;
    double px = physicalWidth  / imageWidthPixels;
    double py = physicalHeight / imageHeightPixels;

    double offsetX = (pixelX - cx) * px;
    double offsetY = (pixelY - cy) * py;

    double yawRad   = atan(offsetX / distance);
    double pitchRad = atan(offsetY / distance);

    const double pitch_offset = 9.0;

    YPAngles result;
    result.yaw   = yawRad   * 180.0 / CV_PI;
    result.pitch = - pitchRad * 180.0 / CV_PI - pitch_offset;
    result.distance = distance * 100.0; // cm
    return result;
}

/* ============================================================
   获取装甲目标 yaw/pitch
   ============================================================ */
YPAngles ArmorPlate::get_target_pitch_yaw()
{
    if (armors.empty())
        return {0,0,0};

    auto c = armors[0].armor.center;
    auto r = getYawAndPitch(c.x, c.y);
    r.distance = armors[0].armor_distance;
    return r;
}

/* ============================================================
   locate_armor 核心检测逻辑
   ============================================================ */
int ArmorPlate::locate_armor(Mat img)
{
    GaussianBlur(img, img, Size(3, 3), 0);
    split(img, channels);

    // 颜色差分法 (RGB)
    if (enemy_color == 2) // 红方敌人
        subtract(channels[2], channels[0], binary);
    else                  // 蓝方敌人
        subtract(channels[0], channels[2], binary);

    threshold(binary, binary, 150, 255, THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(binary, binary, MORPH_CLOSE, element);

    findContours(binary, light_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    light_infos.clear();

    for (auto& contour : light_contours)
    {
        float area = contourArea(contour);
        if (contour.size() < 6 || area < 10) continue;

        RotatedRect light = fitEllipse(contour);
        adjustRec(light);

        if ((light.size.width / light.size.height) > 0.8) continue;

        if (light.size.width / light.size.height > 1.0 ||
            area / (light.size.width * light.size.height) < 0.5)
            continue;

        light.size.width *= 1.1;
        light.size.height *= 1.1;

        if (light.size.height > 10 && light.size.height < 150 &&
            (light.angle < 45 || light.angle > 135))
            light_infos.push_back(light);
    }

    // 无灯条
    if (light_infos.size() < 2)
    {
        armors.clear();
        return 0;
    }

    sort(light_infos.begin(), light_infos.end(),
        [](auto& a, auto& b) { return a.center.x < b.center.x; });

    armors.clear();

    for (int i = 0; i < (int)light_infos.size(); i++)
    {
        for (int j = i+1; j < (int)light_infos.size(); j++)
        {
            auto& left  = light_infos[i];
            auto& right = light_infos[j];

            float heightDiff = fabs(left.size.height - right.size.height);
            float widthDiff  = fabs(left.size.width  - right.size.width);
            float angleDiff  = fabs(left.angle - right.angle);
            float dis = norm(left.center - right.center);
            float meanh = (left.size.height + right.size.height) / 2;
            float yDiffRatio = fabs(left.center.y - right.center.y) / meanh;
            float xDiffRatio = fabs(left.center.x - right.center.x) / meanh;
            float ratio = dis / meanh;

            if (angleDiff > 10 || xDiffRatio < 0.5 || yDiffRatio > 0.7
                || ratio > 3 || ratio < 1)
                continue;

            ArmorObj obj;
            vector<Point2f> edges;
            edges.push_back(right.center - left.center);

            float len = meanh;
            float ang = (left.angle + right.angle + 180) / 360 * CV_PI;
            edges.push_back(Point2f(len*cos(ang), len*sin(ang)));

            obj.edge_vec = edges;
            armor_rect.center = (left.center + right.center) * 0.5;
            armor_rect.angle  = (left.angle + right.angle) * 0.5;
            armor_rect.size.height = meanh;
            armor_rect.size.width  = dis;
            obj.armor = armor_rect;

            const float WH_ratio = 1.98;
            const double dist_coe = 237;
            auto rec = recoverRectangleParameters(
                WH_ratio,
                Vec2d(edges[0].x, edges[0].y),
                Vec2d(edges[1].x, edges[1].y)
            );

            double absoluteAngle = calculateAbsoluteRotationAngle(rec.rotation);
            double dist = dist_coe / rec.width;

            obj.armor_distance = dist;
            armors.push_back(obj);
        }
    }

    return armors.size();
}


class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() : Node("armor_detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode started");

        // 敌方颜色参数
        this->declare_parameter<int>("enemy_color", 2);
        int color;
        this->get_parameter("enemy_color", color);
        armor_.set_enemy_color(color);

        // 图像订阅
        image_sub_ = image_transport::create_subscription(
            this,
            "/driver/hikvision/my_hik_camera/raw/image",
            std::bind(&ArmorDetectorNode::imageCallback, this, _1),
            "raw"
        );

        // 可选：相机信息订阅
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/driver/hikvision/my_hik_camera/info",
            10,
            std::bind(&ArmorDetectorNode::cameraInfoCallback, this, _1)
        );
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        (void)msg; // 当前未使用
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        Mat frame = cv_ptr->image;

        int armor_count = armor_.locate_armor(frame);

        Mat mask_gray = armor_.getBinary().clone();
        if (mask_gray.empty()) mask_gray = Mat::zeros(frame.size(), CV_8UC1);

        Mat mask_color;
        cvtColor(mask_gray, mask_color, COLOR_GRAY2BGR);

        Mat combined;
        hconcat(mask_color, frame, combined);

        YPAngles yp = armor_.get_target_pitch_yaw();
        string info1 = "Lights: " + std::to_string((int)armor_.getLightCount());
        string info2 = "Armors: " + std::to_string(armor_count);
        string info3 = "Yaw=" + std::to_string((int)yp.yaw) +
                       "  Pitch=" + std::to_string((int)yp.pitch) +
                       "  Dist=" + std::to_string((int)yp.distance) + "cm";

        putText(combined, info1, Point(20, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), 2);
        putText(combined, info2, Point(20, 80), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), 2);
        putText(combined, info3, Point(20, 120), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255,255,0), 2);

        Mat display;
        resize(combined, display, Size(), 0.6, 0.6);
        imshow("Armor Detection Combined", display);
        waitKey(1);
    }

    ArmorPlate armor_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
