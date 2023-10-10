#include <cstdio>

#include <dirent.h>

#include <fstream>
#include <sstream>
#include <thread>
#include <queue>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "interfaces/srv/device_info.hpp"

#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

cv::Mat k = (cv::Mat_<double>(3, 3)
        << 437.41295244430756, 0.0, 323.8199711610768, 0.0, 436.9383095177688, 249.3574041448629, 0.0, 0.0, 1.0);
cv::Mat d = (cv::Mat_<double>(1, 5)
        << -0.3561328868709848, 0.1033529329451981, -0.0003164000040999268, -0.0008101407846692977, 0.0);


class CameraNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr armImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereoImageLPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereoImageRPublisher;
    rclcpp::Service<interfaces::srv::DeviceInfo>::SharedPtr switchCameraService;
    rclcpp::TimerBase::SharedPtr timer;
    cv::VideoCapture armCapture;
    cv::VideoCapture stereoCapture;

    cv::Mat image;
    cv::Mat processed_image;
    sensor_msgs::msg::Image imageMsg;
    cv_bridge::CvImage cvImg;

public:
    CameraNode() : Node("camera_node") {


        armImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/arm_camera/img_raw", 10);

        while (!armCapture.isOpened()) {
            armCapture.open(0);
            RCLCPP_WARN_SKIPFIRST(this->get_logger(), "ArmCamera opened failed!");
            std::this_thread::sleep_for(1s);
        }

        armCapture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        armCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        armCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        armCapture.set(cv::CAP_PROP_FPS, 30);

        RCLCPP_INFO(this->get_logger(), "ArmCamera opened success!");

        std::thread armCameraRead([this]() {
            while (rclcpp::ok() && armCapture.isOpened()) {
                armCapture >> image;

                if (!armCapture.isOpened()) {
                    return;
                }

                cv::undistort(image, processed_image, k, d);

                cvImg.encoding = "bgr8";
                cvImg.header.stamp = this->now();

                cvImg.image = processed_image;
                cvImg.toImageMsg(imageMsg);

                armImagePublisher->publish(imageMsg);
            }
        });
        armCameraRead.detach();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}


