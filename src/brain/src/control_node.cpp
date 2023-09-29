#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/item_info.hpp"
#include "interfaces/msg/serial_data.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ControlNode : public rclcpp::Node {
private:
    enum Status {
        NONE, DISC, PILING, PLATFORM
    } status;
    enum Color {
        RED, BLUE
    } color;
    enum Item {
        RED_BOX,
        BLUE_BOX,
        QR_BOX,
        RED_CUBE,
        BLUE_CUBE,
        QR_CUBE,
        RED_SPHERE,
        BLUE_SPHERE,
        YELLOW_SPHERE,
        RED_TUBE,
        BLUE_TUBE,
        WHITE_SPHERE,
    };
    rclcpp::Subscription<interfaces::msg::ItemInfo>::SharedPtr itemInfoSubscription;
    rclcpp::Publisher<interfaces::msg::SerialData>::SharedPtr armSerialDataPublisher;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr startSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    std::array<int, 3> redLow = {0, 115, 80};
    std::array<int, 3> redUp = {15, 255, 255};
    std::array<int, 3> yellowLow = {16, 115, 80};
    std::array<int, 3> yellowUp = {40, 255, 255};
    std::array<int, 3> blueLow = {90, 115, 50};
    std::array<int, 3> blueUp = {120, 255, 255};
public:
    ControlNode() : Node("control") {
        imageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
                "/arm_camera/img_raw",
                1,
                [this](const sensor_msgs::msg::Image::SharedPtr imageMsg) {
                    interfaces::msg::SerialData serialData;
                    cv_bridge::CvImagePtr cvImage;
                    cvImage = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
                    cv::Mat image = cvImage->image;
                    if (image.empty()) {
                        return;
                    }
                    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
                    cv::Mat red, yellow, blue;
                    cv::inRange(image, redLow, redUp, red);
                    cv::inRange(image, yellowLow, yellowUp, yellow);
                    cv::inRange(image, blueLow, blueUp, blue);
                    cv::imshow("red", red);
                    cv::imshow("yellow", yellow);
                    cv::imshow("blue", blue);
                    cv::waitKey(1);
                    RCLCPP_INFO(this->get_logger(), "%f %f %f",
                                1.0f * cv::countNonZero(red) / image.size().area(),
                                1.0f * cv::countNonZero(yellow) / image.size().area(),
                                1.0f * cv::countNonZero(blue) / image.size().area());
                    if (cv::countNonZero(red) / image.size().area() > 0.2) {
                        serialData.id = 0x72;
                        serialData.data[0] = 1;
                        armSerialDataPublisher->publish(serialData);
                    }
                    if (cv::countNonZero(yellow) / image.size().area() > 0.2) {
                        serialData.id = 0x72;
                        serialData.data[0] = 2;
                        armSerialDataPublisher->publish(serialData);
                    }
                    if (cv::countNonZero(blue) / image.size().area() > 0.2) {
                        serialData.id = 0x72;
                        serialData.data[0] = 1;
                        armSerialDataPublisher->publish(serialData);
                    }
                }
        );
        itemInfoSubscription = this->create_subscription<interfaces::msg::ItemInfo>(
                "item_info",
                10,
                [this](const interfaces::msg::ItemInfo::SharedPtr) {
                    interfaces::msg::SerialData serialData;
                    switch (status) {
                        case DISC:
                            /*
                            if (abs(itemInfo->x - 0.5) < 1) {
                                serialData.id = 0x72;
                                switch (itemInfo->id) {
                                    case RED_BOX:
                                    case RED_CUBE:
                                    case RED_SPHERE:
                                    case RED_TUBE:
                                        if (color == RED) {
                                            serialData.data[0] = 1;
                                            armSerialDataPublisher->publish(serialData);
                                        }
                                        break;
                                    case BLUE_CUBE:
                                    case BLUE_SPHERE:
                                    case BLUE_TUBE:
                                    case BLUE_BOX:
                                        if (color == BLUE) {
                                            serialData.data[0] = 1;
                                            armSerialDataPublisher->publish(serialData);
                                        }
                                        break;
                                    case QR_CUBE:
                                    case QR_BOX:
                                        break;
                                    case YELLOW_SPHERE:
                                        serialData.data[0] = 2;
                                        armSerialDataPublisher->publish(serialData);
                                        break;
                                    case WHITE_SPHERE:
                                        break;
                                }
                            }
                             */
                            break;
                        case PILING:

                            break;
                        case PLATFORM:
                            break;
                        case NONE:
                            break;
                    }
                }
        );
        armSerialDataPublisher = this->create_publisher<interfaces::msg::SerialData>("arm_serial", 10);
        startSubscription = this->create_subscription<example_interfaces::msg::Int32>(
                "start",
                10,
                [this](const example_interfaces::msg::Int32::SharedPtr) {
                    run();
                }
        );

    }

    void run() {
        RCLCPP_WARN(this->get_logger(), "START");
        disc();
    }

    void disc() {
        interfaces::msg::SerialData serialData;
        serialData.id = 0x72;
        serialData.data[0] = 0;
        armSerialDataPublisher->publish(serialData);
        status = DISC;
    }

    void piling() {
        interfaces::msg::SerialData serialData;
        serialData.id = 0x73;
        serialData.data[0] = 1;
        armSerialDataPublisher->publish(serialData);
        status = PILING;
    }

    void platform() {

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}