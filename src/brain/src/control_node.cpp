#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/item_info.hpp"
#include "interfaces/msg/serial_data.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node {
private:
    enum Status {
        NONE, DISC, PLATFORM, PILING, DONE, SORT
    } status;
    enum Color {
        RED, BLUE
    } color = RED;
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
    };
    rclcpp::Subscription<interfaces::msg::ItemInfo>::SharedPtr itemInfoSubscription;
    rclcpp::Publisher<interfaces::msg::SerialData>::SharedPtr armSerialDataPublisher;
    rclcpp::Subscription<interfaces::msg::SerialData>::SharedPtr armDataSubscription;
    rclcpp::Publisher<interfaces::msg::SerialData>::SharedPtr chassisDataPublisher;
    rclcpp::Subscription<interfaces::msg::SerialData>::SharedPtr chassisDataSubscription;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr startSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr qrInfoSubscription;
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr lightPublisher;
    std::array<int, 3> redLow = {0, 115, 80};
    std::array<int, 3> redUp = {15, 255, 255};
    std::array<int, 3> yellowLow = {16, 115, 80};
    std::array<int, 3> yellowUp = {40, 255, 255};
    std::array<int, 3> blueLow = {90, 115, 50};
    std::array<int, 3> blueUp = {120, 255, 255};

    std::array<int, 3> greenLow = {65, 10, 10};
    std::array<int, 3> greenUp = {110, 255, 255};
    bool waiting = false;
    bool turnLeft = false;
    bool turnRight = false;
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

                    if (!waiting && status == PLATFORM) {
                        cv::Mat lineImage;
                        cv::cvtColor(image, lineImage, cv::COLOR_BGR2HSV);
                        cv::Mat line;
                        cv::inRange(lineImage, greenLow, greenUp, line);

                        cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
                        cv::morphologyEx(line, line, cv::MORPH_OPEN, element);
                        cv::imshow("raw", line);
                        cv::Canny(line, line, 50, 200, 3);
                        cv::imshow("line", line);
                        cv::waitKey(1);
                        std::vector<cv::Vec2f> lines;
                        cv::HoughLines(line, lines, 1, CV_PI / 180, 100);

                        cv::Mat copy = image.clone();
                        for (size_t i = 0; i < lines.size(); i++) {
                            float rho = lines[i][0], theta = lines[i][1];
                            if (theta < CV_PI / 3 || theta > 2 * CV_PI / 3 || rho < ) {
                                continue;
                            }

                            cv::Point pt1, pt2;
                            double a = cos(theta), b = sin(theta);
                            double x0 = a * rho, y0 = b * rho;
                            pt1.x = cvRound(x0 + 1000 * (-b));
                            pt1.y = cvRound(y0 + 1000 * (a));
                            pt2.x = cvRound(x0 - 1000 * (-b));
                            pt2.y = cvRound(y0 - 1000 * (a));
                            cv::line(copy, pt1, pt2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

                            if (theta > CV_PI / 180 * 93) {
                                //turnRight = true;
                                serialData.id = 0x32;
                                serialData.data[2] = 1;
                                chassisDataPublisher->publish(serialData);
                                std::thread th([this](){
                                    using namespace std::chrono_literals;
                                    std::this_thread::sleep_for(0.5s);
                                    interfaces::msg::SerialData serialData1;
                                    serialData1.id = 0x32;
                                    serialData1.data[2] = -1;
                                    chassisDataPublisher->publish(serialData1);
                                });
                                th.detach();
                                break;
                            }

                            if (theta < CV_PI / 180 * 87) {
                                //turnLeft = true;
                                serialData.id = 0x32;
                                serialData.data[2] = -1;
                                chassisDataPublisher->publish(serialData);
                                std::thread th([this](){
                                    using namespace std::chrono_literals;
                                    std::this_thread::sleep_for(0.5s);
                                    interfaces::msg::SerialData serialData1;
                                    serialData1.id = 0x32;
                                    serialData1.data[2] = 1;
                                    chassisDataPublisher->publish(serialData1);
                                });
                                th.detach();
                                break;
                            }

//                            if (theta > CV_PI / 180 * 85 && theta < CV_PI / 180 * 95) {
//                                if(turnLeft){
//                                    serialData.id = 0x32;
//                                    serialData.data[2] = 1;
//                                    chassisDataPublisher->publish(serialData);
//                                }
//
//                                if(turnRight){
//                                    serialData.id = 0x32;
//                                    serialData.data[2] = -1;
//                                    chassisDataPublisher->publish(serialData);
//                                }
//                                turnRight = false;
//                                turnLeft = false;
//                                break;
//                            }
                        }
                        cv::imshow("result", copy);
                    }

                    if (status == DISC || status == DONE) {
                        cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
                        cv::Mat red, yellow, blue;
                        cv::inRange(image, redLow, redUp, red);
                        cv::inRange(image, yellowLow, yellowUp, yellow);
                        cv::inRange(image, blueLow, blueUp, blue);
//                        cv::imshow("red", red);
//                        cv::imshow("yellow", yellow);
//                        cv::imshow("blue", blue);
//                        cv::waitKey(1);
                        if (color == RED && (1.0f * cv::countNonZero(red) / image.size().area() > 0.2)) {
                            serialData.id = 0x72 + ((status == DISC) ? 0 : 2);
                            serialData.data[0] = 1 + ((status == DISC) ? 0 : 2);
                            armSerialDataPublisher->publish(serialData);
                        }
                        if (1.0f * cv::countNonZero(yellow) / image.size().area() > 0.2) {
                            serialData.id = 0x72 + ((status == DISC) ? 0 : 2);
                            serialData.data[0] = 2 + ((status == DISC) ? 0 : 10);
                            armSerialDataPublisher->publish(serialData);
                        }
                        if (color == BLUE && (1.0f * cv::countNonZero(blue) / image.size().area() > 0.2)) {
                            serialData.id = 0x72 + ((status == DISC) ? 0 : 2);
                            serialData.data[0] = 1 + ((status == DISC) ? 0 : 2);
                            armSerialDataPublisher->publish(serialData);
                        }
                    }
                }
        );
        itemInfoSubscription = this->create_subscription<interfaces::msg::ItemInfo>(
                "item_info",
                10,
                [this](const interfaces::msg::ItemInfo::SharedPtr itemInfo) {
                    interfaces::msg::SerialData serialData;
                    switch (status) {
                        case NONE:
                            break;
                        case DISC:
                            break;
                        case PILING:
                            break;
                        case PLATFORM:
                            if (!waiting && abs(itemInfo->x - 0.75) < 0.1) {
                                auto cube = [this, &serialData]() {
                                    RCLCPP_WARN(this->get_logger(), "FOUND CUBE！");
                                    waiting = true;
                                    serialData.id = 0x31;
                                    chassisDataPublisher->publish(serialData);

                                    serialData.id = 0x73;
                                    serialData.data[0] = 0;
                                    armSerialDataPublisher->publish(serialData);
                                };
                                auto tube = [this, &serialData]() {
                                    RCLCPP_WARN(this->get_logger(), "FOUND TUBE！");
                                    waiting = true;
                                    serialData.id = 0x31;
                                    chassisDataPublisher->publish(serialData);

                                    serialData.id = 0x73;
                                    serialData.data[0] = 1;
                                    armSerialDataPublisher->publish(serialData);
                                };
                                auto qr = [this, &serialData]() {
                                    RCLCPP_WARN(this->get_logger(), "FOUND QR！");
                                    waiting = true;
                                    serialData.id = 0x31;
                                    chassisDataPublisher->publish(serialData);

                                    serialData.id = 0x73;
                                    serialData.data[0] = 2;
                                    armSerialDataPublisher->publish(serialData);

                                    example_interfaces::msg::Int32 lightData;
                                    lightData.data = 1;
                                    lightPublisher->publish(lightData);
                                };
                                switch (itemInfo->id) {
                                    case RED_BOX:
                                    case RED_CUBE:
                                        if (color == RED) {
                                            cube();
                                        }
                                        break;
                                    case RED_SPHERE:
                                    case RED_TUBE:
                                        if (color == RED) {
                                            tube();
                                        }
                                        break;
                                    case BLUE_BOX:
                                    case BLUE_CUBE:
                                        if (color == BLUE) {
                                            cube();
                                        }
                                        break;
                                    case BLUE_SPHERE:
                                    case BLUE_TUBE:
                                        if (color == BLUE) {
                                            tube();
                                        }
                                        break;
                                    case QR_CUBE:
                                    case QR_BOX:
                                        qr();
                                        break;
                                    case YELLOW_SPHERE:
                                        break;
                                }
                            }
                            break;
                        case DONE:
                            break;
                        case SORT:
                            break;
                    }
                }
        );
        lightPublisher = this->create_publisher<example_interfaces::msg::Int32>("light_change", 10);
        armSerialDataPublisher = this->create_publisher<interfaces::msg::SerialData>("arm_serial", 10);
        chassisDataPublisher = this->create_publisher<interfaces::msg::SerialData>("chassis_serial", 10);
        chassisDataSubscription = this->create_subscription<interfaces::msg::SerialData>(
                "chassis_data",
                10,
                [this](const interfaces::msg::SerialData::SharedPtr) {
                    interfaces::msg::SerialData serialData;
                    example_interfaces::msg::Int32 lightData;
                    switch (status) {
                        case NONE:
                            RCLCPP_WARN(this->get_logger(), "DISC ARRIVED！");
                            status = DISC;
                            lightData.data = 0;
                            lightPublisher->publish(lightData);
                            disc();
                            break;
                        case DISC:
                            RCLCPP_WARN(this->get_logger(), "PLATFORM ARRIVED！");
                            status = PLATFORM;
                            platform();
                            break;
                        case PLATFORM:
                            RCLCPP_WARN(this->get_logger(), "PLATFORM FINISHED！");
                            status = PILING;
                            serialData.id = 0x74;
                            serialData.data[0] = 1;
                            armSerialDataPublisher->publish(serialData);
                            break;
                        case PILING:
                            RCLCPP_WARN(this->get_logger(), "PILING ARRIVED！");
                            lightData.data = 0;
                            lightPublisher->publish(lightData);
                            piling();
                            status = DONE;
                            break;
                        case DONE:
                            RCLCPP_WARN(this->get_logger(), "PILING FINISHED！");
                            serialData.id = 0x74;
                            serialData.data[0] = 4;
                            armSerialDataPublisher->publish(serialData);
                            status = SORT;
                            break;
                        case SORT:
                            RCLCPP_WARN(this->get_logger(), "WAREHOUSE ARRIVED！");
                            serialData.id = 0x75;
                            serialData.data[0] = 1;
                            armSerialDataPublisher->publish(serialData);
                            //status = NONE;
                            break;
                    }
                }
        );
        armDataSubscription = this->create_subscription<interfaces::msg::SerialData>(
                "arm_data",
                10,
                [this](const interfaces::msg::SerialData::SharedPtr) {
                    if (status == PLATFORM) {
                        waiting = false;
                        interfaces::msg::SerialData serialData;
                        serialData.id = 0x31;
                        chassisDataPublisher->publish(serialData);
                        RCLCPP_WARN(this->get_logger(), "ARM FINISHED. CONTINUE MOVING!！");
                    } else if (status == DONE) {
                        interfaces::msg::SerialData serialData;
                        serialData.id = 0x3f;
                        chassisDataPublisher->publish(serialData);
                        RCLCPP_WARN(this->get_logger(), "ARM PICKING CENTER BALL FINISHED. PILING MOVING!！");
                    } else if (status == SORT) {
                        interfaces::msg::SerialData serialData;
                        serialData.id = 0x3f;
                        chassisDataPublisher->publish(serialData);
                        RCLCPP_WARN(this->get_logger(), "ARM TUBE SORT FINISHED!");
                    }
                }
        );
        startSubscription = this->create_subscription<example_interfaces::msg::Int32>(
                "start",
                10,
                [this](const example_interfaces::msg::Int32::SharedPtr data) {
                    if (data->data == 0) {
                        color = RED;
                        RCLCPP_WARN(this->get_logger(), "RED");
                    } else {
                        color = BLUE;
                        RCLCPP_WARN(this->get_logger(), "BLUE");
                    }
                    run();
                }
        );

        qrInfoSubscription = this->create_subscription<example_interfaces::msg::Int32>(
                "qr_code_info",
                10,
                [this](const example_interfaces::msg::Int32::SharedPtr id) {
                    if (waiting) {
                        example_interfaces::msg::Int32 lightData;
                        lightData.data = 0;
                        lightPublisher->publish(lightData);

                        interfaces::msg::SerialData serialData;
                        serialData.id = 0x73;
                        if (id->data == 1) {
                            if (color == BLUE) {
                                RCLCPP_WARN(this->get_logger(), "COLOR RIGHT!！");
                                serialData.data[0] = 4;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "COLOR WRONG!！");
                                serialData.data[0] = 3;
                            }
                        } else if (id->data == 2) {
                            if (color == RED) {
                                RCLCPP_WARN(this->get_logger(), "COLOR RIGHT!！");
                                serialData.data[0] = 4;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "COLOR WRONG!！");
                                serialData.data[0] = 3;
                            }
                        }
                        armSerialDataPublisher->publish(serialData);
                    }
                }
        );


    }

    void run() {
        RCLCPP_WARN(this->get_logger(), "START");
        interfaces::msg::SerialData serialData;
        serialData.id = 0x3F;
        chassisDataPublisher->publish(serialData);
        status = NONE;
    }

    void disc() {
        interfaces::msg::SerialData serialData;
        serialData.id = 0x72;
        serialData.data[0] = 0;
        armSerialDataPublisher->publish(serialData);
        RCLCPP_WARN(this->get_logger(), "ARM REACHING！");

        std::thread th([this]() {
            std::this_thread::sleep_for(40s);
            interfaces::msg::SerialData serialData;
            serialData.id = 0x3F;
            chassisDataPublisher->publish(serialData);
            RCLCPP_WARN(this->get_logger(), "DISC TIMES UP！");

            serialData.id = 0x72;
            serialData.data[0] = 3;
            armSerialDataPublisher->publish(serialData);
            RCLCPP_WARN(this->get_logger(), "ARM CHANGING!");
        });
        th.detach();
    }

    void piling() {
        interfaces::msg::SerialData serialData;
        serialData.id = 0x74;
        serialData.data[0] = 2;
        armSerialDataPublisher->publish(serialData);
    }

    void platform() {
        interfaces::msg::SerialData serialData;
        serialData.id = 0x31;
        chassisDataPublisher->publish(serialData);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
