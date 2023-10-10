#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "body/Communication.h"
#include "interfaces/msg/item_info.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

const std::array<int, 3> redLow = {0, 115, 80};
const std::array<int, 3> redUp = {15, 255, 255};
const std::array<int, 3> yellowLow = {16, 115, 80};
const std::array<int, 3> yellowUp = {40, 255, 255};
const std::array<int, 3> blueLow = {90, 115, 50};
const std::array<int, 3> blueUp = {120, 255, 255};

const std::array<int, 3> greenLow = {65, 10, 10};
const std::array<int, 3> greenUp = {110, 255, 255};

class BodyNode : public rclcpp::Node {
private:
    enum Color {
        RED, BLUE
    } color;

    enum Status {
        NONE,
        DISC,
        PLATFORM,
        PILLING,
        SORTING,
    } status;

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

    bool waiting;

    Communication arm;
    Communication chassis;
    Communication qrCode;

    rclcpp::Subscription<interfaces::msg::ItemInfo>::SharedPtr itemInfoSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;

public:
    BodyNode() : Node("body") {
        arm.open(0x0114, 0x0514);
        chassis.open(0x9909, 0x0090);
        qrCode.open(0x26f1, 0x8803);
        itemInfoSubscription = create_subscription<interfaces::msg::ItemInfo>(
                "item_info",
                10,
                [this](const interfaces::msg::ItemInfo::SharedPtr itemInfo) {
                    if (status == PILLING) {

                        if (!waiting && abs(itemInfo->x - 0.75) < 0.1) {
                            auto cube = [this]() {
                                RCLCPP_WARN(this->get_logger(), "FOUND CUBE！");
                                Data data;
                                waiting = true;
                                data[0] = 0x31;
                                chassis.send(data);

                                data[0] = 0x01;
                                data[1] = 0x00;
                                arm.send(data);

                                arm.read();
                                waiting = false;
                                data[0] = 0x31;
                                chassis.send(data);
                            };
                            auto tube = [this]() {
                                RCLCPP_WARN(this->get_logger(), "FOUND TUBE！");
                                Data data;
                                waiting = true;
                                data[0] = 0x31;
                                chassis.send(data);

                                data[0] = 0x01;
                                data[1] = 0x01;
                                arm.send(data);

                                arm.read();
                                waiting = false;
                                data[0] = 0x31;
                                chassis.send(data);
                            };
                            auto qr = [this]() {
                                RCLCPP_WARN(this->get_logger(), "FOUND QR！");
                                Data data;
                                waiting = true;
                                data[0] = 0x31;
                                chassis.send(data);

                                data[0] = 0x01;
                                data[1] = 0x02;
                                arm.send(data);

                                data = qrCode.read_timeout();
                                if (!((data[0] == 'R') ^ (color == RED))) {
                                    data[0] = 0x01;
                                    data[1] = 0x04;
                                    arm.send(data);
                                }else{
                                    data[0] = 0x01;
                                    data[1] = 0x03;
                                    arm.send(data);
                                }

                                arm.read();
                                waiting = false;
                                data[0] = 0x31;
                                chassis.send(data);
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
                    }
                }
        );
        imageSubscription = create_subscription<sensor_msgs::msg::Image>(
                "arm_camera/img_raw",
                10,
                [this](const sensor_msgs::msg::Image::SharedPtr imageMsg) {
                    cv_bridge::CvImagePtr cvImage;
                    cvImage = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
                    cv::Mat image = cvImage->image;
                    if (image.empty()) {
                        return;
                    }

//                    if (!waiting && status == PLATFORM) {
//                        cv::Mat lineImage;
//                        cv::cvtColor(image, lineImage, cv::COLOR_BGR2HSV);
//                        cv::Mat line;
//                        cv::inRange(lineImage, greenLow, greenUp, line);
//
//                        cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
//                        cv::morphologyEx(line, line, cv::MORPH_OPEN, element);
//                        cv::imshow("raw", line);
//                        cv::Canny(line, line, 50, 200, 3);
//                        cv::imshow("line", line);
//                        cv::waitKey(1);
//                        std::vector<cv::Vec2f> lines;
//                        cv::HoughLines(line, lines, 1, CV_PI / 180, 100);
//
//                        cv::Mat copy = image.clone();
//                        for (size_t i = 0; i < lines.size(); i++) {
//                            float rho = lines[i][0], theta = lines[i][1];
//                            if (theta < CV_PI / 3 || theta > 2 * CV_PI / 3 || rho < 100) {
//                                continue;
//                            }
//
//                            cv::Point pt1, pt2;
//                            double a = cos(theta), b = sin(theta);
//                            double x0 = a * rho, y0 = b * rho;
//                            pt1.x = cvRound(x0 + 1000 * (-b));
//                            pt1.y = cvRound(y0 + 1000 * (a));
//                            pt2.x = cvRound(x0 - 1000 * (-b));
//                            pt2.y = cvRound(y0 - 1000 * (a));
//                            cv::line(copy, pt1, pt2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
//
//                            if (theta > CV_PI / 180 * 93) {
//                                //turnRight = true;
//                                data.id = 0x32;
//                                serialData.data[2] = 1;
//                                chassisDataPublisher->publish(serialData);
//                                std::thread th([this](){
//                                    using namespace std::chrono_literals;
//                                    std::this_thread::sleep_for(0.5s);
//                                    interfaces::msg::SerialData serialData1;
//                                    serialData1.id = 0x32;
//                                    serialData1.data[2] = -1;
//                                    chassisDataPublisher->publish(serialData1);
//                                });
//                                th.detach();
//                                break;
//                            }
//
//                            if (theta < CV_PI / 180 * 87) {
//                                //turnLeft = true;
//                                serialData.id = 0x32;
//                                serialData.data[2] = -1;
//                                chassisDataPublisher->publish(serialData);
//                                std::thread th([this](){
//                                    using namespace std::chrono_literals;
//                                    std::this_thread::sleep_for(0.5s);
//                                    interfaces::msg::SerialData serialData1;
//                                    serialData1.id = 0x32;
//                                    serialData1.data[2] = 1;
//                                    chassisDataPublisher->publish(serialData1);
//                                });
//                                th.detach();
//                                break;
//                            }
//                        }
//                        cv::imshow("result", copy);
//                    }

                    if (status == DISC) {
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
                            Data data;
                            data[0] = 0x00;
                            data[1] = 0x01;
                            arm.send(data);
                        }
                        if (1.0f * cv::countNonZero(yellow) / image.size().area() > 0.2) {
                            Data data;
                            data[0] = 0x00;
                            data[1] = 0x02;
                            arm.send(data);
                        }
                        if (color == BLUE && (1.0f * cv::countNonZero(blue) / image.size().area() > 0.2)) {
                            Data data;
                            data[0] = 0x00;
                            data[1] = 0x01;
                            arm.send(data);
                        }
                    }
                }
        );

        std::thread th([this]() {
            Data data;

            arm.testConnect();
            RCLCPP_WARN(get_logger(),"ARM_CONNECT!");
            chassis.testConnect();
            RCLCPP_WARN(get_logger(),"CHASSIS_CONNECT!");


            //wait start
            data = chassis.read();
            if (data[0] == 0) {
                color = RED;
            } else {
                color = BLUE;
            }

            //wait for arriving disc
            chassis.read();
            status = DISC;

            data[0] = 0x00;
            data[1] = 0x00;
            arm.send(data);

            //wait disc end
            std::this_thread::sleep_for(30s);

            data[0] = 0x00;
            data[1] = 0x03;
            arm.send(data);

            data[0] = 0x3F;
            chassis.send(data);

            //wait for arriving platform
            chassis.read();
            status = PLATFORM;

            data[0] = 0x31;
            chassis.send(data);

            //wait for platform end
            chassis.read();
            status = PILLING;
            data[0] = 0x31;
            chassis.send(data);

            data[0] = 0x02;
            data[1] = 0x00;
            arm.send(data);

            chassis.read();

            data[0] = 0x02;
            data[1] = 0x01;
            arm.send(data);

            arm.read();

            data[0] = 0x31;
            chassis.send(data);

            //wait for arriving tube pilling
            chassis.read();
            status = SORTING;

            data[0] = 0x03;
            data[1] = 0x00;
            arm.send(data);

            //wait for arm finish
            arm.read();

            data[0] = 0x3F;
            chassis.send(data);
        });

        th.detach();
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyNode>());
    rclcpp::shutdown();
    return 0;
}
