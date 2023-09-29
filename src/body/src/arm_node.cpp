#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/serial_data.hpp"
#include "interfaces/msg/arm_pose.hpp"
#include "interfaces/srv/device_info.hpp"
#include "SerialUltra/Communicate.h"

class ArmNode : public rclcpp::Node {
private:
    rclcpp::Subscription<interfaces::msg::SerialData>::SharedPtr serialDataSubscription;
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr lightSubscription;
    rclcpp::Service<interfaces::srv::DeviceInfo>::SharedPtr service;
    rclcpp::Subscription<interfaces::msg::ArmPose>::SharedPtr positionSubscription;

    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr startPublisher;
    Communicate communicate;
public:
    ArmNode() : Node("arm") {
        communicate.setNode(this);
        communicate.registerCallBack(0x11, [this](const Data&) {
            example_interfaces::msg::Int32 flag;
            startPublisher->publish(flag);
            RCLCPP_WARN(this->get_logger(), "START");
        });

        communicate.registerCallBack(0x73, [this](const Data& data) {
            switch (data.msg[0]) {
                case 1:

                    break;
                case 2:

                    break;
                case 3:

                    break;
            }
        });

        auto serviceCallBack = [this](const std::shared_ptr<interfaces::srv::DeviceInfo::Request> req,
                                      const std::shared_ptr<interfaces::srv::DeviceInfo::Response>) {
            RCLCPP_DEBUG(this->get_logger(), "Received port: " + req->port);
            communicate.open(req->port, 115200);
            communicate.spin(true);
        };

        service = this->create_service<interfaces::srv::DeviceInfo>("arm_port", serviceCallBack);

        serialDataSubscription = this->create_subscription<interfaces::msg::SerialData>(
                "arm_serial",
                10,
                [this](const interfaces::msg::SerialData::SharedPtr serialData) {
                    Data data = {};
                    memcpy(&data, serialData->data.data(), sizeof(data));
                    communicate.call(serialData->id, data);
                }
        );

        lightSubscription = this->create_subscription<example_interfaces::msg::Int32>(
                "qr_code_info",
                10,
                [this](const example_interfaces::msg::Int32::SharedPtr color) {
                    RCLCPP_DEBUG(this->get_logger(), "Received color info: " + std::to_string(color->data));
                    if (color->data == 1) {
                        communicate.changeLight(2);
                    } else if (color->data == 2) {
                        communicate.changeLight(0);
                    }
                }
        );

        positionSubscription = this->create_subscription<interfaces::msg::ArmPose>(
                "arm_position",
                10,
                [this](const interfaces::msg::ArmPose::SharedPtr pose) {
                    communicate.sendArmPosition(pose->x);
                }
        );

        startPublisher = this->create_publisher<example_interfaces::msg::Int32>("start", 10);

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}