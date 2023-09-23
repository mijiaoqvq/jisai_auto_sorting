#include <cstdio>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/srv/device_info.hpp"

#include "SerialUltra/thirdparty/serialib/serialib.h"

#define MAX_READ_ONCE_CHAR 400

using namespace std::chrono_literals;

class QrCodeNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr publisher;
    rclcpp::Service<interfaces::srv::DeviceInfo>::SharedPtr service;
    std::shared_ptr<serialib> pSerial;

public:
    QrCodeNode() : Node("qr_code") {
        rclcpp::QoS qoS(10);
        qoS.reliable();

        auto serviceCallBack = [this](const std::shared_ptr<interfaces::srv::DeviceInfo::Request> req,
                                      const std::shared_ptr<interfaces::srv::DeviceInfo::Response>) {
            pSerial = std::make_shared<serialib>();
            pSerial->openDevice(req->port.data(), 115200);
            RCLCPP_DEBUG(this->get_logger(), "Received port: " + req->port);
            if (!pSerial->isDeviceOpen()) {
                RCLCPP_FATAL(this->get_logger(), "Serial port Open Failed!");
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Serial port Open success!");
            }

            publisher = this->create_publisher<example_interfaces::msg::Int32>("qr_code_info", 1);
            timer = this->create_wall_timer(
                    5ms, [this]() {
                        char buffer[MAX_READ_ONCE_CHAR];
                        auto message = example_interfaces::msg::Int32();
                        message.data = 0;
                        int len = pSerial->readBytes(buffer, MAX_READ_ONCE_CHAR, 1);
                        if (len > 0) {
                            if (buffer[0] == 'B') {
                                message.data = 1;
                                RCLCPP_INFO(this->get_logger(), "Scanned Blue!");
                            } else if (buffer[0] == 'R') {
                                message.data = 2;
                                RCLCPP_INFO(this->get_logger(), "Scanned Red!");
                            }
                            publisher->publish(message);
                        }
                    });
        };

        service = this->create_service<interfaces::srv::DeviceInfo>("qr_code_port", serviceCallBack);

    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrCodeNode>());
    rclcpp::shutdown();
    return 0;
}
