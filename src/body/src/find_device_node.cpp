#include <cstdio>

#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/device_info.hpp"

#include "SerialUltra/Communicate.h"

#define findService(client) (findServiceImpl(client, #client))

#define PASSWORD ""

using namespace std::chrono_literals;

class FindDevice : public rclcpp::Node {
private:
    rclcpp::Client<interfaces::srv::DeviceInfo>::SharedPtr armClient;
    rclcpp::Client<interfaces::srv::DeviceInfo>::SharedPtr chassisClient;
    rclcpp::Client<interfaces::srv::DeviceInfo>::SharedPtr qrCodeClient;

    template<class Client>
    void findServiceImpl(Client client, const std::string& s) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The service of [%s] is not available, waiting again...",
                        s.c_str());
        }
    }

public:
    FindDevice() : Node("find_device") {
        qrCodeClient = this->create_client<interfaces::srv::DeviceInfo>("qr_code_port");
        armClient = this->create_client<interfaces::srv::DeviceInfo>("arm_port");
        chassisClient = this->create_client<interfaces::srv::DeviceInfo>("chassis_port");

        findService(qrCodeClient);
        findService(armClient);
        findService(chassisClient);

        std::string s = "/dev/ttyACM";
        for (int i = 0; i < 3; i++) {
            std::string port = s + std::to_string(i);
            Communicate communicate(port, 115200);
            communicate.setNode(this);

            if (!communicate.status()) {
                RCLCPP_WARN(this->get_logger(), "Serial port " + port + " Open failed! Retrying...");
                std::stringstream cmd;
                cmd << "echo " << PASSWORD << " | sudo -S chmod 666 " << port;
                system(cmd.str().c_str());
                communicate.open(port, 115200);
                if (!communicate.status()) {
                    RCLCPP_FATAL(this->get_logger(), "Retrying failed! Skipping!");
                    continue;
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "Serial port " + port + " Open success!");
            auto info = std::make_shared<interfaces::srv::DeviceInfo::Request>();
            info->port = port;
            int id = communicate.askID();
            if (id == 0) {
                qrCodeClient->async_send_request(info);
                RCLCPP_INFO(this->get_logger(), "Found qrCode!");
            } else if (id == 0x22) {
                chassisClient->async_send_request(info);
                RCLCPP_INFO(this->get_logger(), "Found chassis!");
            } else if (id == 0x21) {
                armClient->async_send_request(info);
                RCLCPP_INFO(this->get_logger(), "Found arm!");

            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindDevice>());
    rclcpp::shutdown();
    return 0;
}
