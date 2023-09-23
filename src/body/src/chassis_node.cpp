#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/srv/device_info.hpp"
#include "SerialUltra/Communicate.h"
#include "tf2_msgs/msg/tf_message.hpp"

class ArmNode : public rclcpp::Node {
private:
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr lightSubscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transformSubscription;
    rclcpp::Subscription<interfaces::msg::Pose>::SharedPtr poseSubscription;
    rclcpp::Service<interfaces::srv::DeviceInfo>::SharedPtr service;
    Communicate communicate;
public:
    ArmNode() : Node("chassis") {
        communicate.setNode(this);

        auto serviceCallBack = [this](const std::shared_ptr<interfaces::srv::DeviceInfo::Request> req,
                                      const std::shared_ptr<interfaces::srv::DeviceInfo::Response>) {
            RCLCPP_INFO(this->get_logger(), "Received port: " + req->port);
            communicate.open(req->port, 115200);
            communicate.spin(true);
        };

        service = this->create_service<interfaces::srv::DeviceInfo>("chassis_port", serviceCallBack);

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

        transformSubscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "tf",
                10,
                [this](const tf2_msgs::msg::TFMessage::SharedPtr tf) {
                    auto translation = tf->transforms.data()->transform.translation;
                    auto rotation = tf->transforms.data()->transform.rotation;

                    communicate.setTransform({(float) translation.x,
                                              (float) translation.y,
                                              (float) (atan2(2 * (rotation.x * rotation.w +
                                                                  rotation.y * rotation.z),
                                                             1 - 2 * (rotation.z * rotation.w +
                                                                      rotation.z * rotation.w)))});
                }
        );

        poseSubscription = this->create_subscription<interfaces::msg::Pose>(
                "pose",
                10,
                [this](const interfaces::msg::Pose::SharedPtr pose) {
                    communicate.sendChassisPosition(pose->x, pose->y, pose->w);
                }
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}