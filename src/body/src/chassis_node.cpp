#include <opencv2/core/quaternion.hpp>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/srv/device_info.hpp"
#include "SerialUltra/Communicate.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "interfaces/msg/serial_data.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ArmNode : public rclcpp::Node {
private:
    rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr lightSubscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transformSubscription;
    rclcpp::Subscription<interfaces::msg::Pose>::SharedPtr poseSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ctlSubscription;
    rclcpp::Service<interfaces::srv::DeviceInfo>::SharedPtr service;
    rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr startPublisher;

    rclcpp::Subscription<interfaces::msg::SerialData>::SharedPtr serialDataSubscription;
    rclcpp::Publisher<interfaces::msg::SerialData>::SharedPtr serialDataPublisher;
    Communicate communicate;
public:
    ArmNode() : Node("chassis") {
        communicate.setNode(this);

        auto serviceCallBack = [this](const std::shared_ptr<interfaces::srv::DeviceInfo::Request> req,
                                      const std::shared_ptr<interfaces::srv::DeviceInfo::Response>) {
            RCLCPP_INFO(this->get_logger(), "Received port: " + req->port);
            communicate.registerCallBack(0x11, [this](const Data& data) {
                example_interfaces::msg::Int32 flag;
                flag.data = data.msg[0];
                startPublisher->publish(flag);
            });
            communicate.registerCallBack(0x3F, [this](const Data&) {
                interfaces::msg::SerialData serialData;
                serialData.id = 0x3F;
                serialDataPublisher->publish(serialData);
            });
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
                    auto _rotation = tf->transforms.data()->transform.rotation;
                    cv::Quatf position;
                    position.w = 0;
                    position.x = -translation.x;
                    position.y = -translation.y;
                    position.z = -translation.z;
                    cv::Quatf rotation;
                    rotation.w = _rotation.w;
                    rotation.x = _rotation.x;
                    rotation.y = _rotation.y;
                    rotation.z = _rotation.z;
                    rotation = cv::Quatf(0, 1, 0, 0) * rotation;
                    cv::Vec3f eulerAngle;
                    eulerAngle = rotation.toEulerAngles(cv::QuatEnum::EulerAnglesType::INT_ZXZ);

                    communicate.setTransform({position.x, position.y, eulerAngle[0]});
                }
        );

        poseSubscription = this->create_subscription<interfaces::msg::Pose>(
                "pose",
                10,
                [this](const interfaces::msg::Pose::SharedPtr pose) {
                    communicate.sendChassisPosition(pose->x, pose->y, pose->w);
                }
        );

        serialDataSubscription = this->create_subscription<interfaces::msg::SerialData>(
                "chassis_serial",
                10,
                [this](const interfaces::msg::SerialData::SharedPtr serialData) {
                    Data data = {};
                    memcpy(&data, serialData->data.data(), sizeof(data));
                    communicate.call(serialData->id, data);
                }
        );

        ctlSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
                "/direction_ctl",
                10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    Data data = {};
                    if(msg->linear.x > 0){
                        data.msg[0]+=1;
                        RCLCPP_INFO(this->get_logger(), "up");
                    }
                    if(msg->linear.x < 0){
                        data.msg[0]-=1;
                        RCLCPP_INFO(this->get_logger(), "down");
                    }
                    if(msg->linear.y > 0){
                        data.msg[1]+=1;
                        RCLCPP_INFO(this->get_logger(), "left");
                    }
                    if(msg->linear.y < 0){
                        data.msg[1]-=1;
                        RCLCPP_INFO(this->get_logger(), "right");
                    }
                    if(msg->angular.z > 0){
                        data.msg[2]-=1;
                        RCLCPP_INFO(this->get_logger(), "lt");
                    }
                    if(msg->angular.z < 0){
                        data.msg[2]+=1;
                        RCLCPP_INFO(this->get_logger(), "rt");
                    }
                    communicate.call(0x32,data);
                }
        );

        serialDataPublisher = this->create_publisher<interfaces::msg::SerialData>("chassis_data", 10);

        startPublisher = this->create_publisher<example_interfaces::msg::Int32>("start", 10);

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}