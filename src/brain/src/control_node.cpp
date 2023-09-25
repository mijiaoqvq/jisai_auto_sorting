#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/item_info.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class ControlNode : public rclcpp::Node {
private:
    enum Status {
        DISC, PILING, PLATFORM
    } status;
    rclcpp::Subscription<interfaces::msg::ItemInfo>::SharedPtr itemInfoSubscription;
public:
    ControlNode() : Node("control") {
        itemInfoSubscription = this->create_subscription<interfaces::msg::ItemInfo>(
                "item_info",
                10,
                [this](const interfaces::msg::ItemInfo::SharedPtr itemInfo) {
                    switch (status) {
                        case DISC:
                            break;
                        case PILING:
                            break;
                        case PLATFORM:
                            break;
                    }
                }
        );
    }

    void run() {

    }

    void disc() {

    }

    void piling() {

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