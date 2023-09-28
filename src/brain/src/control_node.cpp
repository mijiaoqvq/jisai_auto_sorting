#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/item_info.hpp"
#include "interfaces/msg/serial_data.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

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
public:
    ControlNode() : Node("control") {
        itemInfoSubscription = this->create_subscription<interfaces::msg::ItemInfo>(
                "item_info",
                10,
                [this](const interfaces::msg::ItemInfo::SharedPtr itemInfo) {
                    interfaces::msg::SerialData serialData;
                    switch (status) {
                        case DISC:
                            if (abs(itemInfo->y - 0.5) < 0.1){
                                serialData.id = 0x72;
                                switch (itemInfo->id) {
                                    case RED_BOX:
                                    case RED_CUBE:
                                    case RED_SPHERE:
                                    case RED_TUBE:
                                        if(color==RED){
                                            serialData.data[0] = 1;
                                            armSerialDataPublisher->publish(serialData);
                                        }
                                        break;
                                    case BLUE_CUBE:
                                    case BLUE_SPHERE:
                                    case BLUE_TUBE:
                                    case BLUE_BOX:
                                        if(color==BLUE){
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
                                serialData.data[0] = 1;
                                armSerialDataPublisher->publish(serialData);
                            }
                                break;
                        case PILING:
                            break;
                        case PLATFORM:
                            break;
                    }
                }
        );
        disc();
    }

    void run() {

    }

    void disc() {
        interfaces::msg::SerialData serialData;
        serialData.id = 0x72;
        serialData.data[0] = 0;
        armSerialDataPublisher->publish(serialData);
        status = DISC;
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