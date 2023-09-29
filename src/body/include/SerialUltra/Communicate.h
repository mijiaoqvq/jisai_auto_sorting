//
// Created by mijiao on 23-9-6.
//

#ifndef SERIALULTRA_COMMUNICATE_H
#define SERIALULTRA_COMMUNICATE_H

#include "SerialUltra.h"

MessageData Data {
    uint8_t msg[59];
};
MessageData Head {
    uint8_t head1 = 0xF4;
    uint8_t head2 = 0xF1;
    uint8_t id = 0xFF;
    uint8_t length = sizeof(Data);
};
MessageData Tail {
    uint8_t sum;
};

class Communicate : public su::SerialUltra<Head, Tail> {
private:
    uint8_t id_{};
    MessageData Transform {
        float x;
        float y;
        float rotation;
    } transform;

    rclcpp::Node* node;

    void init() {
        registerChecker([](const Head& head) { return !(head.head1 == 0xF4 && head.head2 == 0xF1); });
        registerChecker([](const Tail& tail, const uint8_t* p, int len) {
            uint8_t sum = 0;
            for (int i = 0; i < len; i++) {
                sum += p[i];
            }
            return sum != tail.sum;
        });
        setGetLength([](const Head& head) { return head.length; });
        setGetId([](const Head& head) { return head.id; });
        registerTailPreprocessor([](Tail& tail, const uint8_t* p, size_t len) {
            uint8_t sum = 0;
            for (size_t i = 0; i < len; i++) {
                sum += p[i];
            }
            tail.sum = sum;
        });

        registerCallBack(0xFF, [this](const Data& data) { id_ = data.msg[0]; });
        registerCallBack(0x30, [this](const Data&) {
            RCLCPP_DEBUG(node->get_logger(), "Received location request!");
            sendLocation();
        });
    }

public:
    Communicate() {
        init();
    }

    Communicate(const std::string& device, int baud) : su::SerialUltra<Head, Tail>(device, baud) {
        init();
    }

    void setNode(rclcpp::Node* _node) {
        node = _node;
    }

    void call(uint8_t id, Data msg = {}) {
        Head head{};
        Tail tail{};
        head.id = id;
        std::stringstream ss;
        std::string info;
        for (auto& byte: msg.msg) {
            std::string temp;
            ss << std::hex << (int) byte;
            ss >> temp;
            ss.clear();
            info += temp + " ";
        }
        RCLCPP_DEBUG(node->get_logger(), "%s", info.c_str());
        write(head, msg, tail);
    }

    int askID() {
        using namespace std::chrono_literals;
        call(0xFF);
        spinOnce(5s);
        return id_;
    }

    void changeLight(int color) {
        call(color);
    }

    void sendLocation() {
        Data data = {};
        memcpy(&data, &transform, sizeof(transform));
        call(0x30, data);
//        RCLCPP_INFO(node->get_logger(), "Sent location success! %f %f %f", transform.x, transform.y,
//                    transform.rotation);
    }

    void setTransform(Transform _transform) {
        transform = _transform;
    }

    void sendArmPosition(float dx) {
        Data data = {};
        memcpy(&data.msg[0], &dx, sizeof(float));
        call(0x72, data);
        RCLCPP_INFO(node->get_logger(), "Sent arm position success! %f %f", dx);
    }

    void sendChassisPosition(float dx, float dy, float dTheta) {
        Data data = {};
        memcpy(&data.msg[0], &dx, sizeof(float));
        memcpy(&data.msg[4], &dy, sizeof(float));
        memcpy(&data.msg[8],&dTheta,sizeof(float));
        call(0x31, data);
        RCLCPP_INFO(node->get_logger(), "Sent chassis position success! %f %f %f", dx, dy, dTheta);
    }

    void sendChassisVelocity(float vx, float vy, float w) {
        Data data = {};
        memcpy(&data.msg[0], &vx, sizeof(float));
        memcpy(&data.msg[4], &vy, sizeof(float));
        memcpy(&data.msg[8],&w,sizeof(float));
        call(0x32, data);
        RCLCPP_INFO(node->get_logger(), "Sent chassis position success! %f %f %f", vx, vy, w);
    }

    void reachArm(){
        Data data = {};
        data.msg[0] = 1;
        call(0x73, data);
    }
    
};

#endif //SERIALULTRA_COMMUNICATE_H
