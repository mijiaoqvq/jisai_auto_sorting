//
// Created by mijiao on 23-10-11.
//

#ifndef BODY_COMMUNICATION_H
#define BODY_COMMUNICATION_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <array>
#include "thirdparty/hidapi/hidapi.h"

using Data = std::array<uint8_t, 64>;

class Communication {
private:
    hid_device* hidDevice = nullptr;
    uint8_t buffer[64] = {};

public:
    Communication() {
        hid_init();
    }

    void open(uint16_t vid, uint16_t pid, wchar_t* serialNumber = nullptr) {
        hidDevice = hid_open(vid, pid, serialNumber);
    }

    void send(Data data) {
        hid_write(hidDevice, data.data(), 64);
    }

    Data read() {
        hid_read(hidDevice, buffer, 64);
        Data data;
        memcpy(data.data(), buffer, 64);
        return data;
    }

    Data read_timeout() {
        int ret = hid_read_timeout(hidDevice, buffer, 64, 5000);
        Data data;
        if (ret) {
            memcpy(data.data(), buffer, 64);
        }
        return data;
    }

    void testConnect() {
        Data data;
        data[0] = 0xFF;
        send(data);
    }

    void lightOn() {
        std::string cmd = "S_CMD_03L2";

        Data data = {};
        data[0] = 0x04;
        data[1] = strlen(cmd.c_str());
        memcpy(data.data() + 2, cmd.data(), data[1]);

        send(data);
    }

    void lightOff(){
        std::string cmd = "S_CMD_03L2";

        Data data = {};
        data[0] = 0x04;
        data[1] = strlen(cmd.c_str());
        memcpy(data.data() + 2, cmd.data(), data[1]);

        send(data);
    }
};


#endif //BODY_COMMUNICATION_H
