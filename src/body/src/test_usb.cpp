//
// Created by mijiao on 23-10-11.
//
#include <iostream>
#include "body/Communication.h"

int main() {
    Communication communication;
    int hid, pid;
    //std::cin >> hid >> pid;
    hid = 0x114;
    pid = 0x514;
    communication.open(hid, pid);
    while (true) {
        Data data = {};
        int a,b;
        std::cin >> a >> b;
        data[0] = a;
        data[1] = b;
        communication.send(data);
    }
}