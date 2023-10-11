//
// Created by mijiao on 23-10-11.
//
#include <iostream>
#include "body/Communication.h"

int main() {
    Communication communication;
    int hid, pid;
    //std::cin >> hid >> pid;
    hid = 0x26f1;
    pid = 0x8803;
    communication.open(hid, pid);
    while (true) {
        Data data = communication.read();
        for(auto& d : data){
            std::cout<<(int)d<<" ";
        }
        std::cout<<std::endl;
    }
}