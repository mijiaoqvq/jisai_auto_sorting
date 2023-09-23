//
// Created by mijiao on 23-2-21.
//

#ifndef SERIALULTRA_SERIALULTRA_H
#define SERIALULTRA_SERIALULTRA_H

#include <iostream>
#include "serialib.h"
#include "MessageProcessor/OutputMessageProcessor.h"
#include "MessageProcessor/InputMessageProcessor.h"

#define MessageData struct __attribute__((packed))

namespace su {
    template<typename Head, typename Tail>
    class SerialUltra {
    private:
        std::shared_ptr<serialib> pSerial;
        mp::OutputMessageProcessor<Head, Tail> OMP;
        mp::InputMessageProcessor<Head, Tail> IMP;

    public:
        SerialUltra() : pSerial(std::make_shared<serialib>()), OMP(pSerial), IMP(pSerial) {}

        SerialUltra(const std::string& device, int baud) : pSerial(std::make_shared<serialib>()), OMP(pSerial),
                                                           IMP(pSerial) {
            if (open(device, baud) == 1) {
                IMP.start();
            } else {
                std::cout << "[ERROR] open Failed!" << std::endl;
            }
        }

        ~SerialUltra() {
            pSerial->closeDevice();
        }

        bool status() {
            return pSerial->isDeviceOpen();
        }

        char open(const std::string& device, int baud) {
            if (!pSerial->isDeviceOpen()) {
                char ret = pSerial->openDevice(device.c_str(), baud);
                if (ret == 1) {
                    IMP.start();
                }
                return ret;
            } else {
                return 0;
            }
        }

        void close() {
            IMP.stop();
            pSerial->closeDevice();
        }

        void spinOnce(std::chrono::duration<int64_t> duration) {
            IMP.spinOnce(duration);
        }

        void spin(bool background = false) {
            IMP.spin(background);
        }

        bool setMaxSize(size_t _maxSize) {
            return IMP.setMaxSize(_maxSize);
        }

        void registerChecker(std::function<int(const Head&)> checker) {
            IMP.registerChecker(checker);
        }

        void registerChecker(std::function<int(const Tail&, const uint8_t*, int)> checker) {
            IMP.registerChecker(checker);
        }

        void setGetLength(std::function<size_t(const Head&)> _getLength) {
            IMP.setGetLength(_getLength);
        }

        void setGetId(std::function<int(const Head&)> _getID) {
            IMP.setGetId(_getID);
        }

        template<typename functionType>
        void registerCallBack(size_t id, functionType callback) {
            IMP.registerCallBack(id, callback);
        }

        template<typename functionType>
        void registerHeadPreprocessor(functionType headPreprocessor) {
            OMP.registerHeadPreprocessor(headPreprocessor);
        }

        template<typename functionType>
        void registerTailPreprocessor(functionType tailPreprocessor) {
            OMP.registerTailPreprocessor(tailPreprocessor);
        }

        template<typename Data>
        int write(Head head, Data data, Tail tail) {
            return OMP.write(head, data, tail);
        }

    };
}

#endif //SERIALULTRA_SERIALULTRA_H
