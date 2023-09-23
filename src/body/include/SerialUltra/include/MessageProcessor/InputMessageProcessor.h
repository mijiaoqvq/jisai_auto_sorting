//
// Created by mijiao on 23-2-21.
//

#ifndef SERIALULTRA_INPUTMESSAGEPROCESSOR_H
#define SERIALULTRA_INPUTMESSAGEPROCESSOR_H

#define MAX_READ_ONCE_CHAR 400

#include <atomic>
#include <functional>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "serialib.h"
#include "MessageProcessor/CallbackManager.h"


namespace mp {
    template<typename Head, typename Tail>
    class InputMessageProcessor {
        enum readReturnCode {
            noGetID = -3,
            noGetLength = -2,
            bufferOverflow = -1
        };


    private:
        size_t maxSize = 1024;
        std::string rxBuffer;
        CallbackManager callbackManager;
        std::list<std::function<int(const Head&)>> headCheckerList;
        std::list<std::function<int(const Tail&, const uint8_t*, size_t)>> tailCheckerList;
        std::function<size_t(const Head&)> getLength;
        std::function<size_t(const Head&)> getID;
        std::thread readThread;
        std::atomic<bool> running{false};
        std::shared_ptr<serialib> pSerial;

        template<typename pFunc>
        struct lambda_type;

        template<typename Lambda, typename... Args>
        struct lambda_type<void (Lambda::*)(Args...) const> {
            using type = std::function<void(Args...)>;
        };

        int read() {
            if (rxBuffer.size() > maxSize) {
                return bufferOverflow;
            }
            char buffer[MAX_READ_ONCE_CHAR];
            int len = pSerial->readBytes(buffer, MAX_READ_ONCE_CHAR, 1);
            if (len > 0) {
                if (!getLength) {
                    std::cout << "[ERROR] Function getLength do not be registered!" << std::endl;
                    return noGetLength;
                }
                if (!getID) {
                    std::cout << "[ERROR] Function getID do not be registered!" << std::endl;
                    return noGetID;
                }

                Head head;
                Tail tail;
                int headCheckStatus = 0;
                int tailCheckStatus = 0;
                size_t eraseSize = 0;
                int dataNum = 0;
                rxBuffer.append(buffer, len);
                for (size_t i = 0; i < rxBuffer.size(); i++) {
                    bool failed = false;
                    uint8_t* p = (uint8_t*) rxBuffer.data() + i;

                    memcpy(&head, p, sizeof(Head));
                    for (auto headCheck: headCheckerList) {
                        headCheckStatus = headCheck(head);
                        if (headCheckStatus) {
                            failed = true;
                            break;
                        }
                    }

                    if (failed) {
                        eraseSize = i + 1;
                        continue;
                    }

                    ;

                    size_t dataLength = getLength(head);

                    memcpy(&tail, p + sizeof(Head) + dataLength, sizeof(Tail));
                    for (auto tailCheck: tailCheckerList) {
                        tailCheckStatus = tailCheck(tail, p, sizeof(Head) + dataLength);
                        if (tailCheckStatus) {
                            failed = true;
                            break;
                        }
                    }
                    if (failed) {
                        eraseSize = i + 1;
                        continue;
                    }

                    size_t id = getID(head);
                    callbackManager[id](p + sizeof(Head));
                    dataNum++;
                }
                rxBuffer.erase(0, eraseSize);
                return dataNum;
            }

            return 0;
        }

        void readLoop() {
            while (running) {
                read();
            }
        }

        void _registerCallBack(size_t id, std::function<void(const uint8_t*)> callback) {
            callbackManager.registerCallback(id, callback);
        }

    public:
        InputMessageProcessor() = default;

        explicit InputMessageProcessor(std::shared_ptr<serialib>& _pSerial) {
            pSerial = _pSerial;
        }

        InputMessageProcessor(InputMessageProcessor& other) = delete;

        InputMessageProcessor(InputMessageProcessor&& other) noexcept = default;

        ~InputMessageProcessor() = default;

        InputMessageProcessor& operator=(InputMessageProcessor&) = delete;

        void start() {
            running = true;
        }

        void stop() {
            running = false;
            if (readThread.joinable()) {
                readThread.join();
            }
        }

        void spinOnce(std::chrono::duration<int64_t> duration) {
            bool waiting = true;

            std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
            while (waiting && (std::chrono::steady_clock::now() - beginTime) < duration) {
                int len = pSerial->available();
                if (len != 0) {
                    waiting = false;
                }
            }
            while (running) {
                int len = pSerial->available();
                if (len == 0) {
                    break;
                } else {
                    read();
                }
            }
        }

        void spin(bool background = false) {
            if (background) {
                readThread = std::thread(&InputMessageProcessor::readLoop, this);
            } else {
                readLoop();
            }
        }

        bool setMaxSize(size_t _maxSize) {
            if (_maxSize <= 0) return false;
            maxSize = _maxSize;
            return true;
        }

        void registerChecker(std::function<int(const Head&)> checker) {
            headCheckerList.push_back(checker);
        }

        void registerChecker(std::function<int(const Tail&, const uint8_t*, int)> checker) {
            tailCheckerList.push_back(checker);
        }

        void setGetLength(std::function<size_t(const Head&)> _getLength) {
            getLength = _getLength;
        }

        void setGetId(std::function<int(const Head&)> _getID) {
            getID = _getID;
        }

        template<typename Data>
        void registerCallBack(size_t id, std::function<void(const Data&)> callback) {
            std::function<void(const uint8_t*)> callbackRaw = [callback](const uint8_t* pData) {
                Data data;
                memcpy((void*) &data, (void*) pData, sizeof(Data));
                callback(data);
            };
            _registerCallBack(id, callbackRaw);
        }

        template<typename Data>
        void registerCallBack(size_t id, void(* callback)(const Data&)) {
            registerCallBack(id, std::function<void(const Data&)>(callback));
        }

        template<typename Lambda, typename Func = typename lambda_type<decltype(&Lambda::operator())/*lambda的函数指针的类型*/>::type>
        void registerCallBack(size_t id, Lambda callback) {
            registerCallBack(id, Func(callback));
        }

    };
}
#endif //SERIALULTRA_INPUTMESSAGEPROCESSOR_H
