//
// Created by mijiao on 23-2-21.
//

#ifndef SERIALULTRA_OUTPUTMESSAGEPROCESSOR_H
#define SERIALULTRA_OUTPUTMESSAGEPROCESSOR_H

#include <cstring>
#include <functional>
#include <list>
#include <memory>
#include <vector>

#include "serialib.h"


namespace mp {
    template<typename Head, typename Tail>
    class OutputMessageProcessor {
    private:
        std::list<std::function<void(Head&, size_t)>> headPreprocessorList;
        std::list<std::function<void(Tail&, const uint8_t*/*数据指针*/, size_t/*头和数据长度*/)>> tailPreprocessorList;
        std::shared_ptr<serialib> pSerial;

        template<typename pFunc>
        struct lambda_type;

        template<typename Lambda, typename... Args>
        struct lambda_type<void (Lambda::*)(Args...) const> {
            using type = std::function<void(Args...)>;
        };

        template<typename Data>
        std::string serialize(Head head, Data data, Tail tail) {
            std::string message;
            message.resize(sizeof(Head) + sizeof(Data) + sizeof(Tail));
            for (auto headPreprocessor: headPreprocessorList) {
                headPreprocessor(head, sizeof(Data));
            }
            memcpy((void*) message.data(), &head, sizeof(Head));
            memcpy((void*)(message.data() + sizeof(Head)), &data, sizeof(Data));
            for (auto tailPreprocessor: tailPreprocessorList) {
                tailPreprocessor(tail, (uint8_t*) message.data(), sizeof(Head) + sizeof(Data));
            }
            memcpy((void*)(message.data() + sizeof(Head) + sizeof(Data)), &tail, sizeof(Tail));
            return message;

        }

    public:
        OutputMessageProcessor() = default;

        explicit OutputMessageProcessor(std::shared_ptr<serialib>& _pSerial) {
            pSerial = _pSerial;
        }

        OutputMessageProcessor(OutputMessageProcessor& other) = delete;

        OutputMessageProcessor(OutputMessageProcessor&& other) noexcept = default;

        ~OutputMessageProcessor() = default;

        OutputMessageProcessor& operator=(OutputMessageProcessor&) = delete;

        void registerHeadPreprocessor(std::function<void(Head&, size_t)> headPreprocessor) {
            headPreprocessorList.push_back(headPreprocessor);
        }

        template<typename Lambda, typename Func = typename lambda_type<decltype(&Lambda::operator())/*lambda的函数指针的类型*/>::type>
        void registerHeadPreprocessor(Lambda callback) {
            registerHeadPreprocessor(Func(callback));
        }

        void registerTailPreprocessor(std::function<void(Tail&, const uint8_t*, size_t)> tailPreprocessor) {
            tailPreprocessorList.push_back(tailPreprocessor);
        }

        template<typename Lambda, typename Func = typename lambda_type<decltype(&Lambda::operator())/*lambda的函数指针的类型*/>::type>
        void registerTailPreprocessor(Lambda callback) {
            registerTailPreprocessor(Func(callback));
        }

        template<typename Data>
        int write(Head head, Data data, Tail tail){
            std::string message = serialize(head, data, tail);
            return pSerial->writeBytes(message.data(),message.size());
        }

    };
}
#endif //SERIALULTRA_OUTPUTMESSAGEPROCESSOR_H
