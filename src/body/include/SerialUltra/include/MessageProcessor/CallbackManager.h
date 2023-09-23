//
// Created by mijiao on 23-2-21.
//

#ifndef SERIALULTRA_CALLBACKMANAGER_H
#define SERIALULTRA_CALLBACKMANAGER_H

#include <map>
#include <functional>

namespace mp {
    class CallbackManager {
    private:
        std::map<size_t, std::function<void(const uint8_t*)>> callbackMap;
    public:
        bool registerCallback(size_t id, std::function<void(const uint8_t*)>& callback) {
            if (callbackMap.find(id) != callbackMap.end()) {
                return false;
            } else {
                callbackMap[id] = std::move(callback);
                return true;
            }

        }

        std::function<void(const uint8_t*)> operator[](size_t id) {
            auto it = callbackMap.find(id);
            if (it != callbackMap.end()) {
                return it->second;
            } else {
                return [](const uint8_t*) {};
            }
        }
    };
}

#endif //SERIALULTRA_CALLBACKMANAGER_H
