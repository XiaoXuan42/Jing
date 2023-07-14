#pragma once
#include <map>
#include <memory>

#include "JsonUtil.h"

#define REGISTER_CLASS(CLASS, CLASS_NAME)                                      \
    CLASS *CLASS##_helper(const Json &json) { return new CLASS(json); }        \
    static struct CLASS##_ {                                                   \
        CLASS##_() {                                                           \
            if (Factory::getMap().count(CLASS_NAME) != 0) {                    \
                std::cerr << "Fatal, multiple definition class " << CLASS_NAME \
                          << std::endl;                                        \
                std::exit(1);                                                  \
            }                                                                  \
            Factory::register_class(CLASS_NAME, CLASS##_helper);               \
        }                                                                      \
    } CLASS##__;

class Factory {
public:
    Factory() = delete;

    static void register_class(
        const std::string &name,
        const std::function<void *(const Json &json)> &constructor) {
        // constructor_map[name] = constructor;
        getMap().emplace(std::make_pair(name, constructor));
    }

    template <typename T>
    static std::unique_ptr<T> construct_class_unique(const Json &json) {
        std::string type = fetchRequired<std::string>(json, "type");
        if (getMap().count(type) == 0) {
            std::cerr << "Fatal, unknown type " << type << std::endl;
            std::exit(1);
        }
        T *p = static_cast<T *>(getMap()[type](json));
        return std::unique_ptr<T>(p);
    }

    template <typename T>
    static std::shared_ptr<T> construct_class(const Json &json) {
        return std::shared_ptr<T>(std::move(construct_class_unique<T>(json)));
    }

    static std::map<std::string, std::function<void *(const Json &)>> &
    getMap() {
        static std::map<std::string, std::function<void *(const Json &)>>
            constructor_map = {};
        return constructor_map;
    }
};