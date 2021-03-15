#pragma once

#include <functional>


namespace impl { namespace Publisher {
    class Publisher {
    public:
        Publisher(const std::type_info& type) : type_{type} {

        }

        template<class T>
        void publish(const T&) {
            assert(type_ == typeid(T));

        }

        protected:
            const std::type_info& type_;
    };

    template<class T>
    std::shared_ptr<Publisher> create(const std::string& topic, int qSize, bool latched) {
        auto pub = std::make_shared<Publisher>(typeid(T));
        return pub;
    }

} }