#pragma once

#include <functional>

#include <QtCore/QString>
#include <QtCore/QVariant>

#if 0 //!defined(WIN32) legacy
namespace std {
    template <>
    struct hash<QString> {
        inline std::size_t
        operator()(QString const &s) const {
            return qHash(s);
        }
    };
}
#endif