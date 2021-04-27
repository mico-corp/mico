#pragma once

#include <functional>

#include <QtCore/QString>
#include <QtCore/QVariant>



// #if defined(_WIN32)

namespace std
{
template<>
struct hash<QString>
{
  inline std::size_t
  operator()(QString const &s) const
  {
    return qHash(s);
  }
};
}
// #endif