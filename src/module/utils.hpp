#pragma once
#include <boost/range/algorithm/find.hpp>
#include <boost/range/end.hpp>
#include <string>

namespace utils {

template <typename ContainerT>
inline bool contains(std::string const &str, ContainerT const &container) {
  return boost::range::find(container, str) != boost::end(container);
}

} // namespace utils
