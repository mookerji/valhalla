#ifndef MMP_DEMO_UTILS_H_
#define MMP_DEMO_UTILS_H_

#include <functional>

namespace valhalla {

namespace matching {

template <class T> inline void hash_combine(std::size_t& seed, const T& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_UTILS_H_
