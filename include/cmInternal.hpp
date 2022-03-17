#ifndef CM_INTERNAL_HPP
#define CM_INTERNAL_HPP

#include <cmath>

namespace cmInternal {
constexpr int intLog2(int x) {
  int val = 0;
  while (x >>= 1)
    ++val;
  return val;
}

/****************************************************************************
 * Distance Functions                                                       *
 ****************************************************************************/
template <class VectorType>
constexpr typename VectorType::value_type
manhattanReducedDistance(const VectorType &a, const VectorType &b) {
  constexpr int D = std::tuple_size<VectorType>();

  typename VectorType::value_type sum{0};
  for (int i = 0; i < D; ++i)
    sum += std::abs(b[i] - a[i]);

  return sum;
}

template <class VectorType>
constexpr typename VectorType::value_type
euclideanReducedDistance(const VectorType &a, const VectorType &b) {
  constexpr int D = std::tuple_size<VectorType>();

  typename VectorType::value_type sum{0};
  for (int i = 0; i < D; ++i) {
    typename VectorType::value_type d = b[i] - a[i];
    sum += d * d;
  }
  return sum;
}

template <class VectorType>
constexpr typename VectorType::value_type
manhattanDistance(const VectorType &a, const VectorType &b) {
  return manhattanReducedDistance(a, b);
}

template <class VectorType>
constexpr typename VectorType::value_type
euclideanDistance(const VectorType &a, const VectorType &b) {
  return std::sqrt(euclideanReducedDistance(a, b));
}
} // namespace cmInternal
#endif