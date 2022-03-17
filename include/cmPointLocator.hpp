#ifndef CM_POINT_LOCATOR_HPP
#define CM_POINT_LOCATOR_HPP

#include <tuple>   // std::tuple_size
#include <utility> // std::pair
#include <vector>  // std::vector

#include <lsSmartPointer.hpp>

template <class VectorType> struct cmPointLocator {
  using SizeType = std::size_t;
  using T = typename VectorType::value_type;
  using DistanceFunctionType = T (*)(const VectorType &, const VectorType &);

  static constexpr int D = std::tuple_size<VectorType>();

  cmPointLocator() {}

  virtual void build() = 0;

  virtual std::pair<SizeType, T> findNearest(const VectorType &x) const = 0;

  virtual lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findKNearest(const VectorType &x, const int k) const = 0;

  virtual lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findNearestWithinRadius(const VectorType &x, const T radius) const = 0;
};
#endif