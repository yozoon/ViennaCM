#ifndef CM_POINT_LOCATOR_HPP
#define CM_POINT_LOCATOR_HPP

#include <array>
#include <utility> // std::pair
#include <vector>  // std::vector

#include <lsSmartPointer.hpp>

template <class NumericType, int D, int Dim = D> struct cmPointLocator {
  using VectorType = std::array<NumericType, Dim>;
  using SizeType = std::size_t;

  cmPointLocator() {}

  virtual void build() = 0;

  virtual std::pair<SizeType, NumericType>
  findNearest(const VectorType &x) const = 0;

  virtual lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>
  findKNearest(const VectorType &x, const int k) const = 0;

  virtual lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>
  findNearestWithinRadius(const VectorType &x,
                          const NumericType radius) const = 0;
};
#endif