#ifndef CM_EXTRACT_MIN_THICKNESS_ALONG_AXIS_HPP
#define CM_EXTRACT_MIN_THICKNESS_ALONG_AXIS_HPP

#include <algorithm>
#include <array>
#include <limits>
#include <type_traits>

#include "cmKDTree.hpp"
#include "cmPointLocator.hpp"

// Extracts
template <class T, int D, class LocatorType = cmKDTree<T, D, 1>>
class cmExtractMinThicknessAlongAxis {
  using PointType = std::array<T, 1>;
  using VectorType = std::array<T, 3>;
  static_assert(
      std::is_base_of<cmPointLocator<T, D, 1>, LocatorType>::value,
      "The passed point locator is not a subclass of cmPointLocator or does "
      "not work along a single axis.");

  const int axis;
  const T gridDelta;
  std::array<T, 2> bounds;
  const T direction;
  const unsigned N;

  lsSmartPointer<std::vector<VectorType>> points = nullptr;
  lsSmartPointer<std::vector<T>> dataDestination = nullptr;

  static constexpr T euclideanDistance(const VectorType &a,
                                       const VectorType &b) {
    T sum{0};
    for (int i = 0; i < D; ++i) {
      T d = b[i] - a[i];
      sum += d * d;
    }
    return std::sqrt(sum);
  }

public:
  cmExtractMinThicknessAlongAxis(
      lsSmartPointer<std::vector<std::array<T, 3>>> passedPoints,
      lsSmartPointer<std::vector<T>> passedDataDestination, int passedAxis,
      std::array<T, 2> passedBounds, T passedGridDelta)
      : axis(passedAxis), gridDelta(passedGridDelta), bounds(passedBounds),
        direction((bounds[0] <= bounds[1]) ? 1.0 : -1.0),
        N(std::abs(bounds[0] - bounds[1]) / gridDelta + 1),
        points(passedPoints), dataDestination(passedDataDestination) {}

  void apply() {
    // Stop if the provided axis is invalid.
    if (axis >= D)
      return;

    if (points == nullptr || dataDestination == nullptr)
      return;

    std::vector<PointType> zPoints;
    zPoints.reserve(points->size());

    // According to https://www.quick-bench.com/q/iJlxQe6oq0N8kQ0GO32Piijf464
    // this should be a little bit faster than a simple for loop, at least on
    // GCC
    std::generate_n(std::back_inserter(zPoints), points->size(),
                    [&, i = 0]() mutable {
                      return std::move(std::array<T, 1>{(*points)[i++][axis]});
                    });

    auto tree = lsSmartPointer<LocatorType>::New(zPoints);
    tree->build();

    for (unsigned i = 0; i < N; ++i) {
      PointType x = {bounds[0] + direction * i * gridDelta};

      auto nearest = tree->findNearestWithinRadius(x, gridDelta);

      VectorType xv{0.};
      xv[axis] = x[0];

      T dist = std::numeric_limits<T>::infinity();
      if (nearest->size() > 0) {
        for (const auto n : *nearest) {
          T d = euclideanDistance((*points)[n.first], xv);
          if (d < dist)
            dist = d;
        }
      }

      dataDestination->push_back(dist);
    }
  }
};
#endif