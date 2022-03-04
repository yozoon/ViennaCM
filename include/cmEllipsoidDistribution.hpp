#ifndef CM_ELLIPSOID_DISTRIBUTION_HPP
#define CM_ELLIPSOID_DISTRIBUTION_HPP

#include <algorithm>
#include <cmath>

#include <lsGeometricAdvectDistributions.hpp>

template <class T, int D>
class cmEllipsoidDistribution : public lsGeometricAdvectDistribution<T, D> {
public:
  const T a = 0.;
  const T b = 0.;
  const T c = 0.;

  const T a2;
  const T b2;
  const T c2;

  const T radius = 0.;
  const T gridDelta;

  cmEllipsoidDistribution(const T passedA, const T passedB, const T passedC,
                          const T delta)
      : a(passedA), b(passedB), c(passedC), a2(a * a), b2(b * b), c2(c * c),
        radius(std::max({a, b, c})), gridDelta(delta) {}

  bool isInside(const std::array<hrleCoordType, 3> &initial,
                const std::array<hrleCoordType, 3> &candidate,
                double eps = 0.) const override {
    hrleCoordType dot = 0.;
    for (unsigned i = 0; i < D; ++i) {
      double tmp = candidate[i] - initial[i];
      dot += tmp * tmp;
    }

    if (std::sqrt(dot) <= std::abs(radius) + eps)
      return true;
    else
      return false;
  }

  T getSignedDistance(const std::array<hrleCoordType, 3> &initial,
                      const std::array<hrleCoordType, 3> &candidate,
                      unsigned long /*initialPointId*/) const override {
    T distance = std::numeric_limits<T>::max();
    std::array<hrleCoordType, 3> v{};
    for (unsigned i = 0; i < D; ++i)
      v[i] = candidate[i] - initial[i];

    if (std::abs(radius) <= gridDelta) {
      distance =
          std::max(std::max(std::abs(v[0]), std::abs(v[1])), std::abs(v[2])) -
          std::abs(radius);
    } else {
      T distX = std::numeric_limits<T>::max();
      T distY = std::numeric_limits<T>::max();
      T distZ = std::numeric_limits<T>::max();

      T x = v[0];
      T y = v[1];
      T z = v[2];

      T x2 = x * x;
      T y2 = y * y;
      T z2 = z * z;

      T xx = x2 / a2;
      T yy = y2 / b2;
      T zz = z2 / c2;

      if (y2 <= b2 * (1. - zz) && z2 <= c2 * (1. - yy))
        distX = std::abs(x) - a * std::sqrt(1. - yy - zz);

      if (x2 <= a2 * (1. - zz) && z2 <= c2 * (1. - xx))
        distY = std::abs(y) - b * std::sqrt(1. - xx - zz);

      if (x2 <= a2 * (1. - yy) && y2 <= b2 * (1. - xx))
        distZ = std::abs(z) - c * std::sqrt(1. - xx - yy);

      distance = std::min({distX, distY, distZ}, [](const T x, const T y) {
        return std::abs(x) < std::abs(y);
      });
    }

    if (radius < 0) {
      return -distance;
    } else {
      return distance;
    }
  }

  std::array<hrleCoordType, 6> getBounds() const override {
    std::array<hrleCoordType, 6> bounds = {};
    for (unsigned i = 0; i < D; ++i) {
      bounds[2 * i] = -radius;
      bounds[2 * i + 1] = radius;
    }
    return bounds;
  }
};
#endif