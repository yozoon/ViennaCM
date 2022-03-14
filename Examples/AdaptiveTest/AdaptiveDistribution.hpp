#ifndef ADAPTIVE_DISTRIBUTION_HPP
#define ADAPTIVE_DISTRIBUTION_HPP

#include <mutex>
#include <shared_mutex>

#include <lsDomain.hpp>
#include <lsGeometricAdvectDistributions.hpp>
#include <lsSmartPointer.hpp>

inline static std::shared_mutex lock;

template <class T, int D, class MapType>
class AdaptiveDistribution : public lsGeometricAdvectDistribution<T, D> {
  using VectorType = std::array<hrleCoordType, 3>;

  const lsSmartPointer<lsDomain<T, D>> levelSet;
  MapType &map;
  std::vector<std::array<hrleCoordType, 3>> &normals;
  std::vector<hrleCoordType> &curvatures;
  T gridDelta;

  static constexpr T dotProduct(const VectorType &a, const VectorType &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  static constexpr std::array<VectorType, 2> generatePoints() {
    T r = 6.;
    std::array<VectorType, 2> points({});
    points[0][D - 1] = r;

    points[1][0] = 2 * r;
    points[1][D - 1] = r;

    return points;
  }

  static constexpr std::array<VectorType, 2> generateNormals() {
    std::array<VectorType, 2> normals({});
    normals[0][D - 1] = 1;

    normals[1][0] = 1 / std::sqrt(2);
    normals[1][D - 1] = -1 / std::sqrt(2);
    return normals;
  }

  static constexpr std::array<T, D * D>
  generateNormalBasis(const std::array<T, 3> &n) {
    // Row major representation
    // Calculates normal basis using Gram Schmidt procedure with vector n as
    // initial vector. The unit vector of axis D-1 will be used as the second
    // orientation vector in the orthogonalization procedure. If n is
    // (anti)parallel to the axis D-1, the unit vector of axis 0 is used
    // instead. In 3D the third basis vector is calculated by taking the
    // cross-product of the two other base vectors.

    std::array<T, D * D> basis({0.});
    basis[0] = n[0];
    basis[D] = n[1];
    if constexpr (D == 3)
      basis[2 * D] = n[2];

    constexpr int Z = D - 1;
    if (std::abs(n[Z]) != 1.) {
      T sum{0.};
      for (unsigned i = 0; i < D; ++i) {
        T tmp = 1. * (i == Z) - n[Z] * n[i];
        basis[(i * D) + 1] = tmp;
        sum += tmp * tmp;
      }

      if (sum != 1)
        sum = std::sqrt(sum);

      for (unsigned i = 0; i < D; ++i)
        basis[(i * D) + 1] /= sum;

    } else {
      // If n is parallel to the normal vector in axis D-1, simply use unit
      // vector in x-direction as basis vector.
      basis[1] = 1.;
    }

    if constexpr (D == 3) {
      // Cross Product
      basis[2] = basis[3] * basis[7] - basis[6] * basis[4];
      basis[5] = basis[6] * basis[1] - basis[0] * basis[7];
      basis[8] = basis[0] * basis[4] - basis[3] * basis[1];
    }

    return basis;
  }

  static constexpr VectorType matMul(const std::array<T, D * D> &A,
                                     const VectorType &v) {
    VectorType res{0.};
    for (unsigned i = 0; i < D; ++i) {
      if constexpr (D == 2) {
        res[i] = A[i * D] * v[0] + A[i * D + 1] * v[1];
      } else {
        res[i] = A[i * D] * v[0] + A[i * D + 1] * v[1] + A[i * D + 2] * v[2];
      }
    }
    return res;
  }

  static constexpr std::array<T, D * D> inverse(const std::array<T, D * D> &m) {
    if constexpr (D == 2) {
      T det = m[0] * m[3] - m[1] * m[2];
      return {m[3] / det, -m[1] / det, -m[2] / det, m[0] / det};
    } else {
      // Sarrus' rule (row major matrix)
      T det = m[0] * m[4] * m[8] + m[1] * m[5] * m[6] + m[3] * m[7] * m[2] -
              m[2] * m[4] * m[6] - m[1] * m[3] * m[8] - m[0] * m[5] * m[7];

      return {
          (m[4] * m[8] - m[7] * m[5]) / det, (m[2] * m[7] - m[8] * m[1]) / det,
          (m[1] * m[5] - m[4] * m[2]) / det, (m[5] * m[6] - m[8] * m[3]) / det,
          (m[0] * m[8] - m[6] * m[2]) / det, (m[2] * m[3] - m[5] * m[0]) / det,
          (m[3] * m[7] - m[6] * m[4]) / det, (m[1] * m[6] - m[7] * m[0]) / det,
          (m[0] * m[4] - m[3] * m[1]) / det,
      };
    }
  }

public:
  AdaptiveDistribution(const lsSmartPointer<lsDomain<T, D>> passedLevelSet,
                       MapType &passedMap,
                       std::vector<std::array<hrleCoordType, 3>> &passedNormals,
                       std::vector<T> &passedCurvatures, T passedGridDelta)
      : levelSet(passedLevelSet), map(passedMap), normals(passedNormals),
        curvatures(passedCurvatures), gridDelta(passedGridDelta) {}

  T getSignedDistance(const std::array<hrleCoordType, 3> &initial,
                      const std::array<hrleCoordType, 3> &candidate,
                      unsigned long initialPointId) const override {

    const auto &normal = normals[initialPointId];
    const auto &curvature = curvatures[initialPointId];
    const T curvatureThreshold = gridDelta;

    T thickness = std::max(0.5, (initial[D - 1] + 40.) / 10);

    if (candidate[D - 1] < 0.)
      thickness /= 2;
    // T radius2 = radius * radius;

    std::array<T, D> halfAxis{};
    halfAxis[0] = thickness;
    halfAxis[1] = gridDelta;
    if constexpr (D == 3)
      halfAxis[2] = gridDelta;

    std::array<hrleCoordType, 3> v{0.};
    for (unsigned i = 0; i < D; ++i)
      v[i] = candidate[i] - initial[i];

    // if (curvature > curvatureThreshold) {
    //  Convex part
    std::array<T, D * D> inv;
    bool contains = false;
    {
      std::shared_lock<std::shared_mutex> r_lock(lock);
      auto it = map.find(normal);
      contains = it != map.end();
      if (contains)
        inv = it->second;
    }
    if (!contains) {
      auto normalBasis = generateNormalBasis(normal);
      inv = inverse(normalBasis);
      // Do stuff
      std::unique_lock<std::shared_mutex> w_lock(lock);
      map.insert(std::pair{normal, inv});
    }

    auto vprime = matMul(inv, v);

    T distance = std::numeric_limits<T>::lowest();
    // static constexpr auto points = generatePoints();
    // static constexpr auto normals = generateNormals();
    // for (unsigned i = 0; i < 2; ++i) {
    //   distance =
    //       std::max(distance, dotProduct(subtract(v, points[i]),
    //       normals[i]));
    for (unsigned i = 0; i < D; ++i) {
      T vector = std::abs(vprime[i]);
      distance = std::max(vector - std::abs(halfAxis[i]), distance);
    }
    //}
    return distance;
    /*} else {
      // Flat and Concave regions

      // Spherical Distribution (approximation of spherical signed distance)
      T distance = std::numeric_limits<T>::max();
      if (std::abs(radius) <= gridDelta) {
        distance =
            std::max(std::max(std::abs(v[0]), std::abs(v[1])), std::abs(v[2])) -
            std::abs(radius);
      } else {
        for (unsigned i = 0; i < D; ++i) {
          T y = (v[(i + 1) % D]);
          T z = 0;
          if constexpr (D == 3)
            z = (v[(i + 2) % D]);
          T x = radius2 - y * y - z * z;
          if (x < 0.)
            continue;
          T dirRadius = std::abs(v[i]) - std::sqrt(x);
          if (std::abs(dirRadius) < std::abs(distance))
            distance = dirRadius;
        }
      }
      return distance;

  }*/
  }

  std::array<hrleCoordType, 6> getBounds() const override {
    static constexpr hrleCoordType b = 5.;
    std::array<hrleCoordType, 6> bounds{};
    for (unsigned i = 0; i < D - 1; ++i) {
      bounds[2 * i] = -b;
      bounds[2 * i + 1] = b;
    }
    bounds[2 * (D - 1)] = -b;
    bounds[2 * D - 1] = b;
    return bounds;
  }
};
#endif