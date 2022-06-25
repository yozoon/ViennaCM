#pragma once

#include <array>
#include <vector>

#include <rayGeometry.hpp>
#include <rayRNG.hpp>
#include <rayUtil.hpp>

template <class NumericType, int D> class cmRaySampler {
public:
  virtual unsigned getNumberOfRaysPerPoint() const = 0;

  // Ray direction initialization. This function is called multiple times
  // (determined by numberOfRaysPerPoint) for every point on the geometry. Given
  // the normal vector and the index of this particular sampling request, the
  // function should return the direction in which the ray should be cast from
  // this particular surface point.
  virtual rayTriple<NumericType>
  getDirection(rayRNG &RNG, const rayTriple<NumericType> &surfaceNormal,
               const size_t dirIdx) const = 0;
};

template <class NumericType, int D>
class cmUniformRaySampler : public cmRaySampler<NumericType, D> {
  const unsigned mNumOfRaysPerPoint;

  std::vector<std::array<NumericType, 3>> rayDirections;

public:
  cmUniformRaySampler(unsigned pNumOfRaysPerPoint)
      : mNumOfRaysPerPoint(pNumOfRaysPerPoint) {
    initializeDirections();
  }

  // Calculates the direction of the outgoing ray relative to the normal
  // vector at the origin point.
  rayTriple<NumericType>
  getDirection(rayRNG &RNG, const rayTriple<NumericType> &surfaceNormal,
               const size_t dirIdx) const override {
    rayTriple<NumericType> direction{0., 0., 0.};

    const auto rayDirection = rayDirections[dirIdx];

    rayTriple<NumericType> t{0.};
    rayTriple<NumericType> s{0.};

    if constexpr (D == 2) {
      // In 2D we can simply calculate the normal vector by swapping the
      // components and switching one sign
      t[0] = surfaceNormal[1];
      t[1] = -surfaceNormal[0];
    } else {
      if (std::abs(surfaceNormal[2]) != 1.) {
        t = rayInternal::CrossProduct(surfaceNormal,
                                      rayTriple<NumericType>{0., 0., 1.});
        s = rayInternal::CrossProduct(surfaceNormal, t);
      } else {
        t[1] = 1.;
        s[0] = 1.;
      }
    }
    rayTriple<rayTriple<NumericType>> basis = {surfaceNormal, t, s};
    for (size_t j = 0; j < D; ++j)
      for (size_t i = 0; i < D; ++i)
        direction[j] += rayDirection[j] + basis[i][j] * rayDirection[i];

    rayInternal::Normalize(direction);
    return direction;
  }

  unsigned getNumberOfRaysPerPoint() const override {
    return mNumOfRaysPerPoint;
  }

private:
  void initializeDirections() {
    // Calculate the ray directions in advance. This creates
    // mNumOfRaysPerPoint uniformly distributed rays.
    rayDirections.reserve(mNumOfRaysPerPoint);
    if constexpr (D == 2) {
      // Uniformly place points on a circle
      for (unsigned i = 0; i < mNumOfRaysPerPoint; ++i) {
        rayDirections.emplace_back(std::array<NumericType, 3>{
            std::cos(2. * rayInternal::PI * i / mNumOfRaysPerPoint),
            std::sin(2. * rayInternal::PI * i / mNumOfRaysPerPoint), 0.});
      }
    } else {
      // Uniformly place points on sphere using Fibonacci spiral placement.
      static constexpr NumericType sqrt5 = std::sqrt(5.);
      // TODO: Decide which offset value works best
      static constexpr NumericType offset = 0.;
      for (unsigned i = 0; i < mNumOfRaysPerPoint; ++i) {
        NumericType phi =
            std::acos(1. - 2. * (i + offset) / mNumOfRaysPerPoint);
        NumericType theta = M_PI * (1. + sqrt5) * (i + offset);
        rayDirections.emplace_back(std::array<NumericType, 3>{
            std::cos(theta) * std::sin(phi),
            std::sin(theta) * std::sin(phi),
            std::cos(phi),
        });
      }
    }
  }
};
template <class NumericType, int D>
class cmCosineDistributionRaySampler : public cmRaySampler<NumericType, D> {
  const unsigned mNumOfRaysPerPoint;

public:
  cmCosineDistributionRaySampler(unsigned pNumOfRaysPerPoint)
      : mNumOfRaysPerPoint(pNumOfRaysPerPoint) {}

  rayTriple<NumericType>
  getDirection(rayRNG &RNG, const rayTriple<NumericType> &surfaceNormal,
               const size_t dirIdx) const override {
    auto randomDirection =
        rayInternal::PickRandomPointOnUnitSphere<NumericType>(RNG);
    randomDirection[0] += surfaceNormal[0];
    randomDirection[1] += surfaceNormal[1];
    if constexpr (D == 3)
      randomDirection[2] += surfaceNormal[2];
    else
      randomDirection[2] = 0;

    rayInternal::Normalize(randomDirection);
    assert(rayInternal::IsNormalized(randomDirection) &&
           "cmCosineDistributionRaySampler: New direction is not normalized");
    return randomDirection;
  }

  unsigned getNumberOfRaysPerPoint() const override {
    return mNumOfRaysPerPoint;
  }
};
