#pragma once

#include <embree3/rtcore.h>

#include <rayGeometry.hpp>
#include <rayRNG.hpp>
#include <rayTracingData.hpp>
#include <rayUtil.hpp>

#include <cmGraphData.hpp>

template <typename NumericType, typename GraphNumericType>
class cmAbstractGraphBuilder {
public:
  /// These function must NOT be overwritten by user
  virtual ~cmAbstractGraphBuilder() = default;
  virtual std::unique_ptr<cmAbstractGraphBuilder> clone() const = 0;

  /// Surface collision. This function gets called whenever an intersection of
  /// the ray and a disc is found.
  /// rayDir: direction of the particle at collision;
  /// geomNormal: surface normal of the hit disc;
  /// primId: ID fo the hit disc;
  /// localData: user-defined data;
  /// globalData: constant user-defined data;
  /// Rng: thread-safe randon number generator (standard library conform);
  virtual void surfaceCollision(
      const unsigned int originID, const unsigned int destID,
      const rayTriple<NumericType> &origin, const rayTriple<NumericType> &dest,
      const rayTriple<NumericType> &originNormal,
      const rayTriple<NumericType> &destNormal, const int materialId,
      const RTCRay &ray, cmGraphData<GraphNumericType> &localGraphData,
      const rayTracingData<NumericType> *globalData, rayRNG &Rng) = 0;

  // Source collision. This function gets called whenever a ray hits the source
  // plane.
  virtual void sourceCollision(const unsigned int originID,
                               const unsigned int sourceID,
                               const rayTriple<NumericType> &origin,
                               const rayTriple<NumericType> &originNormal,
                               const rayTriple<NumericType> &rayDir,
                               const rayTriple<NumericType> &sourceCenter,
                               const int sourceDir, const int posNeg,
                               cmGraphData<GraphNumericType> &localGraphData,
                               const rayTracingData<NumericType> *globalData,
                               rayRNG &Rng) = 0;

  /// Set the number of required data vectors for this particle to
  /// collect data.
  virtual int getRequiredNodeDataSize() const = 0;
  virtual int getRequiredEdgeDataSize() const = 0;

  virtual std::vector<std::string> getNodeDataLabels() const = 0;
  virtual std::vector<std::string> getEdgeDataLabels() const = 0;

  virtual const bool connectNeighbors() const = 0;

  virtual void connectNeighbor(
      const unsigned int originID, const unsigned int neighborID,
      const NumericType distance, const rayTriple<NumericType> &geomNormal,
      const rayTriple<NumericType> &neighborNormal, const int materialId,
      cmGraphData<GraphNumericType> &localGraphData,
      const rayTracingData<NumericType> *globalData, rayRNG &Rng) = 0;
};

/// This CRTP class implements clone() for the derived graph builder class.
/// A user has to interface this class.
template <typename Derived, typename NumericType,
          typename GraphNumericType = NumericType>
class cmGraphBuilder
    : public cmAbstractGraphBuilder<NumericType, GraphNumericType> {
public:
  std::unique_ptr<cmAbstractGraphBuilder<NumericType, GraphNumericType>>
  clone() const override final {
    return std::make_unique<Derived>(static_cast<Derived const &>(*this));
  }

  virtual void surfaceCollision(
      const unsigned int originID, const unsigned int destID,
      const rayTriple<NumericType> &origin, const rayTriple<NumericType> &dest,
      const rayTriple<NumericType> &originNormal,
      const rayTriple<NumericType> &destNormal, const int materialId,
      const RTCRay &ray, cmGraphData<GraphNumericType> &localGraphData,
      const rayTracingData<NumericType> *globalData, rayRNG &Rng) override {}

  virtual void sourceCollision(const unsigned int originID,
                               const unsigned int sourceID,
                               const rayTriple<NumericType> &origin,
                               const rayTriple<NumericType> &originNormal,
                               const rayTriple<NumericType> &rayDir,
                               const rayTriple<NumericType> &sourceCenter,
                               const int sourceDir, const int posNeg,
                               cmGraphData<GraphNumericType> &localGraphData,
                               const rayTracingData<NumericType> *globalData,
                               rayRNG &Rng) override {}

  virtual void connectNeighbor(
      const unsigned int originID, const unsigned int neighborID,
      const NumericType distance, const rayTriple<NumericType> &geomNormal,
      const rayTriple<NumericType> &neighborNormal, const int materialId,
      cmGraphData<GraphNumericType> &localGraphData,
      const rayTracingData<NumericType> *globalData, rayRNG &Rng) override{};

  virtual int getRequiredNodeDataSize() const override { return 0; }
  virtual int getRequiredEdgeDataSize() const override { return 0; }

  virtual std::vector<std::string> getNodeDataLabels() const override {
    return std::vector<std::string>(getRequiredNodeDataSize(), "nodeData");
  }

  virtual std::vector<std::string> getEdgeDataLabels() const override {
    return std::vector<std::string>(getRequiredEdgeDataSize(), "edgeData");
  }

protected:
  // Ensure that cmGraphBuilder class needs to be inherited
  cmGraphBuilder() = default;
  cmGraphBuilder(const cmGraphBuilder &) = default;
  cmGraphBuilder(cmGraphBuilder &&) = default;
};
