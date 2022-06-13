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
  virtual void surfaceCollision(const unsigned int fromID, const RTCRay &ray,
                                const rayTriple<NumericType> &geomNormal,
                                const unsigned int toID, const int materialId,
                                cmGraphData<GraphNumericType> &localGraphData,
                                const rayTracingData<NumericType> *globalData,
                                rayRNG &Rng) = 0;

  // Source collision. This function gets called whenever a ray hits the source
  // plane.
  virtual void sourceCollision(
      const unsigned int fromID, const rayTriple<NumericType> &rayOrigin,
      const rayTriple<NumericType> &rayDir,
      const rayTriple<NumericType> &geomNormal, const unsigned int sourceID,
      const rayTriple<NumericType> &sourceCenter, const unsigned int sourceDir,
      const unsigned int posNeg, cmGraphData<GraphNumericType> &localGraphData,
      const rayTracingData<NumericType> *globalData, rayRNG &Rng) = 0;

  /// Set the number of required data vectors for this particle to
  /// collect data.
  virtual int getRequiredNodeDataSize() const = 0;
  virtual int getRequiredEdgeDataSize() const = 0;

  virtual std::vector<std::string> getNodeDataLabels() const = 0;
  virtual std::vector<std::string> getEdgeDataLabels() const = 0;
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

  virtual void surfaceCollision(const unsigned int fromID, const RTCRay &ray,
                                const rayTriple<NumericType> &geomNormal,
                                const unsigned int toID, const int materialId,
                                cmGraphData<GraphNumericType> &localGraphData,
                                const rayTracingData<NumericType> *globalData,
                                rayRNG &Rng) override {}

  virtual void sourceCollision(
      const unsigned int fromID, const rayTriple<NumericType> &rayOrigin,
      const rayTriple<NumericType> &rayDir,
      const rayTriple<NumericType> &geomNormal, const unsigned int sourceID,
      const rayTriple<NumericType> &sourceCenter, const unsigned int sourceDir,
      const unsigned int posNeg, cmGraphData<GraphNumericType> &localGraphData,
      const rayTracingData<NumericType> *globalData, rayRNG &Rng) override {}

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

template <typename NumericType, typename GraphNumericType = NumericType>
class cmGeometricGraphBuilder
    : public cmGraphBuilder<
          cmGeometricGraphBuilder<NumericType, GraphNumericType>, NumericType,
          GraphNumericType> {
public:
  void surfaceCollision(const unsigned int fromID, const RTCRay &ray,
                        const rayTriple<NumericType> &geomNormal,
                        const unsigned int toID, const int materialId,
                        cmGraphData<GraphNumericType> &localGraphData,
                        const rayTracingData<NumericType> *globalData,
                        rayRNG &Rng) override final {
    localGraphData.addEdge(fromID, toID);

    // Edge length
    // TODO: This is only correct for the first intersection. Subsequent
    // intersections (based on neighbors), would have a different length.
    // myLocalEdgeLengths[dataIndex] = ray.tfar;
    localGraphData.pushBackEdgeData(0, ray.tfar);

    rayTriple<NumericType> rayDir{ray.dir_x, ray.dir_y, ray.dir_z};
    rayTriple<NumericType> invRayDir{-ray.dir_x, -ray.dir_y, -ray.dir_z};

    // Outbound angle
    GraphNumericType outboundDot = std::min(
        std::max(rayInternal::DotProduct(geomNormal, rayDir), -1.), 1.);
    // myLocalOutboundAngles[dataIndex] = std::acos(outboundDot);
    localGraphData.pushBackEdgeData(1, std::acos(outboundDot));

    // Inbound angle
    GraphNumericType inboundDot = std::min(
        std::max(rayInternal::DotProduct(geomNormal, invRayDir), -1.), 1.);
    // myLocalInboundAngles[dataIndex] = std::acos(inboundDot);
    localGraphData.pushBackEdgeData(2, std::acos(inboundDot));

    // Source connection
    // myLocalSourceConnection[dataIndex] = 0;
    localGraphData.pushBackEdgeData(3, 0.);
  }

  void sourceCollision(const unsigned int fromID,
                       const rayTriple<NumericType> &rayOrigin,
                       const rayTriple<NumericType> &rayDir,
                       const rayTriple<NumericType> &geomNormal,
                       const unsigned int sourceID,
                       const rayTriple<NumericType> &sourceCenter,
                       const unsigned int sourceDir, const unsigned int posNeg,
                       cmGraphData<GraphNumericType> &localGraphData,
                       const rayTracingData<NumericType> *globalData,
                       rayRNG &Rng) override final {
    localGraphData.addEdge(fromID, sourceID);

    // Edge length
    localGraphData.pushBackEdgeData(
        0, std::abs(rayOrigin[sourceDir] - sourceCenter[sourceDir]));

    // Outbound angle
    GraphNumericType outboundDot = std::min(
        std::max(rayInternal::DotProduct(geomNormal, rayDir), -1.), 1.);
    localGraphData.pushBackEdgeData(1, std::acos(outboundDot));

    // Inbound angle
    rayTriple<NumericType> invRayDir{-rayDir[0], -rayDir[1], -rayDir[2]};
    rayTriple<NumericType> sourceNormal{0.};
    sourceNormal[sourceDir] = posNeg;
    GraphNumericType inboundDot = std::min(
        std::max(rayInternal::DotProduct(sourceNormal, invRayDir), -1.), 1.);
    localGraphData.pushBackEdgeData(2, std::acos(inboundDot));

    // Source connection
    localGraphData.pushBackEdgeData(3, 1.);
  }

  int getRequiredNodeDataSize() const override final { return 1; }

  std::vector<std::string> getNodeDataLabels() const override final {
    return std::vector<std::string>{"source"};
  }

  int getRequiredEdgeDataSize() const override final { return 4; }

  std::vector<std::string> getEdgeDataLabels() const override final {
    return std::vector<std::string>{"length", "outboundAngle", "inboundAngle",
                                    "sourceConnection"};
  }
};
