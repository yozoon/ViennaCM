#pragma once

#include <string>
#include <vector>

#include <cmGraphBuilder.hpp>

template <typename NumericType, typename GraphNumericType = NumericType>
class GraphBuilder
    : public cmGraphBuilder<GraphBuilder<NumericType, GraphNumericType>,
                            NumericType, GraphNumericType> {
public:
  const bool connectNeighbors() const override { return true; };

  static constexpr char edgeLengthLabel[] = "length";
  static constexpr char edgeOutboundAngleLabel[] = "outboundAngle";
  static constexpr char edgeInboundAngleLabel[] = "inboundAngle";
  static constexpr char edgeSourceConnectionLabel[] = "sourceConnection";

  void surfaceCollision(const unsigned int originID, const unsigned int destID,
                        const rayTriple<NumericType> &origin,
                        const rayTriple<NumericType> &dest,
                        const rayTriple<NumericType> &originNormal,
                        const rayTriple<NumericType> &destNormal,
                        const int materialId, const RTCRay &ray,
                        cmGraphData<GraphNumericType> &localGraphData,
                        const rayTracingData<NumericType> *globalData,
                        rayRNG &Rng) override final {
    localGraphData.addEdge(originID, destID);

    // Edge length
    // TODO: This is only correct for the first intersection. Subsequent
    // intersections (based on neighbors), would have a different length.
    localGraphData.pushBackEdgeData(0, ray.tfar);

    rayTriple<NumericType> rayDir{ray.dir_x, ray.dir_y, ray.dir_z};
    rayTriple<NumericType> invRayDir{-ray.dir_x, -ray.dir_y, -ray.dir_z};

    // Outbound angle
    GraphNumericType outboundDot = std::min(
        std::max(rayInternal::DotProduct(originNormal, rayDir), -1.), 1.);
    localGraphData.pushBackEdgeData(1, std::acos(outboundDot));

    // Inbound angle
    GraphNumericType inboundDot = std::min(
        std::max(rayInternal::DotProduct(destNormal, invRayDir), -1.), 1.);
    localGraphData.pushBackEdgeData(2, std::acos(inboundDot));

    // Source connection
    localGraphData.pushBackEdgeData(3, 0.);
  }

  void sourceCollision(const unsigned int originID, const unsigned int sourceID,
                       const rayTriple<NumericType> &origin,
                       const rayTriple<NumericType> &originNormal,
                       const rayTriple<NumericType> &rayDir,
                       const rayTriple<NumericType> &sourceCenter,
                       const int sourceDir, const int posNeg,
                       cmGraphData<GraphNumericType> &localGraphData,
                       const rayTracingData<NumericType> *globalData,
                       rayRNG &Rng) override final {
    localGraphData.addEdge(originID, sourceID);

    // Edge length
    localGraphData.pushBackEdgeData(
        0, std::abs(origin[sourceDir] - sourceCenter[sourceDir]));

    // Outbound angle
    GraphNumericType outboundDot = std::min(
        std::max(rayInternal::DotProduct(originNormal, rayDir), -1.), 1.);
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

  void connectNeighbor(const unsigned int fromID, const unsigned int neighborID,
                       const NumericType distance,
                       const rayTriple<NumericType> &geomNormal,
                       const rayTriple<NumericType> &neighborNormal,
                       const int materialId,
                       cmGraphData<GraphNumericType> &localGraphData,
                       const rayTracingData<NumericType> *globalData,
                       rayRNG &Rng) override {
    localGraphData.addEdge(fromID, neighborID);

    // Edge length
    // TODO: This is only correct for the first intersection. Subsequent
    // intersections (based on neighbors), would have a different length.
    localGraphData.pushBackEdgeData(0, distance);

    rayTriple<NumericType> invNeighborNormal{
        -neighborNormal[0], -neighborNormal[1], -neighborNormal[2]};

    // Outbound angle
    GraphNumericType outboundDot = std::min(
        std::max(rayInternal::DotProduct(geomNormal, neighborNormal), -1.), 1.);
    localGraphData.pushBackEdgeData(1, std::acos(outboundDot));

    // Inbound angle
    GraphNumericType inboundDot = std::min(
        std::max(rayInternal::DotProduct(geomNormal, invNeighborNormal), -1.),
        1.);
    localGraphData.pushBackEdgeData(2, std::acos(inboundDot));

    // Source connection
    localGraphData.pushBackEdgeData(3, 0.);
  };

  int getRequiredNodeDataSize() const override final { return 1; }

  std::vector<std::string> getNodeDataLabels() const override final {
    return std::vector<std::string>{"source"};
  }

  int getRequiredEdgeDataSize() const override final { return 4; }

  std::vector<std::string> getEdgeDataLabels() const override final {
    return std::vector<std::string>{edgeLengthLabel, edgeOutboundAngleLabel,
                                    edgeInboundAngleLabel,
                                    edgeSourceConnectionLabel};
  }
};
