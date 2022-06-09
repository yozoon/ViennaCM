#pragma once

#include <tuple>

#include <lsMesh.hpp>
#include <lsMessage.hpp>
#include <lsSmartPointer.hpp>

#include <rayBoundCondition.hpp>

#include "cmGraph.hpp"
#include "cmRaySampler.hpp"
#include "cmRayTraceGraph.hpp"

template <class NumericType, int D,
          class raySampler = cmRandomRaySampler<NumericType, D>>
class cmGeometryToGraph {
  static_assert(
      std::is_base_of_v<cmRaySampler<NumericType, D>, raySampler> &&
      "The provided raySampler has to be derived from the cmRaySampler class.");

  lsSmartPointer<lsMesh<>> mMesh = nullptr;
  lsSmartPointer<cmGraph> mGraph = nullptr;
  const NumericType gridDelta;
  const unsigned numberOfRaysPerPoint;

public:
  cmGeometryToGraph() {}

  cmGeometryToGraph(lsSmartPointer<lsMesh<>> pMesh,
                    lsSmartPointer<cmGraph> pGraph, NumericType pGridDelta,
                    unsigned pNumberOfRaysPerPoint = 11)
      : mMesh(pMesh), mGraph(pGraph), gridDelta(pGridDelta),
        numberOfRaysPerPoint(pNumberOfRaysPerPoint) {}

  void setMesh(lsSmartPointer<lsMesh<>> pMesh) { mMesh = pMesh; }

  void setGraph(lsSmartPointer<cmGraph> pGraph) { mGraph = pGraph; }

  void apply() {
    if (mMesh == nullptr) {
      lsMessage::getInstance().addError("The passed mesh is null. Aborting.");
      return;
    }
    if (mGraph == nullptr) {
      lsMessage::getInstance().addError("The passed graph is null. Aborting.");
      return;
    }

    auto &points = mMesh->getNodes();
    auto &normals = *mMesh->getCellData().getVectorData("Normals");
    auto &materialIds = *mMesh->getCellData().getScalarData("MaterialIds");

    // ray tracing setup
    rayTraceBoundary rtBC[D];
    for (unsigned i = 0; i < D; ++i)
      rtBC[i] = rayTraceBoundary::REFLECTIVE;

    raySampler sampler(numberOfRaysPerPoint);

    cmRayTraceGraph<NumericType, D> graphTracer(sampler);

    graphTracer.setSourceDirection(D == 2 ? rayTraceDirection::POS_Y
                                          : rayTraceDirection::POS_Z);
    graphTracer.setBoundaryConditions(rtBC);
    graphTracer.setGeometry(points, normals, gridDelta);
    graphTracer.setMaterialIds(materialIds);

    const auto &[nodes, edges, edgeLengths, outboundAngles, inboundAngles,
                 sourceConnections] = graphTracer.apply();

    mGraph->x = std::vector<float>(nodes->size(), 0.f);
    mGraph->x.back() = 1.f;

    mGraph->edgeIndex.swap(*edges);
    mGraph->edgeIndex.reserve(mGraph->edgeIndex.size() * 2);
    std::copy(mGraph->edgeIndex.rbegin(), mGraph->edgeIndex.rend(),
              std::back_inserter(mGraph->edgeIndex));

    mGraph->edgeLengths.swap(*edgeLengths);
    mGraph->edgeLengths.reserve(mGraph->edgeLengths.size() * 2);
    std::copy(mGraph->edgeLengths.rbegin(), mGraph->edgeLengths.rend(),
              std::back_inserter(mGraph->edgeLengths));

    mGraph->edgeAngles.swap(*outboundAngles);

    mGraph->edgeAngles.reserve(mGraph->edgeAngles.size() * 2);

    std::copy(inboundAngles->rbegin(), inboundAngles->rend(),
              std::back_inserter(mGraph->edgeAngles));

    mGraph->edgeSourceConnection.swap(*sourceConnections);
    mGraph->edgeSourceConnection.reserve(mGraph->edgeAngles.size() * 2);

    std::copy(mGraph->edgeSourceConnection.rbegin(),
              mGraph->edgeSourceConnection.rend(),
              std::back_inserter(mGraph->edgeSourceConnection));
  }
};