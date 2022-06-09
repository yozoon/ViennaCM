#pragma once

#include <embree3/rtcore.h>

#include <lsSmartPointer.hpp>

#include <rayBoundCondition.hpp>
#include <rayBoundary.hpp>
#include <rayGeometry.hpp>
#include <rayMessage.hpp>
#include <rayTraceDirection.hpp>
#include <rayTracingData.hpp>

#include "cmRaySampler.hpp"
#include "cmRaySourceGeometry.hpp"
#include "cmRayTraceGraphKernel.hpp"

template <class NumericType, int D> class cmRayTraceGraph {
private:
  RTCDevice mDevice;
  cmRaySampler<NumericType, D> &mSampler;
  rayGeometry<NumericType, D> mGeometry;
  NumericType mDiskRadius = 0;
  NumericType mGridDelta = 0;
  rayTraceBoundary mBoundaryConds[D] = {};
  rayTraceDirection mSourceDirection = rayTraceDirection::POS_Z;
  rayTraceInfo mRTInfo;
  bool mUseRandomSeeds = false;
  size_t mRunNumber = 0;

public:
  cmRayTraceGraph(cmRaySampler<NumericType, D> &pSampler)
      : mDevice(rtcNewDevice("hugepages=1")), mSampler(pSampler) {}

  ~cmRayTraceGraph() {
    mGeometry.releaseGeometry();
    rtcReleaseDevice(mDevice);
  }

  /// Run the ray tracer
  std::tuple<
      lsSmartPointer<std::vector<std::array<NumericType, 3>>>,
      lsSmartPointer<std::vector<long>>, lsSmartPointer<std::vector<float>>,
      lsSmartPointer<std::vector<float>>, lsSmartPointer<std::vector<float>>,
      lsSmartPointer<std::vector<float>>>
  apply() {
    if (checkSettings()) {
      return {};
    }

    initMemoryFlags();
    auto boundingBox = mGeometry.getBoundingBox();
    rayInternal::adjustBoundingBox<NumericType, D>(
        boundingBox, mSourceDirection, mDiskRadius);
    auto traceSettings = rayInternal::getTraceSettings(mSourceDirection);

    auto boundary = rayBoundary<NumericType, D>(mDevice, boundingBox,
                                                mBoundaryConds, traceSettings);

    auto source = cmRaySourceGeometry<NumericType, D>(
        mDevice, boundingBox, mBoundaryConds, traceSettings);

    auto tracer =
        cmRayTraceGraphKernel(mDevice, mGeometry, source, boundary, mSampler,
                              mUseRandomSeeds, ++mRunNumber);

    // tracer.setRayTraceInfo(&mRTInfo);
    const auto &[edges, edgeLenghts, outboundAngles, inboundAngles,
                 sourceConnections] = tracer.apply();

    auto nodes = lsSmartPointer<std::vector<std::array<NumericType, 3>>>::New();
    nodes->reserve(mGeometry.getNumPoints());
    for (size_t i = 0; i < mGeometry.getNumPoints(); ++i) {
      auto p = mGeometry.getPoint(i);
      nodes->push_back(p);
    }

    std::array<NumericType, 3> sourceNode{0};
    for (size_t i = 0; i < 4; ++i) {
      auto pt = source.getPoint(i);
      sourceNode[0] += pt[0] / 4;
      sourceNode[1] += pt[1] / 4;
      sourceNode[2] += pt[2] / 4;
    }
    nodes->push_back(sourceNode);

    source.releaseGeometry();
    boundary.releaseGeometry();

    return {nodes,          edges,         edgeLenghts,
            outboundAngles, inboundAngles, sourceConnections};
  }

  /// Set the ray tracing geometry
  /// It is possible to set a 2D geometry with 3D points.
  /// In this case the last dimension is ignored.
  template <std::size_t Dim>
  void setGeometry(std::vector<std::array<NumericType, Dim>> &points,
                   std::vector<std::array<NumericType, Dim>> &normals,
                   const NumericType gridDelta) {
    static_assert((D != 3 || Dim != 2) &&
                  "Setting 2D geometry in 3D trace object");

    mGridDelta = gridDelta;
    mDiskRadius = rayInternal::DiskFactor * mGridDelta;
    mGeometry.initGeometry(mDevice, points, normals, mDiskRadius);
  }

  /// Set material ID's for each geometry point.
  /// If not set, all material ID's are default 0.
  template <typename T> void setMaterialIds(std::vector<T> &pMaterialIds) {
    // pMaterialIds.push_back(-1);
    mGeometry.setMaterialIds(pMaterialIds);
  }

  /// Set the boundary conditions.
  /// There has to be a boundary condition defined for each space dimension,
  /// however the boundary condition in direction of the tracing direction is
  /// ignored.
  void setBoundaryConditions(rayTraceBoundary pBoundaryConds[D]) {
    for (size_t i = 0; i < D; ++i) {
      mBoundaryConds[i] = pBoundaryConds[i];
    }
  }

  /// Set the source direction, where the rays should be traced from.
  void setSourceDirection(const rayTraceDirection pDirection) {
    mSourceDirection = pDirection;
  }

  rayTraceInfo getRayTraceInfo() { return mRTInfo; }

private:
  int checkSettings() {
    if (mGeometry.checkGeometryEmpty()) {
      rayMessage::getInstance().addError(
          "No geometry was passed to rayTrace. Aborting.");
      return 1;
    }
    if ((D == 2 && mSourceDirection == rayTraceDirection::POS_Z) ||
        (D == 2 && mSourceDirection == rayTraceDirection::NEG_Z)) {
      rayMessage::getInstance().addError(
          "Invalid source direction in 2D geometry. Aborting.");
      return 1;
    }
    if (mDiskRadius > mGridDelta) {
      rayMessage::getInstance()
          .addWarning("Disk radius should be smaller than grid delta. Hit "
                      "count normalization not correct.")
          .print();
      return 1;
    }
    return 0;
  }

  void initMemoryFlags() {
#ifdef ARCH_X86
    // for best performance set FTZ and DAZ flags in MXCSR control and status
    // register
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif
  }
};