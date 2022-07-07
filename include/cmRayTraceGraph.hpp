#pragma once

#include <algorithm>

#include <embree3/rtcore.h>

#include <lsSmartPointer.hpp>

#include <rayBoundCondition.hpp>
#include <rayBoundary.hpp>
#include <rayGeometry.hpp>
#include <rayMessage.hpp>
#include <rayTraceDirection.hpp>
#include <rayTracingData.hpp>

#include <cmGraphBuilder.hpp>
#include <cmGraphData.hpp>
#include <cmRaySampler.hpp>
#include <cmRaySourceGeometry.hpp>
#include <cmRayTraceGraphKernel.hpp>

template <typename NumericType, int D, typename GraphNumericType = NumericType>
class cmRayTraceGraph {
private:
  RTCDevice mDevice;
  rayGeometry<NumericType, D> mGeometry;
  std::unique_ptr<cmAbstractRaySampler<NumericType, D>> mSampler = nullptr;
  std::unique_ptr<cmAbstractGraphBuilder<NumericType, GraphNumericType>>
      mBuilder = nullptr;
  NumericType mDiskRadius = 0;
  NumericType mGridDelta = 0;
  rayTraceBoundary mBoundaryConds[D] = {};
  rayTraceDirection mSourceDirection = rayTraceDirection::POS_Z;
  bool mUseRandomSeeds = false;
  size_t mRunNumber = 0;
  cmGraphData<GraphNumericType> mLocalGraphData;
  rayTracingData<NumericType> *mGlobalData = nullptr;
  rayTraceInfo mRTInfo;

public:
  cmRayTraceGraph() : mDevice(rtcNewDevice("hugepages=1")) {}

  ~cmRayTraceGraph() {
    mGeometry.releaseGeometry();
    rtcReleaseDevice(mDevice);
  }

  void apply() {
    if (checkSettings())
      return;

    initMemoryFlags();
    auto boundingBox = mGeometry.getBoundingBox();
    rayInternal::adjustBoundingBox<NumericType, D>(
        boundingBox, mSourceDirection, mDiskRadius);
    auto traceSettings = rayInternal::getTraceSettings(mSourceDirection);

    auto boundary = rayBoundary<NumericType, D>(mDevice, boundingBox,
                                                mBoundaryConds, traceSettings);

    auto source = cmRaySourceGeometry<NumericType, D>(
        mDevice, boundingBox, mBoundaryConds, traceSettings);

    mLocalGraphData.clear();

    auto numberOfNodeData = mBuilder->getRequiredNodeDataSize();
    if (numberOfNodeData) {
      mLocalGraphData.setNumberOfNodeData(numberOfNodeData);
      auto numPoints = mGeometry.getNumPoints() + 1;
      auto nodeDataLabes = mBuilder->getNodeDataLabels();
      for (int i = 0; i < numberOfNodeData; ++i) {
        mLocalGraphData.setNodeData(i, numPoints, 0., nodeDataLabes[i]);
      }
    }

    auto numberOfEdgeData = mBuilder->getRequiredEdgeDataSize();
    if (numberOfEdgeData) {
      mLocalGraphData.setNumberOfEdgeData(numberOfEdgeData);
      auto edgeDataLabels = mBuilder->getEdgeDataLabels();
      for (int i = 0; i < numberOfEdgeData; ++i) {
        mLocalGraphData.setEdgeData(i, {}, edgeDataLabels[i]);
      }
    }

    auto tracer =
        cmRayTraceGraphKernel(mDevice, mGeometry, source, boundary, mSampler,
                              mBuilder, mUseRandomSeeds, ++mRunNumber);

    tracer.setTracingData(&mLocalGraphData, mGlobalData);
    tracer.setRayTraceInfo(&mRTInfo);

    tracer.apply();

    // Add point coordinates to the graph data
    auto &nodes = mLocalGraphData.getNodes();
    nodes.clear();
    nodes.reserve(mGeometry.getNumPoints() + 1);
    for (size_t i = 0; i < mGeometry.getNumPoints(); ++i) {
      const auto &point = mGeometry.getPoint(i);
      nodes.push_back(std::array<GraphNumericType, 3>{
          static_cast<GraphNumericType>(point[0]),
          static_cast<GraphNumericType>(point[1]),
          static_cast<GraphNumericType>(point[2]),
      });
    }

    std::array<GraphNumericType, 3> sourceNode{0};
    for (size_t i = 0; i < 4; ++i) {
      auto pt = source.getPoint(i);
      sourceNode[0] += pt[0] / 4;
      sourceNode[1] += pt[1] / 4;
      sourceNode[2] += pt[2] / 4;
    }

    nodes.push_back(sourceNode);

    source.releaseGeometry();
    boundary.releaseGeometry();
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
    mDiskRadius = rayInternal::DiskFactor<D> * mGridDelta;
    mGeometry.initGeometry(mDevice, points, normals, mDiskRadius);
  }

  /// Set the graph builder type used for ray tracing
  /// The graph builder is a user defined object that has to interface the
  /// cmGraphBuilder class.
  template <typename BuilderType>
  void setGraphBuilderType(std::unique_ptr<BuilderType> &p) {
    static_assert(
        std::is_base_of<cmAbstractGraphBuilder<NumericType, GraphNumericType>,
                        BuilderType>::value &&
        "Graph Builder object does not interface correct class");
    mBuilder = p->clone();
  }

  /// Set the graph sampler type used for ray tracing
  /// The graph sampler is a user defined object that has to interface the
  /// cmGraphSampler class.
  template <typename SamplerType>
  void setGraphSamplerType(std::unique_ptr<SamplerType> &p) {
    static_assert(std::is_base_of<cmAbstractRaySampler<NumericType, D>,
                                  SamplerType>::value &&
                  "Ray Sampler object does not interface correct class");
    mSampler = p->clone();
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

  cmGraphData<GraphNumericType> &getLocalGraphData() { return mLocalGraphData; }

  rayTracingData<NumericType> *getGlobalData() { return mGlobalData; }

  void setGlobalData(rayTracingData<NumericType> &data) { mGlobalData = &data; }

  rayTraceInfo getRayTraceInfo() { return mRTInfo; }

private:
  int checkSettings() {
    if (mBuilder == nullptr) {

      return 1;
    }
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