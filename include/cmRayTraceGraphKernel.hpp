#pragma once

#include <embree3/rtcore.h>

#include <lsSmartPointer.hpp>

#include <rayBoundary.hpp>
#include <rayGeometry.hpp>
#include <rayParticle.hpp>
#include <rayRNG.hpp>
#include <raySource.hpp>
#include <rayTracingData.hpp>
#include <rayUtil.hpp>

#include "cmRaySampler.hpp"
#include "cmRaySourceGeometry.hpp"

#define PRINT_PROGRESS false
#define PRINT_RESULT false

template <typename NumericType, int D> class cmRayTraceGraphKernel {
  RTCDevice &mDevice;
  rayGeometry<NumericType, D> &mGeometry;
  cmRaySourceGeometry<NumericType, D> &mSource;
  rayBoundary<NumericType, D> &mBoundary;
  cmRaySampler<NumericType, D> &mSampler;
  rayTraceInfo *mTraceInfo = nullptr;
  const bool mUseRandomSeeds;
  const size_t mRunNumber;

public:
  cmRayTraceGraphKernel(RTCDevice &pDevice,
                        rayGeometry<NumericType, D> &pRTCGeometry,
                        cmRaySourceGeometry<NumericType, D> &pRTCSource,
                        rayBoundary<NumericType, D> &pRTCBoundary,
                        cmRaySampler<NumericType, D> &pSampler,
                        const bool pUseRandomSeeds, const size_t pRunNumber)
      : mDevice(pDevice), mGeometry(pRTCGeometry), mSource(pRTCSource),
        mBoundary(pRTCBoundary), mSampler(pSampler),
        mUseRandomSeeds(pUseRandomSeeds), mRunNumber(pRunNumber) {
    assert(rtcGetDeviceProperty(mDevice, RTC_DEVICE_PROPERTY_VERSION) >=
               30601 &&
           "Error: The minimum version of Embree is 3.6.1");
  }

  std::tuple<
      lsSmartPointer<std::vector<long>>, lsSmartPointer<std::vector<float>>,
      lsSmartPointer<std::vector<float>>, lsSmartPointer<std::vector<float>>,
      lsSmartPointer<std::vector<float>>>
  apply() {
    auto rtcScene = rtcNewScene(mDevice);

    // RTC scene flags
    rtcSetSceneFlags(rtcScene, RTC_SCENE_FLAG_NONE);

    // Selecting higher build quality results in better rendering performance
    // but slower scene commit times. The default build quality for a scene is
    // RTC_BUILD_QUALITY_MEDIUM.
    auto bbquality = RTC_BUILD_QUALITY_HIGH;
    rtcSetSceneBuildQuality(rtcScene, bbquality);
    auto rtcGeometry = mGeometry.getRTCGeometry();
    auto rtcSource = mSource.getRTCGeometry();
    auto rtcBoundary = mBoundary.getRTCGeometry();

    auto boundaryID = rtcAttachGeometry(rtcScene, rtcBoundary);
    auto sourceID = rtcAttachGeometry(rtcScene, rtcSource);
    auto geometryID = rtcAttachGeometry(rtcScene, rtcGeometry);
    assert(rtcGetDeviceError(mDevice) == RTC_ERROR_NONE &&
           "Embree device error");

    const long numOfRaysPerPoint = mSampler.getNumberOfRaysPerPoint();

    size_t geohitc = 0;
    size_t nongeohitc = 0;
    size_t totaltraces = 0;

    // thread local data storage
    const int numThreads = omp_get_max_threads();

    // edges
    std::vector<std::vector<long>> threadLocalEdges(numThreads);
    for (auto &e : threadLocalEdges)
      e = std::vector<long>(2 * numOfRaysPerPoint * mGeometry.getNumPoints(),
                            -1);

    // edge lengths
    std::vector<std::vector<NumericType>> threadLocalEdgeLengths(numThreads);
    for (auto &l : threadLocalEdgeLengths)
      l = std::vector<NumericType>(numOfRaysPerPoint * mGeometry.getNumPoints(),
                                   -1.);

    // Outbound Angles
    std::vector<std::vector<NumericType>> threadLocalOutboundAngles(numThreads);
    for (auto &l : threadLocalOutboundAngles)
      l = std::vector<NumericType>(numOfRaysPerPoint * mGeometry.getNumPoints(),
                                   -1.);

    // Inbound Angles
    std::vector<std::vector<NumericType>> threadLocalInboundAngles(numThreads);
    for (auto &l : threadLocalInboundAngles)
      l = std::vector<NumericType>(numOfRaysPerPoint * mGeometry.getNumPoints(),
                                   -1.);

    // Source Connection
    std::vector<std::vector<int>> threadLocalSourceConnections(numThreads);
    for (auto &l : threadLocalSourceConnections)
      l = std::vector<int>(numOfRaysPerPoint * mGeometry.getNumPoints(), -1);

    auto time = rayInternal::timeStampNow<std::chrono::milliseconds>();

#pragma omp parallel reduction(+: geohitc, nongeohitc, totaltraces) \
shared(threadLocalEdges, threadLocalEdgeLengths, threadLocalInboundAngles)
    {
      rtcJoinCommitScene(rtcScene);

      alignas(128) auto rayHit =
          RTCRayHit{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

      const int threadID = omp_get_thread_num();

      constexpr int numRngStates = 8;
      unsigned int seeds[numRngStates];
      if (mUseRandomSeeds) {
        std::random_device rd;
        for (size_t i = 0; i < numRngStates; ++i) {
          seeds[i] = static_cast<unsigned int>(rd());
        }
      } else {
        for (size_t i = 0; i < numRngStates; ++i) {
          seeds[i] =
              static_cast<unsigned int>((threadID + 1) * 31 + i + mRunNumber);
        }
      }
      // It seems really important to use two separate seeds / states for
      // sampling the source and sampling reflections. When we use only one
      // state for both, then the variance is very high.
      rayRNG RngState1(seeds[0]);

      auto &myLocalEdges = threadLocalEdges[threadID];
      auto &myLocalEdgeLengths = threadLocalEdgeLengths[threadID];
      auto &myLocalOutboundAngles = threadLocalOutboundAngles[threadID];
      auto &myLocalInboundAngles = threadLocalInboundAngles[threadID];
      auto &myLocalSourceConnection = threadLocalSourceConnections[threadID];

      const int sourceDir = mSource.getSourceDir();
      const auto &sourceCenter = mSource.getSourceCenter();
      const auto &sourceNormal = mSource.getSourceNormal();

      auto rtcContext = RTCIntersectContext{};
      rtcInitIntersectContext(&rtcContext);

#pragma omp for schedule(dynamic)
      for (long long idx = 0; idx < mGeometry.getNumPoints(); ++idx) {
        // For each point of the surface launch numRaysPerPoint rays
        // uniformly distributed
        const auto surfacePoint = mGeometry.getPoint(idx);
        const auto surfaceNormal = mGeometry.getPrimNormal(idx);

        for (long dirIdx = 0; dirIdx < numOfRaysPerPoint; ++dirIdx) {
          unsigned dataIndex = numOfRaysPerPoint * idx + dirIdx;

          auto &ray = rayHit.ray;
          auto rayDirection =
              mSampler.getDirection(RngState1, surfaceNormal, dirIdx);

#ifdef ARCH_X86
          reinterpret_cast<__m128 &>(ray) =
              _mm_set_ps(1e-4f, (float)surfacePoint[2], (float)surfacePoint[1],
                         (float)surfacePoint[0]);

          reinterpret_cast<__m128 &>(ray.dir_x) =
              _mm_set_ps(0.0f, (float)rayDirection[2], (float)rayDirection[1],
                         (float)rayDirection[0]);
#else
          ray.org_x = (float)surfacePoint[0];
          ray.org_y = (float)surfacePoint[1];
          ray.org_z = (float)surfacePoint[2];
          ray.tnear = 1e-4f;

          ray.dir_x = (float)rayDirection[0];
          ray.dir_y = (float)rayDirection[1];
          ray.dir_z = (float)rayDirection[2];
          ray.time = 0.0f;
#endif
#ifdef VIENNARAY_USE_RAY_MASKING
          ray.mask = -1;
#endif

          bool active = true;
          bool hitFromBack = false;
          do {
            rayHit.ray.tfar = std::numeric_limits<rtcNumericType>::max();
            rayHit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            rayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            // rayHit.ray.tnear = 1e-4f; // tnear is also set in the particle
            // source

            // Run the intersection
            rtcIntersect1(rtcScene, &rtcContext, &rayHit);
            ++totaltraces;

            /* -------- No hit -------- */
            if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
              ++nongeohitc;
              break;
            }

            /* -------- Boundary hit -------- */
            if (rayHit.hit.geomID == boundaryID) {
              mBoundary.processHit(rayHit, active);
              // break;
              continue;
            }

            const rtcNumericType xx = ray.org_x + ray.dir_x * ray.tfar;
            const rtcNumericType yy = ray.org_y + ray.dir_y * ray.tfar;
            const rtcNumericType zz = ray.org_z + ray.dir_z * ray.tfar;

            const auto rayDir =
                rayTriple<NumericType>{ray.dir_x, ray.dir_y, ray.dir_z};
            const auto invRayDir =
                rayTriple<NumericType>{-ray.dir_x, -ray.dir_y, -ray.dir_z};
            const auto rayOrigin =
                rayTriple<NumericType>{ray.org_x, ray.org_y, ray.org_z};

            const auto primID = rayHit.hit.primID;
            const auto &geomNormal = mGeometry.getPrimNormal(primID);
            const auto &disk = mGeometry.getPrimRef(primID);
            const auto &diskOrigin =
                *reinterpret_cast<rayTriple<rtcNumericType> const *>(&disk);
            const auto &diskNormal = mGeometry.getPrimNormal(idx);

            /* ------- Source Plane Hit ----- */
            if (rayHit.hit.geomID == sourceID) {
              myLocalEdges[2 * dataIndex] = idx;
              myLocalEdges[2 * dataIndex + 1] = mGeometry.getNumPoints();

              myLocalEdgeLengths[dataIndex] =
                  std::abs(rayOrigin[sourceDir] - sourceCenter[sourceDir]);

              float outboundDot = std::min(
                  std::max(rayInternal::DotProduct(diskNormal, rayDir), -1.),
                  1.);
              myLocalOutboundAngles[dataIndex] = std::acos(outboundDot);

              float inboundDot = std::min(
                  std::max(rayInternal::DotProduct(sourceNormal, invRayDir),
                           -1.),
                  1.);
              myLocalInboundAngles[dataIndex] = std::acos(inboundDot);

              myLocalSourceConnection[dataIndex] = 1;

              break;
            }

            /* -------- Hit from back -------- */
            if (rayInternal::DotProduct(rayDir, geomNormal) > 0) {
              // If the dot product of the ray direction and the surface
              // normal is greater than zero, then we hit the back face of the
              // disk. Here we discard such hits.
              break;
            }

            /* -------- Surface hit -------- */
            assert(rayHit.hit.geomID == geometryID &&
                   "Geometry hit ID invalid");
            ++geohitc;
            myLocalEdges[2 * dataIndex] = idx;
            myLocalEdges[2 * dataIndex + 1] = primID;

            myLocalEdgeLengths[dataIndex] = ray.tfar;

            float outboundDot = std::min(
                std::max(rayInternal::DotProduct(diskNormal, rayDir), -1.), 1.);
            myLocalOutboundAngles[dataIndex] = std::acos(outboundDot);

            float inboundDot = std::min(
                std::max(rayInternal::DotProduct(geomNormal, invRayDir), -1.),
                1.);
            myLocalInboundAngles[dataIndex] = std::acos(inboundDot);
            myLocalSourceConnection[dataIndex] = 0;

            active = false;
          } while (active);
        }
      } // end ray tracing for loop
    }   // end parallel section

    auto endTime = rayInternal::timeStampNow<std::chrono::milliseconds>();

    auto edges = lsSmartPointer<std::vector<long>>::New();
    edges->reserve(threadLocalEdges[0].size());

    auto edgeLengths = lsSmartPointer<std::vector<float>>::New();
    edgeLengths->reserve(threadLocalEdgeLengths[0].size());

    auto outboundAngles = lsSmartPointer<std::vector<float>>::New();
    outboundAngles->reserve(threadLocalOutboundAngles[0].size());

    auto inboundAngles = lsSmartPointer<std::vector<float>>::New();
    inboundAngles->reserve(threadLocalInboundAngles[0].size());

    auto sourceConnections = lsSmartPointer<std::vector<float>>::New();
    sourceConnections->reserve(threadLocalSourceConnections[0].size());

    // Combine the edges
    for (size_t j = 0; j < threadLocalEdges[0].size(); j += 2) {
      for (int k = 0; k < numThreads; ++k) {
        // Only update the value if an actual intersection point was stored
        // in the thread local version
        const auto n1 = threadLocalEdges[k][j];
        if (n1 >= 0) {
          const auto n2 = threadLocalEdges[k][j + 1];
          edges->push_back(n1);
          edges->push_back(n2);
          break;
        }
      }
    }
    // Combine the edge attributes
    for (size_t j = 0; j < threadLocalEdgeLengths[0].size(); ++j) {
      for (int k = 0; k < numThreads; ++k) {
        // Only update the value if an actual intersection point was stored
        // in the thread local version
        const auto &length = threadLocalEdgeLengths[k][j];
        const auto &outboundAngle = threadLocalOutboundAngles[k][j];
        const auto &inboundAngle = threadLocalInboundAngles[k][j];
        const auto &sourceConnection = threadLocalSourceConnections[k][j];
        if (length >= 0) {
          edgeLengths->push_back(static_cast<float>(length));
          outboundAngles->push_back(static_cast<float>(outboundAngle));
          inboundAngles->push_back(static_cast<float>(inboundAngle));
          sourceConnections->push_back(static_cast<float>(sourceConnection));
          break;
        }
      }
    }

    if (mTraceInfo != nullptr) {
      mTraceInfo->totalRaysTraced = totaltraces;
      mTraceInfo->nonGeometryHits = nongeohitc;
      mTraceInfo->geometryHits = geohitc;
      mTraceInfo->time = (endTime - time) * 1e-3;
    }

    rtcReleaseGeometry(rtcGeometry);
    rtcReleaseGeometry(rtcSource);
    rtcReleaseGeometry(rtcBoundary);

    return {edges, edgeLengths, outboundAngles, inboundAngles,
            sourceConnections};
  }

  void setRayTraceInfo(rayTraceInfo *pTraceInfo) { mTraceInfo = pTraceInfo; }
};