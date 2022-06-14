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

#include <cmGraphBuilder.hpp>
#include <cmGraphData.hpp>
#include <cmRaySampler.hpp>
#include <cmRaySourceGeometry.hpp>

#define PRINT_PROGRESS false
#define PRINT_RESULT false

template <typename NumericType, int D, typename GraphNumericType = NumericType>
class cmRayTraceGraphKernel {
public:
  cmRayTraceGraphKernel(
      RTCDevice &pDevice, rayGeometry<NumericType, D> &pRTCGeometry,
      cmRaySourceGeometry<NumericType, D> &pRTCSource,
      rayBoundary<NumericType, D> &pRTCBoundary,
      cmRaySampler<NumericType, D> &pSampler,
      std::unique_ptr<cmAbstractGraphBuilder<NumericType, GraphNumericType>>
          &pBuilder,
      const bool pUseRandomSeeds, const size_t pRunNumber)
      : mDevice(pDevice), mGeometry(pRTCGeometry), mSource(pRTCSource),
        mBoundary(pRTCBoundary), mSampler(pSampler),
        mBuilder(pBuilder->clone()), mUseRandomSeeds(pUseRandomSeeds),
        mRunNumber(pRunNumber) {
    assert(rtcGetDeviceProperty(mDevice, RTC_DEVICE_PROPERTY_VERSION) >=
               30601 &&
           "Error: The minimum version of Embree is 3.6.1");
  }

  void apply() {
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

    std::vector<cmGraphData<GraphNumericType>> threadLocalGraphData(numThreads);
    for (auto &data : threadLocalGraphData) {
      data = *localGraphData;
    }

    auto time = rayInternal::timeStampNow<std::chrono::milliseconds>();

#pragma omp parallel reduction(+: geohitc, nongeohitc, totaltraces) \
shared(threadLocalGraphData)
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
      rayRNG RngState5(seeds[4]);

      // thread-local graph builder object
      auto builder = mBuilder->clone();

      auto &myLocalGraphData = threadLocalGraphData[threadID];

      const int sourceDir = mSource.getSourceDir();
      const int sourcePosNeg = mSource.getPosNeg();
      const int sourceColID = mGeometry.getNumPoints();

      const auto &sourceCenter = mSource.getSourceCenter();
      const auto &sourceNormal = mSource.getSourceNormal();

      auto rtcContext = RTCIntersectContext{};
      rtcInitIntersectContext(&rtcContext);

#pragma omp for schedule(dynamic)
      for (long long idx = 0; idx < mGeometry.getNumPoints(); ++idx) {
        // For each point of the surface launch numRaysPerPoint rays
        // uniformly distributed
        const auto rayOrigin = mGeometry.getPoint(idx);
        const auto surfaceNormal = mGeometry.getPrimNormal(idx);

        for (long dirIdx = 0; dirIdx < numOfRaysPerPoint; ++dirIdx) {
          unsigned dataIndex = numOfRaysPerPoint * idx + dirIdx;

          auto &ray = rayHit.ray;
          auto rayDir = mSampler.getDirection(RngState1, surfaceNormal, dirIdx);

          // Ensure that the direction is normalized
          rayInternal::Normalize(rayDir);

          fillRay(rayHit.ray, rayOrigin, rayDir);

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
              // TODO: Properly handle boundary hits!
              // mBoundary.processHit(rayHit, active);
              break;
            }

            const auto primID = rayHit.hit.primID;
            const auto &geomNormal = mGeometry.getPrimNormal(primID);

            /* ------- Source Plane Hit ----- */
            if (rayHit.hit.geomID == sourceID) {
              builder->sourceCollision(idx, rayOrigin, rayDir, geomNormal,
                                       sourceColID, sourceCenter, sourceDir,
                                       sourcePosNeg, myLocalGraphData,
                                       globalData, RngState5);
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
#ifdef MULTI_INTERSECT
            std::vector<unsigned int> hitDiskIds(1, rayHit.hit.primID);

            // check for additional intersections
            for (const auto &id :
                 mGeometry.getNeighborIndicies(rayHit.hit.primID)) {
              rtcNumericType distance;
              if (checkLocalIntersection(ray, id, distance)) {
                hitDiskIds.push_back(id);
              }
            }
            const size_t numDisksHit = hitDiskIds.size();

            // for each disk hit
            for (size_t diskId = 0; diskId < numDisksHit; ++diskId) {
              const auto matID = mGeometry.getMaterialId(hitDiskIds[diskId]);
              const auto normal = mGeometry.getPrimNormal(hitDiskIds[diskId]);
              builder->surfaceCollision(idx, ray, normal, hitDiskIds[diskId],
                                        matID, myLocalGraphData, globalData,
                                        RngState5);
            }
#else
            const auto matID = mGeometry.getMaterialId(primID);
            builder->surfaceCollision(idx, ray, geomNormal, primID, matID,
                                      myLocalGraphData, globalData, RngState5);
#endif
            // Stop after this one iteration (we don't use any reflections)
            break;
          } while (active);
        }
      } // end ray tracing for loop
    }   // end parallel section

    auto endTime = rayInternal::timeStampNow<std::chrono::milliseconds>();

    // Merge node data
    if (!localGraphData->getNodeData().empty()) {
    }

    // Merge edges. Edges and edge data are always appended
    localGraphData->getEdges().clear();
    for (int k = 0; k < numThreads; ++k) {
      localGraphData->appendEdges(threadLocalGraphData[k].getEdges());
    }
    // Merge edge data
    if (!localGraphData->getEdgeData().empty()) {
#pragma omp parallel for
      for (int i = 0; i < localGraphData->getEdgeData().size(); ++i) {
        localGraphData->getEdgeData(i).clear();
        for (int k = 0; k < numThreads; ++k) {
          localGraphData->appendEdgeData(
              i, threadLocalGraphData[k].getEdgeData(i));
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
  }

  void setTracingData(cmGraphData<GraphNumericType> *plocalGraphData,
                      const rayTracingData<NumericType> *pglobalData) {
    localGraphData = plocalGraphData;
    globalData = pglobalData;
  }

  void setRayTraceInfo(rayTraceInfo *pTraceInfo) { mTraceInfo = pTraceInfo; }

private:
  void fillRay(RTCRay &ray, const rayTriple<NumericType> &rayOrigin,
               rayTriple<NumericType> &rayDirection) const {
#ifdef ARCH_X86
    reinterpret_cast<__m128 &>(ray) = _mm_set_ps(
        1e-4f, (float)rayOrigin[2], (float)rayOrigin[1], (float)rayOrigin[0]);

    reinterpret_cast<__m128 &>(ray.dir_x) =
        _mm_set_ps(0.0f, (float)rayDirection[2], (float)rayDirection[1],
                   (float)rayDirection[0]);
#else
    ray.org_x = (float)rayOrigin[0];
    ray.org_y = (float)rayOrigin[1];
    ray.org_z = (float)rayOrigin[2];
    ray.tnear = 1e-4f;

    ray.dir_x = (float)rayDirection[0];
    ray.dir_y = (float)rayDirection[1];
    ray.dir_z = (float)rayDirection[2];
    ray.time = 0.0f;
#endif
#ifdef VIENNARAY_USE_RAY_MASKING
    ray.mask = -1;
#endif
  }

  bool checkLocalIntersection(RTCRay const &ray, const unsigned int primID,
                              rtcNumericType &impactDistance) {
    auto const &rayOrigin =
        *reinterpret_cast<rayTriple<rtcNumericType> const *>(&ray.org_x);
    auto const &rayDirection =
        *reinterpret_cast<rayTriple<rtcNumericType> const *>(&ray.dir_x);

    const auto &normal = mGeometry.getNormalRef(primID);
    const auto &disk = mGeometry.getPrimRef(primID);
    const auto &diskOrigin =
        *reinterpret_cast<rayTriple<rtcNumericType> const *>(&disk);

    auto prodOfDirections = rayInternal::DotProduct(normal, rayDirection);
    if (prodOfDirections > 0.f) {
      // Disk normal is pointing away from the ray direction,
      // i.e., this might be a hit from the back or no hit at all.
      return false;
    }

    constexpr auto eps = 1e-6f;
    if (std::fabs(prodOfDirections) < eps) {
      // Ray is parallel to disk surface
      return false;
    }

    // TODO: Memoize ddneg
    auto ddneg = rayInternal::DotProduct(diskOrigin, normal);
    auto tt =
        (ddneg - rayInternal::DotProduct(normal, rayOrigin)) / prodOfDirections;
    if (tt <= 0) {
      // Intersection point is behind or exactly on the ray origin.
      return false;
    }

    // copy ray direction
    auto rayDirectionC = rayTriple<rtcNumericType>{
        rayDirection[0], rayDirection[1], rayDirection[2]};
    rayInternal::Scale(tt, rayDirectionC);
    auto hitpoint = rayInternal::Sum(rayOrigin, rayDirectionC);
    auto diskOrigin2HitPoint = rayInternal::Diff(hitpoint, diskOrigin);
    auto distance = rayInternal::Norm(diskOrigin2HitPoint);
    auto const &radius = disk[3];
    if (radius > distance) {
      impactDistance = distance;
      return true;
    }
    return false;
  }

  RTCDevice &mDevice;
  rayGeometry<NumericType, D> &mGeometry;
  cmRaySourceGeometry<NumericType, D> &mSource;
  rayBoundary<NumericType, D> &mBoundary;
  cmRaySampler<NumericType, D> &mSampler;
  std::unique_ptr<cmAbstractGraphBuilder<NumericType, GraphNumericType>> const
      mBuilder = nullptr;
  const bool mUseRandomSeeds;
  const size_t mRunNumber;
  cmGraphData<GraphNumericType> *localGraphData = nullptr;
  const rayTracingData<NumericType> *globalData = nullptr;
  rayTraceInfo *mTraceInfo = nullptr;
};