#ifndef CM_RAY_SOURCE_GEOMETRY_HPP
#define CM_RAY_SOURCE_GEOMETRY_HPP

#include <rayBoundCondition.hpp>
#include <rayMetaGeometry.hpp>
#include <rayPreCompileMacros.hpp>
#include <rayReflection.hpp>
#include <rayTraceDirection.hpp>

template <typename NumericType, int D>
class cmRaySourceGeometry : public rayMetaGeometry<NumericType, D> {
  typedef rayPair<rayTriple<NumericType>> boundingBoxType;

public:
  cmRaySourceGeometry(RTCDevice &pDevice, const boundingBoxType &pBoundingBox,
                      rayTraceBoundary pBoundaryConds[D],
                      std::array<int, 5> &pTraceSettings)
      : mbdBox(pBoundingBox), sourceDir(pTraceSettings[0]),
        posNeg(pTraceSettings[4]) {
    initGeometry(pDevice);
  }

  RTCGeometry &getRTCGeometry() override final { return mRtcBoundary; }

  void releaseGeometry() {
    // Attention:
    // This function must not be called when the RTCGeometry reference count is
    // > 1 Doing so leads to leaked memory buffers
    if (mTriangleBuffer == nullptr || mVertexBuffer == nullptr ||
        mRtcBoundary == nullptr) {
      return;
    } else {
      rtcReleaseGeometry(mRtcBoundary);
      mRtcBoundary = nullptr;
      mTriangleBuffer = nullptr;
      mVertexBuffer = nullptr;
    }
  }

  rayTriple<NumericType> getPrimNormal(const unsigned int primID) override {
    assert(primID < numTriangles && "rayBoundary: primID out of bounds");
    return primNormals[primID];
  }

  rayTriple<NumericType> getSourceNormal() const {
    rayTriple<NumericType> normal{0.};
    normal[sourceDir] = posNeg;
    return normal;
  }

  rayTriple<NumericType> getSourceCenter() const { return sourceCenter; }

  int getSourceDir() const { return sourceDir; }

  boundingBoxType getBoundingBox() const { return mbdBox; }

  rayTriple<NumericType> getPoint(const unsigned int primID) const {
    assert(primID < numVertices && "rayGeometry: Prim ID out of bounds");
    auto const &pnt = mVertexBuffer[primID];
    return {(NumericType)pnt.xx, (NumericType)pnt.yy, (NumericType)pnt.zz};
  }

private:
  void initGeometry(RTCDevice &pDevice) {
    assert(mVertexBuffer == nullptr && mTriangleBuffer == nullptr &&
           "Boundary buffer not empty");
    mRtcBoundary = rtcNewGeometry(pDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    mVertexBuffer = (vertex_f3_t *)rtcSetNewGeometryBuffer(
        mRtcBoundary, RTC_BUFFER_TYPE_VERTEX,
        0, // the slot
        RTC_FORMAT_FLOAT3, sizeof(vertex_f3_t), numVertices);

    auto xmin = mbdBox[0][0];
    auto xmax = mbdBox[1][0];
    auto ymin = mbdBox[0][1];
    auto ymax = mbdBox[1][1];
    auto zmin = mbdBox[0][2];
    auto zmax = mbdBox[1][2];

    std::array<rayTriple<NumericType>, 8> Nodes = {
        (float)xmin, (float)ymin, (float)zmin, //
        (float)xmax, (float)ymin, (float)zmin, //
        (float)xmax, (float)ymax, (float)zmin, //
        (float)xmin, (float)ymax, (float)zmin, //
        (float)xmin, (float)ymin, (float)zmax, //
        (float)xmax, (float)ymin, (float)zmax, //
        (float)xmax, (float)ymax, (float)zmax, //
        (float)xmin, (float)ymax, (float)zmax, //
    };

    static constexpr std::array<rayQuadruple<uint32_t>, 6> Vertices = {
        0, 3, 7, 4, // xLeft
        2, 1, 5, 6, // xRight
        1, 0, 4, 5, // yFront
        3, 2, 6, 7, // yBack
        0, 1, 2, 3, // zBottom
        5, 4, 7, 6, // zTop
    };

    for (size_t idx = 0; idx < numVertices; ++idx) {
      uint32_t vertex = Vertices[2 * sourceDir + (posNeg < 0) * 1][idx];
      mVertexBuffer[idx].xx = Nodes[vertex][0];
      mVertexBuffer[idx].yy = Nodes[vertex][1];
      mVertexBuffer[idx].zz = Nodes[vertex][2];

      sourceCenter[0] += Nodes[vertex][0] / numVertices;
      sourceCenter[1] += Nodes[vertex][1] / numVertices;
      sourceCenter[2] += Nodes[vertex][2] / numVertices;
    }

    mTriangleBuffer = (triangle_t *)rtcSetNewGeometryBuffer(
        mRtcBoundary, RTC_BUFFER_TYPE_INDEX,
        0, // slot
        RTC_FORMAT_UINT3, sizeof(triangle_t), numTriangles);

    mTriangleBuffer[0].v0 = 0;
    mTriangleBuffer[0].v1 = 1;
    mTriangleBuffer[0].v2 = 2;

    mTriangleBuffer[1].v0 = 0;
    mTriangleBuffer[1].v1 = 2;
    mTriangleBuffer[1].v2 = 3;

    for (size_t idx = 0; idx < numTriangles; ++idx) {
      auto triangle = getTriangleCoords(idx);
      auto triNorm = rayInternal::ComputeNormal(triangle);
      rayInternal::Normalize(triNorm);
      primNormals[idx] = triNorm;
    }

#ifdef VIENNARAY_USE_RAY_MASKING
    rtcSetGeometryMask(mRtcBoundary, -1);
#endif

    rtcCommitGeometry(mRtcBoundary);
    assert(rtcGetDeviceError(pDevice) == RTC_ERROR_NONE &&
           "RTC Error: rtcCommitGeometry");
  }

  rayTriple<rayTriple<NumericType>> getTriangleCoords(const size_t primID) {
    assert(primID < numTriangles && "cmRayBounday: primID out of bounds");
    auto tt = mTriangleBuffer[primID];
    return {(NumericType)mVertexBuffer[tt.v0].xx,
            (NumericType)mVertexBuffer[tt.v0].yy,
            (NumericType)mVertexBuffer[tt.v0].zz,
            (NumericType)mVertexBuffer[tt.v1].xx,
            (NumericType)mVertexBuffer[tt.v1].yy,
            (NumericType)mVertexBuffer[tt.v1].zz,
            (NumericType)mVertexBuffer[tt.v2].xx,
            (NumericType)mVertexBuffer[tt.v2].yy,
            (NumericType)mVertexBuffer[tt.v2].zz};
  }

  struct vertex_f3_t {
    // vertex is the nomenclature of Embree
    // The triangle geometry has a vertex buffer which uses x, y, and z
    // in single precision floating point types.
    float xx, yy, zz;
  };
  vertex_f3_t *mVertexBuffer = nullptr;

  struct triangle_t {
    // The triangle geometry uses an index buffer that contains an array
    // of three 32-bit indices per triangle.
    uint32_t v0, v1, v2;
  };
  triangle_t *mTriangleBuffer = nullptr;

  RTCGeometry mRtcBoundary = nullptr;
  const boundingBoxType mbdBox;
  const int sourceDir = 0;
  const int posNeg = -1;
  static constexpr size_t numTriangles = 2;
  static constexpr size_t numVertices = 4;
  std::array<rayTriple<NumericType>, numTriangles> primNormals;
  rayTriple<NumericType> sourceCenter{0.};
};

#endif