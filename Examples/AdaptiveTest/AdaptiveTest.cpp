#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#include <lsCalculateCurvatures.hpp>
#include <lsCalculateNormalVectors.hpp>
#include <lsExpand.hpp>
#include <lsGeometricAdvect.hpp>
#include <lsMakeGeometry.hpp>
#include <lsMesh.hpp>
#include <lsPrune.hpp>
#include <lsReader.hpp>
#include <lsToDiskMesh.hpp>
#include <lsToMesh.hpp>
#include <lsToSurfaceMesh.hpp>
#include <lsVTKWriter.hpp>

#include "AdaptiveDistribution.hpp"

#include "cmVectorHash.hpp"

template <class T, int D> lsSmartPointer<lsDomain<T, D>> generateGeometry() {
  T gridDelta = .5;
  T diameter = 20.;
  T depth = 40.;

  hrleVectorType<T, D> center;
  hrleVectorType<T, D> taperNormal;

  double extent = diameter;
  double bounds[2 * D] = {-extent, extent, -extent, extent};
  if constexpr (D == 3) {
    bounds[4] = -extent;
    bounds[5] = extent;
  }

  typename lsDomain<T, D>::BoundaryType boundaryCons[D];
  for (unsigned i = 0; i < D - 1; ++i) {
    boundaryCons[i] = lsDomain<T, D>::BoundaryType::REFLECTIVE_BOUNDARY;
  }
  boundaryCons[D - 1] = lsDomain<T, D>::BoundaryType::INFINITE_BOUNDARY;

  auto substrate =
      lsSmartPointer<lsDomain<T, D>>::New(bounds, boundaryCons, gridDelta);
  {
    T origin[D] = {0.};
    T planeNormal[D] = {0.};
    planeNormal[D - 1] = 1.;

    auto plane = lsSmartPointer<lsPlane<T, D>>::New(origin, planeNormal);
    lsMakeGeometry<T, D>(substrate, plane).apply();
  }
  {
    auto hole =
        lsSmartPointer<lsDomain<T, D>>::New(bounds, boundaryCons, gridDelta);

    hrleVectorType<T, D> minCorner;
    minCorner[0] = -diameter / 2;
    minCorner[D - 1] = -depth;
    if constexpr (D == 3)
      minCorner[1] = -diameter / 2;

    hrleVectorType<T, D> maxCorner;
    maxCorner[0] = diameter / 2;
    maxCorner[D - 1] = gridDelta;
    if constexpr (D == 3)
      maxCorner[1] = diameter / 2;

    auto box = lsSmartPointer<lsBox<T, D>>::New(minCorner, maxCorner);

    lsMakeGeometry<T, D>(hole, box).apply();

    lsBooleanOperation<T, D>(substrate, hole,
                             lsBooleanOperationEnum::RELATIVE_COMPLEMENT)
        .apply();
  }
  return substrate;
};

int main() {
  static constexpr int D = 3;
  using NumericType = double;
  using VectorType = std::array<NumericType, 3>;
  using MapType = std::unordered_map<VectorType, std::array<NumericType, D * D>,
                                     cmVectorHash<VectorType>>;

  auto substrate = generateGeometry<NumericType, D>();

  lsExpand<NumericType, D>(substrate, 5).apply();

  lsCalculateNormalVectors<NumericType, D>(substrate).apply();
  lsCalculateCurvatures<NumericType, D>(substrate).apply();

  {
    auto mesh = lsSmartPointer<lsMesh<>>::New();
    lsToSurfaceMesh<NumericType, D>(substrate, mesh).apply();
    lsVTKWriter<NumericType>(mesh, "first.vtk").apply();
  }

  std::vector<std::array<hrleCoordType, 3>> normals =
      *(substrate->getPointData().getVectorData(
          lsCalculateNormalVectors<NumericType, D>::normalVectorsLabel));

  std::vector<NumericType> curvatures =
      *(substrate->getPointData().getScalarData("MeanCurvatures"));

  MapType map;

  auto dist =
      lsSmartPointer<AdaptiveDistribution<NumericType, D, MapType>>::New(
          substrate, map, normals, curvatures,
          substrate->getGrid().getGridDelta());

  lsGeometricAdvect<NumericType, D, true>(substrate, dist).apply();

  lsPrune<NumericType, D>(substrate).apply();

  {
    auto mesh = lsSmartPointer<lsMesh<>>::New();
    lsToSurfaceMesh<NumericType, D>(substrate, mesh).apply();
    lsVTKWriter<NumericType>(mesh, "second.vtk").apply();
  }
}