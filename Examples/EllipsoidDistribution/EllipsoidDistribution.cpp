#include <lsBooleanOperation.hpp>
#include <lsGeometricAdvect.hpp>
#include <lsMakeGeometry.hpp>
#include <lsToSurfaceMesh.hpp>
#include <lsVTKWriter.hpp>

#include "cmEllipsoidDistribution.hpp"

template <typename NumericType, int D>
void printLS(lsSmartPointer<lsDomain<NumericType, D>> dom, std::string name) {
  auto mesh = lsSmartPointer<lsMesh<NumericType>>::New();
  lsToSurfaceMesh<NumericType, D>(dom, mesh).apply();
  lsVTKWriter<NumericType>(mesh, name).apply();
}

template <typename NumericType>
void makeTaperedTrench(lsSmartPointer<lsMesh<>> mesh,
                       hrleVectorType<NumericType, 2> center,
                       hrleVectorType<NumericType, 2> taperAngle,
                       double diameter, NumericType depth) {
  auto cloud = lsSmartPointer<lsPointCloud<NumericType, 2>>::New();
  {
    // top left
    hrleVectorType<NumericType, 2> point1(-diameter / 2., 0.);
    cloud->insertNextPoint(point1);
    // top right
    hrleVectorType<NumericType, 2> point2(diameter / 2., 0.);
    cloud->insertNextPoint(point2);
    // bottom right
    hrleVectorType<NumericType, 2> point3(
        diameter / 2. - (depth * taperAngle[1] / taperAngle[0]), -depth);
    cloud->insertNextPoint(point3);
    // bottom left
    hrleVectorType<NumericType, 2> point4(
        -diameter / 2. + (depth * taperAngle[1] / taperAngle[0]), -depth);
    cloud->insertNextPoint(point4);
  }
  lsConvexHull<NumericType, 2>(mesh, cloud).apply();
}

int main() {
  static constexpr int D = 2;
  using NumericType = double;

  static constexpr NumericType gridDelta = .5;

  NumericType diameter = 20;
  NumericType depth = 50;

  // Process parameters
  NumericType extent = diameter;
  NumericType bounds[2 * D] = {-extent, extent, -extent, extent};
  if constexpr (D == 3) {
    bounds[4] = -extent;
    bounds[5] = extent;
  }

  typename lsDomain<NumericType, D>::BoundaryType boundaryCons[D];
  for (unsigned i = 0; i < D - 1; ++i) {
    boundaryCons[i] =
        lsDomain<NumericType, D>::BoundaryType::REFLECTIVE_BOUNDARY;
  }
  boundaryCons[D - 1] =
      lsDomain<NumericType, D>::BoundaryType::INFINITE_BOUNDARY;

  auto substrate = lsSmartPointer<lsDomain<NumericType, D>>::New(
      bounds, boundaryCons, gridDelta);
  {
    NumericType origin[D] = {0.};
    NumericType planeNormal[D] = {0.};
    planeNormal[D - 1] = 1.;
    auto plane =
        lsSmartPointer<lsPlane<NumericType, D>>::New(origin, planeNormal);
    lsMakeGeometry<NumericType, D>(substrate, plane).apply();
  }

  {
    auto hole = lsSmartPointer<lsDomain<NumericType, D>>::New(
        bounds, boundaryCons, gridDelta);

    // create hole

    hrleVectorType<NumericType, D> minCorner;
    minCorner[0] = -diameter / 2;
    minCorner[D - 1] = -depth;
    if constexpr (D == 3)
      minCorner[1] = -diameter / 2;

    hrleVectorType<NumericType, D> maxCorner;
    maxCorner[0] = diameter / 2;
    maxCorner[D - 1] = gridDelta;
    if constexpr (D == 3)
      maxCorner[1] = diameter / 2;

    auto box = lsSmartPointer<lsBox<NumericType, D>>::New(minCorner, maxCorner);

    lsMakeGeometry<NumericType, D>(hole, box).apply();

    lsBooleanOperation<NumericType, D>(
        substrate, hole, lsBooleanOperationEnum::RELATIVE_COMPLEMENT)
        .apply();
  }

  auto depoLayer = lsSmartPointer<lsDomain<NumericType, D>>::New(substrate);

  printLS(depoLayer, "surface_i.vtk");

  auto dist = lsSmartPointer<cmEllipsoidDistribution<hrleCoordType, D>>::New(
      1., 5., 1., gridDelta);

  lsGeometricAdvect<NumericType, D>(depoLayer, dist).apply();

  printLS(depoLayer, "surface_f.vtk");
}