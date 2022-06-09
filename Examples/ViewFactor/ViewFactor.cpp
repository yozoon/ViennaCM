#include <chrono>
#include <iostream>

#include <lsBooleanOperation.hpp>
#include <lsExpand.hpp>
#include <lsGeometricAdvect.hpp>
#include <lsMakeGeometry.hpp>
#include <lsToDiskMesh.hpp>
#include <lsToMesh.hpp>
#include <lsToSurfaceMesh.hpp>
#include <lsVTKWriter.hpp>
#include <lsWriteVisualizationMesh.hpp>

#include <lsCalculateCurvatures.hpp>

#include "ViewFactorProcess.hpp"

using NumericType = double;

void makeTaperedTrench(lsSmartPointer<lsMesh<>> mesh,
                       hrleVectorType<NumericType, 2> center,
                       hrleVectorType<NumericType, 2> sidewallNormal,
                       NumericType diameter, NumericType depth) {
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
        diameter / 2. - (depth * sidewallNormal[1] / sidewallNormal[0]),
        -depth);
    cloud->insertNextPoint(point3);
    // bottom left
    hrleVectorType<NumericType, 2> point4(
        -diameter / 2. + (depth * sidewallNormal[1] / sidewallNormal[0]),
        -depth);
    cloud->insertNextPoint(point4);
  }
  lsConvexHull<NumericType, 2>(mesh, cloud).apply();
}

int main() {
  constexpr int D = 2;
  NumericType gridDelta = .1;
  // Process parameters

  NumericType extent = 50;
  NumericType bounds[2 * D] = {-extent, extent, -3 * extent, 3 * extent};
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

  hrleVectorType<NumericType, 2> center(0., 0.);
  hrleVectorType<NumericType, 2> normSide(1, 0);
  NumericType diameter = 30.;
  NumericType depth = 90.;

  auto substrate = lsSmartPointer<lsDomain<NumericType, D>>::New(
      bounds, boundaryCons, gridDelta);
  {
    NumericType origin[D] = {0., 0.};
    NumericType planeNormal[D] = {0., 1.};
    auto plane =
        lsSmartPointer<lsPlane<NumericType, D>>::New(origin, planeNormal);
    lsMakeGeometry<NumericType, D>(substrate, plane).apply();
  }

  // make LS from trench mesh and remove from substrate
  auto trench = lsSmartPointer<lsDomain<NumericType, D>>::New(
      bounds, boundaryCons, gridDelta);
  {
    // create trench
    auto trenchMesh = lsSmartPointer<lsMesh<>>::New();
    makeTaperedTrench(trenchMesh, center, normSide, diameter, depth);
    lsFromSurfaceMesh<NumericType, D>(trench, trenchMesh, false).apply();
    lsBooleanOperation<NumericType, D>(
        substrate, trench, lsBooleanOperationEnum::RELATIVE_COMPLEMENT)
        .apply();
  }

  lsExpand<NumericType, D>(substrate, 5).apply();
  lsCalculateCurvatures<NumericType, D>(substrate).apply();
  
  std::cout << "Output initial" << std::endl;
  auto mesh = lsSmartPointer<lsMesh<>>::New();
  lsToSurfaceMesh<NumericType, D>(substrate, mesh).apply();
  lsVTKWriter(mesh, "surface_i.vtp").apply();

  {
    auto volumeMeshing =
        lsSmartPointer<lsWriteVisualizationMesh<NumericType, D>>::New();
    volumeMeshing->insertNextLevelSet(substrate);
    volumeMeshing->setFileName("volume_i");
    volumeMeshing->apply();
  }

  ViewFactorProcess<NumericType, D> processKernel(substrate);
  processKernel.setTrenchDiameter(diameter);
  processKernel.setTrenchDepth(depth);
  processKernel.setSidewallNormal(normSide);
  processKernel.setTopRate(1.);
  processKernel.setProcessTime(10.);
  processKernel.setTimeStep(1e-4);

  // Run advection
  auto start = std::chrono::high_resolution_clock::now();
  processKernel.apply();
  auto stop = std::chrono::high_resolution_clock::now();
  std::cout << "Geometric advect took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(stop -
                                                                     start)
                   .count()
            << " ms" << std::endl;
  std::cout << "Final structure has " << substrate->getNumberOfPoints()
            << " LS points" << std::endl;

  lsToSurfaceMesh<NumericType, D>(substrate, mesh).apply();
  lsVTKWriter(mesh, "surface_f.vtp").apply();

  std::cout << "Making volume output..." << std::endl;

  {
    auto volumeMeshing =
        lsSmartPointer<lsWriteVisualizationMesh<NumericType, D>>::New();
    volumeMeshing->insertNextLevelSet(substrate);
    volumeMeshing->setFileName("volume_f");
    volumeMeshing->apply();
  }

  return 0;
}
