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

template <class T, int D>
static constexpr std::array<T, D * D>
generateNormalBasis(const std::array<T, 3> &n) {
  // Row major representation
  // Calculates normal basis using Gram Schmidt procedure with vector n as
  // initial vector. If n is not parallel to the unit vector in axis D-1, this
  // axis will be used as second orientation vector in the gram schmidt
  // procedure. In 3D the third basis vector is calculated by taking the
  // cross-product of the two other base vectors.

  std::array<T, D * D> basis({0.});
  basis[0] = n[0];
  basis[D] = n[1];
  if constexpr (D == 3)
    basis[2 * D] = n[2];

  constexpr int Z = D - 1;
  if (std::abs(n[Z]) != 1.) {
    T sum{0.};
    for (unsigned i = 0; i < D; ++i) {
      T tmp = 1. * (i == Z) - n[Z] * n[i];
      basis[(i * D) + 1] = tmp;
      sum += tmp * tmp;
    }

    if (sum != 1)
      sum = std::sqrt(sum);

    for (unsigned i = 0; i < D; ++i)
      basis[(i * D) + 1] /= sum;

  } else {
    // If n is parallel to the normal vector in axis D-1, simply use unit vector
    // in x-direction as basis vector.
    basis[1] = 1.;
  }

  if constexpr (D == 3) {
    // Cross Product
    basis[2] = basis[3] * basis[7] - basis[6] * basis[4];
    basis[5] = basis[6] * basis[1] - basis[0] * basis[7];
    basis[8] = basis[0] * basis[4] - basis[3] * basis[1];
  }

  return basis;
}

int main() {
  using NumericType = double;
  static constexpr int D = 3;

  auto substrate = generateGeometry<NumericType, D>();

  /*
  using NumericType = double;

  static constexpr int D = 2;

  auto substrate = lsSmartPointer<lsDomain<NumericType, D>>::New();

  lsReader<NumericType, D>(substrate, "first.lvst").apply();
*/
  lsExpand<NumericType, D>(substrate, 5).apply();

  lsCalculateNormalVectors<NumericType, D>(substrate).apply();
  lsCalculateCurvatures<NumericType, D>(substrate).apply();

  {
    auto mesh = lsSmartPointer<lsMesh<>>::New();
    lsToMesh<NumericType, D>(substrate, mesh).apply();
    lsVTKWriter<NumericType>(mesh, "first.vtk").apply();
  }

  std::vector<std::array<hrleCoordType, 3>> normals =
      *(substrate->getPointData().getVectorData(
          lsCalculateNormalVectors<NumericType, D>::normalVectorsLabel));

  std::vector<NumericType> curvatures =
      *(substrate->getPointData().getScalarData("MeanCurvatures"));

  // std::vector<std::array<hrleCoordType, D * D>> bases;
  // std::transform(normals.begin(), normals.end(), std::back_inserter(bases),
  //                [](std::array<hrleCoordType, 3> &v) {
  //                  return generateNormalBasis<NumericType, D>(v);
  //                });

  // std::vector<std::array<hrleCoordType, 3>> t1;
  // for (const auto &basis : bases) {
  //   std::array<hrleCoordType, 3> tmp{0.};
  //   for (unsigned i = 0; i < D; ++i)
  //     tmp[i] = basis[i * D + 1];

  //   t1.push_back(tmp);
  // }

  // substrate->getPointData().insertNextVectorData(t1, "T1");

  // if constexpr (D == 3) {
  //   std::vector<std::array<hrleCoordType, 3>> t2;
  //   for (const auto &basis : bases) {
  //     std::array<hrleCoordType, 3> tmp{0.};
  //     for (unsigned i = 0; i < D; ++i)
  //       tmp[i] = basis[i * D + 2];

  //     t2.push_back(tmp);
  //   }
  //   substrate->getPointData().insertNextVectorData(t2, "T2");
  // }

  using VectorType = std::array<NumericType, 3>;
  using MapType = std::unordered_map<VectorType, std::array<NumericType, D *
  D>,
                                     cmVectorHash<VectorType>>;

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