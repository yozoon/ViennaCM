#include <lsToSurfaceMesh.hpp>
#include <lsVTKWriter.hpp>

#include <lsVTKReader.hpp>

#include "cmMoveSurface.hpp"

template <typename NumericType, int D>
void printLS(lsSmartPointer<lsDomain<NumericType, D>> dom, std::string name) {
  auto mesh = lsSmartPointer<lsMesh<NumericType>>::New();
  lsToSurfaceMesh<NumericType, D>(dom, mesh).apply();
  lsVTKWriter<NumericType>(mesh, name).apply();
}

int main() {
  constexpr int D = 2;
  using NumericType = double;

  constexpr NumericType gridDelta = 0.5;

  // Process parameters
  NumericType extent = 50;
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

  auto substrateMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(substrateMesh, "substrate.vtp").apply();
  lsFromSurfaceMesh<NumericType, D>(substrate, substrateMesh, false).apply();

  auto movedLayer = lsSmartPointer<lsDomain<NumericType, D>>::New(substrate);

  NumericType thickness = 4.;
  cmMoveSurface<NumericType, D>(substrate, movedLayer, thickness).apply();

  printLS(movedLayer, "moved.vtp");
}