#include <vector>

#include <lsFromSurfaceMesh.hpp>
#include <lsMesh.hpp>
#include <lsVTKReader.hpp>
#include <lsVTKWriter.hpp>

#include "cmExtractClosestPointThickness.hpp"
#include "cmKDTree.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  // Load both meshes
  auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(baseMesh, "first.vtk").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(depoMesh, "second.vtk").apply();

  auto points = lsSmartPointer<decltype(baseMesh->nodes)>::New(baseMesh->nodes);

  auto thickness = lsSmartPointer<std::vector<NumericType>>::New();

  cmExtractClosestPointThickness<
      typename decltype(baseMesh->nodes)::value_type>(baseMesh, depoMesh,
                                                      thickness)
      .apply();

  baseMesh->getPointData().insertNextScalarData(*thickness, "thickness");

  lsVTKWriter<NumericType>(baseMesh, "layer_thickness.vtk").apply();
}