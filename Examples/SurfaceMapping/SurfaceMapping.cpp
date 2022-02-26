#include <lsFromSurfaceMesh.hpp>

#include <lsMesh.hpp>
#include <lsVTKReader.hpp>
#include <lsVTKWriter.hpp>

#include "cmSurfaceMapper.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(baseMesh, "first.vtk").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(depoMesh, "second.vtk").apply();

  cmSurfaceMapper<NumericType, D>(baseMesh, depoMesh).apply();

  lsVTKWriter<NumericType>(depoMesh, "mapped_mesh.vtp").apply();
  lsVTKWriter<NumericType>(baseMesh, "annotated_mesh.vtp").apply();
}