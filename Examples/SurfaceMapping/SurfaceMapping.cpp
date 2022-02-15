#include <lsFromSurfaceMesh.hpp>

#include <lsMesh.hpp>
#include <lsVTKWriter.hpp>
#include <lsVTKReader.hpp>

#include "cmSurfaceMapper.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(baseMesh, "substrate.vtp").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(depoMesh, "deposited.vtp").apply();

  cmSurfaceMapper<NumericType, D>(baseMesh, depoMesh).apply();

  lsVTKWriter<NumericType>(depoMesh, "depoMesh.vtp").apply();
}