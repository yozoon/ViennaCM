#include <vector>

#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsToDiskMesh.hpp>
#include <lsVTKWriter.hpp>

#include "cmExtractClosestPointThickness.hpp"
#include "cmKDTree.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  auto baseLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(baseLayer, "first.lvst").apply();

  auto depoLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(depoLayer, "second.lvst").apply();

  auto baseMesh = lsSmartPointer<lsMesh<NumericType>>::New();
  lsToDiskMesh<NumericType, D>(baseLayer, baseMesh).apply();

  auto depoMesh = lsSmartPointer<lsMesh<NumericType>>::New();
  lsToDiskMesh<NumericType, D>(depoLayer, depoMesh).apply();

  auto thickness = lsSmartPointer<std::vector<NumericType>>::New();

  using VectorType = typename decltype(baseMesh->nodes)::value_type;

  auto extractor =
      cmExtractClosestPointThickness<VectorType, cmKDTree<VectorType>>(
          baseMesh, depoMesh);
  extractor.apply();

  lsVTKWriter<NumericType>(extractor.getBaseMesh(), "first.vtk").apply();
  lsVTKWriter<NumericType>(extractor.getSecondMesh(), "second.vtk").apply();
}