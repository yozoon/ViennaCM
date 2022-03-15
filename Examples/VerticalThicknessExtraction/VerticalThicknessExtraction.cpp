#include <algorithm>
#include <array>
#include <vector>

#include <lsDomain.hpp>
#include <lsMesh.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>
#include <lsToSurfaceMesh.hpp>
#include <lsVTKWriter.hpp>

#include "cmExtractMinThicknessAlongAxis.hpp"

int main() {
  static constexpr int D = 2;
  using NumericType = double;

  auto substrate = lsSmartPointer<lsDomain<NumericType, D>>::New();

  lsReader<NumericType, D>(substrate, "second.lvst").apply();

  auto mesh = lsSmartPointer<lsMesh<>>::New();
  lsToDiskMesh<NumericType, D>(substrate, mesh).apply();

  auto points = lsSmartPointer<decltype(mesh->nodes)>::New(mesh->nodes);
  auto data = lsSmartPointer<std::vector<NumericType>>::New();

  std::array<NumericType, 2> bounds = {0., -45.};

  static constexpr int axis = D - 1;
  cmExtractMinThicknessAlongAxis<NumericType, D>(
      points, data, axis, bounds, substrate->getGrid().getGridDelta())
      .apply();

  std::for_each(data->cbegin(), data->cend(),
                [](const NumericType &n) { std::cout << n << std::endl; });
}
