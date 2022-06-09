
#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>

#include "cmGeometryToGraph.hpp"
#include "cmGraph.hpp"

int main() {
  using NumericType = double;
  static constexpr int D = 2;

  auto dom = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(dom, "first.lvst").apply();

  auto mesh = lsSmartPointer<lsMesh<>>::New();

  lsToDiskMesh<NumericType, D>(dom, mesh).apply();

  auto graph = lsSmartPointer<cmGraph>::New();

  cmGeometryToGraph<NumericType, D>(mesh, graph, dom->getGrid().getGridDelta())
      .apply();
}
