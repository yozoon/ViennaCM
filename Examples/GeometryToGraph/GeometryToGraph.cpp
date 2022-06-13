
#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>

#include <cmGraphBuilder.hpp>
#include <cmGraphData.hpp>
#include <cmRayTraceGraph.hpp>

int main() {
  using NumericType = double;
  using GraphNumericType = float;

  static constexpr int D = 2;
  static constexpr int numRaysPerPoint = 11;

  auto dom = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(dom, "first.lvst").apply();

  auto diskMesh = lsSmartPointer<lsMesh<>>::New();

  lsToDiskMesh<NumericType, D>(dom, diskMesh).apply();

  auto points = diskMesh->getNodes();
  auto normals = *diskMesh->getCellData().getVectorData("Normals");
  auto materialIds = *diskMesh->getCellData().getScalarData("MaterialIds");

  // ray tracing setup
  rayTraceBoundary rtBC[D];
  for (unsigned i = 0; i < D; ++i)
    rtBC[i] = rayTraceBoundary::REFLECTIVE;

  cmRandomRaySampler<NumericType, D> sampler(numRaysPerPoint);
  cmRayTraceGraph<NumericType, D, GraphNumericType> tracer(sampler);

  tracer.setSourceDirection(D == 2 ? rayTraceDirection::POS_Y
                                   : rayTraceDirection::POS_Z);
  tracer.setBoundaryConditions(rtBC);

  auto builder = std::make_unique<
      cmGeometricGraphBuilder<NumericType, GraphNumericType>>();

  tracer.setGraphBuilderType(builder);

  tracer.apply();

  auto graphData = tracer.getLocalGraphData();

  std::cout << graphData.getEdges().size() << std::endl;
}
