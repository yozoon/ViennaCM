
#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>
#include <lsVTKWriter.hpp>

#include <cmGraphData.hpp>
#include <cmGraphWriter.hpp>
#include <cmRayTraceGraph.hpp>

#include "GraphBuilder.hpp"

int main() {
  using NumericType = double;
  using GraphNumericType = float;

  static constexpr int D = 2;
  static constexpr int numRaysPerPoint = 6;

  auto dom = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(dom, "first.lvst").apply();

  auto diskMesh = lsSmartPointer<lsMesh<>>::New();

  lsToDiskMesh<NumericType, D>(dom, diskMesh).apply();

  lsVTKWriter<NumericType>(diskMesh, "mesh.vtp").apply();

  // ray tracing setup
  rayTraceBoundary rtBC[D];
  for (unsigned i = 0; i < D; ++i)
    rtBC[i] = rayTraceBoundary::REFLECTIVE;

  cmUniformRaySampler<NumericType, D> sampler(numRaysPerPoint);
  //cmCosineDistributionRaySampler<NumericType, D> sampler(numRaysPerPoint);
  cmRayTraceGraph<NumericType, D, GraphNumericType> tracer(sampler);

  tracer.setSourceDirection(D == 2 ? rayTraceDirection::POS_Y
                                   : rayTraceDirection::POS_Z);
  tracer.setBoundaryConditions(rtBC);

  auto builder =
      std::make_unique<GraphBuilder<NumericType, GraphNumericType>>();

  tracer.setGraphBuilderType(builder);

  auto points = diskMesh->getNodes();
  auto normals = *diskMesh->getCellData().getVectorData("Normals");
  auto materialIds = *diskMesh->getCellData().getScalarData("MaterialIds");

  tracer.setGeometry(points, normals, dom->getGrid().getGridDelta());
  tracer.setMaterialIds(materialIds);

  tracer.apply();

  auto graphData = tracer.getLocalGraphData();

  // Indicate that the last node is the source node
  graphData.getNodeData(0).back() = 1.;

  std::cout << "Number of nodes: " << graphData.getNodes().size()
            << "\nNumber of edges: " << graphData.getEdges().size() / 2
            << std::endl;

  cmGraphWriter<GraphNumericType>(
      lsSmartPointer<decltype(graphData)>::New(graphData), "graph.vtp")
      .apply();

  cmGraphWriter<GraphNumericType>(
      lsSmartPointer<decltype(graphData)>::New(graphData), "graph")
      .apply();
}
