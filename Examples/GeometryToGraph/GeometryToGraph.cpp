#include <memory>

#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>
#include <lsVTKWriter.hpp>

#include <cmGraphData.hpp>
#include <cmGraphReader.hpp>
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

  auto sampler =
      std::make_unique<cmUniformRaySampler<NumericType, D>>(numRaysPerPoint);
  // cmCosineDistributionRaySampler<NumericType, D> sampler(numRaysPerPoint);
  cmRayTraceGraph<NumericType, D, GraphNumericType> tracer;

  tracer.setSourceDirection(D == 2 ? rayTraceDirection::POS_Y
                                   : rayTraceDirection::POS_Z);
  tracer.setBoundaryConditions(rtBC);

  auto builder =
      std::make_unique<GraphBuilder<NumericType, GraphNumericType>>();

  tracer.setGraphBuilderType(builder);
  tracer.setGraphSamplerType(sampler);

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

  auto graphDataPtr = lsSmartPointer<decltype(graphData)>::New(graphData);

  /** Writing **/

  cmGraphWriter<GraphNumericType>(graphDataPtr, "graph.vtp").apply();

  cmGraphWriter<GraphNumericType>(graphDataPtr, "graph").apply();
#ifdef WITH_MSGPACK
  cmGraphWriter<GraphNumericType>(graphDataPtr, "graph.msgpack").apply();
#ifdef WITH_GZIP
  cmGraphWriter<GraphNumericType>(graphDataPtr, "graph.msgpack.gz").apply();
#endif
#endif

  /** Reading **/

#ifdef WITH_MSGPACK
  auto gdPtr1 = lsSmartPointer<decltype(graphData)>::New();
  cmGraphReader<GraphNumericType>(gdPtr1, "graph.msgpack").apply();
#ifdef WITH_GZIP
  auto gdPtr2 = lsSmartPointer<decltype(graphData)>::New();
  cmGraphReader<GraphNumericType>(gdPtr2, "graph.msgpack.gz").apply();
#endif
#endif
}
