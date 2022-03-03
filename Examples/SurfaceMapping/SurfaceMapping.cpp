#include <vector>

#include <lsFromSurfaceMesh.hpp>
#include <lsMesh.hpp>
#include <lsVTKReader.hpp>
#include <lsVTKWriter.hpp>

#include "cmKDTree.hpp"
#include "cmVectorHash.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  // Load both meshes
  auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(baseMesh, "first.vtk").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(depoMesh, "second.vtk").apply();

  using VectorType = typename decltype(baseMesh->nodes)::value_type;

  const auto &baseNodes = baseMesh->nodes;

  size_t i = 0;
  std::vector<double> nodeIDs;
  nodeIDs.reserve(baseNodes.size());

  for (const auto &node : baseNodes) {
    nodeIDs.push_back(i);
    i++;
  }

  auto points = lsSmartPointer<decltype(baseMesh->nodes)>::New(baseMesh->nodes);

  auto kdtree =
      lsSmartPointer<cmKDTree<decltype(baseMesh->nodes)::value_type>>::New(
          points);
  kdtree->build();

  std::vector<hrleCoordType> nearestNodeIDs;
  nearestNodeIDs.reserve(depoMesh->nodes.size());

  const auto &nodes = depoMesh->nodes;
  for (size_t i = 0; i < depoMesh->nodes.size(); ++i) {
    auto nearest = kdtree->findNearest(nodes[i]);

    nearestNodeIDs.push_back(nearest.first);
  }

  baseMesh->getPointData().insertNextScalarData(nodeIDs, "ID");
  depoMesh->getPointData().insertNextScalarData(nearestNodeIDs, "nearestNode");

  lsVTKWriter<NumericType>(depoMesh, "mapped_mesh.vtk").apply();
  lsVTKWriter<NumericType>(baseMesh, "annotated_mesh.vtk").apply();
}