#include <vector>

#include <lsFromSurfaceMesh.hpp>
#include <lsMesh.hpp>
#include <lsVTKReader.hpp>
#include <lsVTKWriter.hpp>

#include "cmKDTree.hpp"

int main() {
  static constexpr int D = 2;
  using NumericType = double;

  // Load both meshes
  auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(baseMesh, "first.vtk").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(depoMesh, "second.vtk").apply();

  using VectorType = decltype(baseMesh->nodes)::value_type;

  size_t i = 0;
  std::vector<double> nodeIDs;
  nodeIDs.reserve(baseMesh->nodes.size());
  for (const auto &node : baseMesh->nodes) {
    nodeIDs.push_back(i);
    ++i;
  }

  baseMesh->getPointData().insertNextScalarData(nodeIDs, "ID");
  lsVTKWriter<NumericType>(baseMesh, "annotated_mesh.vtk").apply();

  auto kdtree = lsSmartPointer<cmKDTree<VectorType>>::New(baseMesh->nodes);
  kdtree->build();

  std::vector<hrleCoordType> nearestNodeIDs;
  nearestNodeIDs.reserve(depoMesh->nodes.size());

  const auto &nodes = depoMesh->nodes;
  for (size_t i = 0; i < nodes.size(); ++i) {
    auto nearest = kdtree->findNearest(nodes[i]);
    nearestNodeIDs.push_back(nearest.first);
  }

  depoMesh->getPointData().insertNextScalarData(nearestNodeIDs, "nearestNode");
  lsVTKWriter<NumericType>(depoMesh, "mapped_mesh.vtk").apply();
}