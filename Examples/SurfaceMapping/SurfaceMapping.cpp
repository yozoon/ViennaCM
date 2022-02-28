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

  auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(baseMesh, "first.vtk").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsVTKReader<NumericType>(depoMesh, "second.vtk").apply();

  // Copy nodes to a new location
  auto baseNodes = baseMesh->nodes;

  using VectorType = typename decltype(baseNodes)::value_type;

  std::unordered_map<VectorType, size_t, cmVectorHash<VectorType>> lut;
  lut.reserve(baseNodes.size());

  size_t i = 0;
  std::vector<double> nodeIDs;
  nodeIDs.reserve(baseNodes.size());

  for (const auto &node : baseNodes) {
    lut.insert(std::pair{node, i});
    nodeIDs.push_back(i);
    i++;
  }

  auto points = lsSmartPointer<decltype(baseNodes)>::New(baseNodes);

  auto kdtree =
      lsSmartPointer<cmKDTree<NumericType, D, VectorType>>::New(points);
  kdtree->build();

  std::vector<hrleCoordType> nearestNodeIDs;
  nearestNodeIDs.reserve(depoMesh->nodes.size());

  const auto &nodes = depoMesh->nodes;
  for (size_t i = 0; i < depoMesh->nodes.size(); ++i) {
    auto nearest = kdtree->nearest(nodes[i]);
    auto id = lut[nearest.first];

    nearestNodeIDs.push_back(lut[nearest.first]);
  }

  baseMesh->getPointData().insertNextScalarData(nodeIDs, "ID");
  depoMesh->getPointData().insertNextScalarData(nearestNodeIDs, "nearestNode");

  lsVTKWriter<NumericType>(depoMesh, "mapped_mesh.vtp").apply();
  lsVTKWriter<NumericType>(baseMesh, "annotated_mesh.vtp").apply();
}