#ifndef CM_EXTRACT_MIN_THICKNESS_KDTREE_HPP
#define CM_EXTRACT_MIN_THICKNESS_KDTREE_HPP

#include <vector>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>

#include "cmKDTree.hpp"

template <typename T, int D> class cmExtractMinThicknessKDTree {
  lsSmartPointer<lsMesh<>> referenceMesh = nullptr;
  lsSmartPointer<lsMesh<>> advectedMesh = nullptr;
  lsSmartPointer<std::vector<T>> dataDestination = nullptr;

public:
  cmExtractMinThicknessKDTree() {}

  cmExtractMinThicknessKDTree(
      lsSmartPointer<lsMesh<>> passedReferenceMesh,
      lsSmartPointer<lsMesh<>> passedAdvectedMesh,
      lsSmartPointer<std::vector<T>> passedDataDestination)
      : referenceMesh(passedReferenceMesh), advectedMesh(passedAdvectedMesh),
        dataDestination(passedDataDestination) {}

  void apply() {
    if (referenceMesh == nullptr || advectedMesh == nullptr) {
      lsMessage::getInstance()
          .addWarning("The surface meshes provided to "
                      "cmExtractMinThicknessKDTree mustn't be null.")
          .print();
      return;
    }

    if (dataDestination == nullptr) {
      lsMessage::getInstance()
          .addWarning("No destination vector provided.")
          .print();
      return;
    }

    if (dataDestination->size() != 0)
      dataDestination->clear();

    // Copy nodes to a new location
    auto advNodes = advectedMesh->nodes;

    auto points = lsSmartPointer<decltype(advNodes)>::New(advNodes);

    using VectorType = typename decltype(advNodes)::value_type;

    auto kdtree = lsSmartPointer<cmKDTree<T, D, VectorType>>::New(points);
    kdtree->build();

    const auto &nodes = referenceMesh->nodes;
    dataDestination->reserve(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
      auto nearest = kdtree->findNearest(nodes[i]);
      dataDestination->push_back(nearest.second);
    }
  }
};

#endif