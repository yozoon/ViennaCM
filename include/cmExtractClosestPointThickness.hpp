#ifndef CM_EXTRACT_CLOSEST_POINT_THICKNESS_HPP
#define CM_EXTRACT_CLOSEST_POINT_THICKNESS_HPP

#include <type_traits>
#include <vector>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>

#include "cmPointLocator.hpp"

#include "cmKDTree.hpp"

template <class VectorType, class LocatorType = cmKDTree<VectorType>>
class cmExtractClosestPointThickness {
  static_assert(
      std::is_base_of<cmPointLocator<VectorType>, LocatorType>::value,
      "The passed point locator is not a subclass of cmPointLocator.");

  using SizeType = typename cmPointLocator<VectorType>::SizeType;
  using T = typename cmPointLocator<VectorType>::T;
  static constexpr int D = cmPointLocator<VectorType>::D;

  lsSmartPointer<lsMesh<>> referenceMesh = nullptr;
  lsSmartPointer<lsMesh<>> advectedMesh = nullptr;
  lsSmartPointer<std::vector<T>> dataDestination = nullptr;

public:
  cmExtractClosestPointThickness() {}

  cmExtractClosestPointThickness(
      lsSmartPointer<lsMesh<>> passedReferenceMesh,
      lsSmartPointer<lsMesh<>> passedAdvectedMesh,
      lsSmartPointer<std::vector<T>> passedDataDestination)
      : referenceMesh(passedReferenceMesh), advectedMesh(passedAdvectedMesh),
        dataDestination(passedDataDestination) {}

  void apply() {
    if (referenceMesh == nullptr || advectedMesh == nullptr) {
      lsMessage::getInstance()
          .addWarning("The surface meshes provided to "
                      "cmExtractClosestPointThickness mustn't be null.")
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

    auto points =
        lsSmartPointer<decltype(advectedMesh->nodes)>::New(advectedMesh->nodes);

    auto kdtree = lsSmartPointer<LocatorType>::New(points);

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