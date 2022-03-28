#ifndef CM_EXTRACT_CLOSEST_POINT_THICKNESS_HPP
#define CM_EXTRACT_CLOSEST_POINT_THICKNESS_HPP

#include <limits>
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
  using NumericType = typename cmPointLocator<VectorType>::T;
  typedef lsSmartPointer<lsMesh<NumericType>> MeshPtr;

  static constexpr int D = cmPointLocator<VectorType>::D;

  MeshPtr baseMesh = nullptr;
  MeshPtr secondMesh = nullptr;

public:
  cmExtractClosestPointThickness(MeshPtr passedBaseMesh,
                                 MeshPtr passedSecondMesh)
      : baseMesh(passedBaseMesh), secondMesh(passedSecondMesh) {}

  MeshPtr getBaseMesh() const { return baseMesh; }

  MeshPtr getSecondMesh() const { return secondMesh; }

  void apply() {
    if (baseMesh == nullptr || secondMesh == nullptr) {
      lsMessage::getInstance()
          .addWarning("The disk meshes provided to "
                      "cmExtractClosestPointThickness mustn't be null.")
          .print();
      return;
    }

    auto kdtree = lsSmartPointer<LocatorType>::New(secondMesh->nodes);
    kdtree->build();

    const auto &baseNodes = baseMesh->nodes;
    std::vector<NumericType> layerThickness;
    layerThickness.reserve(baseNodes.size());

    for (auto &node : baseNodes) {
      auto nearest = kdtree->findNearest(node);
      layerThickness.push_back(nearest.second);
    }

    auto &cellData = baseMesh->getCellData();
    auto index = cellData.getScalarDataIndex("layerThickness");
    if (index >= 0)
      cellData.eraseScalarData(index);

    cellData.insertNextScalarData(layerThickness, "layerThickness");
  }
};

#endif