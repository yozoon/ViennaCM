#ifndef CM_EXTRACT_CLOSEST_POINT_THICKNESS_HPP
#define CM_EXTRACT_CLOSEST_POINT_THICKNESS_HPP

#include <limits>
#include <type_traits>
#include <vector>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>

#include <cmKDTree.hpp>
#include <cmPointLocator.hpp>

template <class NumericType, int D,
          class LocatorType = cmKDTree<NumericType, D, 3>>
class cmExtractClosestPointThickness {
  static_assert(
      std::is_base_of<cmPointLocator<NumericType, D, 3>, LocatorType>::value,
      "The passed point locator is not a subclass of cmPointLocator.");

  using SizeType = typename cmPointLocator<NumericType, D, 3>::SizeType;
  typedef lsSmartPointer<lsMesh<>> MeshPtr;

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