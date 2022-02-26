#ifndef CM_SURFACE_MAPPER_HPP
#define CM_SURFACE_MAPPER_HPP

#include <unordered_map>

#include <lsMesh.hpp>
#include <lsMessage.hpp>
#include <lsSmartPointer.hpp>

#include "cmKDTree.hpp"
#include "cmVectorHash.hpp"

template <class T, int D> class cmSurfaceMapper {
  lsSmartPointer<lsMesh<>> baseSurface = nullptr;
  lsSmartPointer<lsMesh<>> advectedSurface = nullptr;

public:
  cmSurfaceMapper() {}

  cmSurfaceMapper(lsSmartPointer<lsMesh<>> passedBaseSurface,
                  lsSmartPointer<lsMesh<>> passedAdvectedSurface)
      : baseSurface(passedBaseSurface), advectedSurface(passedAdvectedSurface) {
  }

  void apply() {
    if (baseSurface == nullptr || advectedSurface == nullptr) {
      lsMessage::getInstance()
          .addWarning(
              "The surface meshes provided to cmSurfaceMapper mustn't be null.")
          .print();
      return;
    }

    // Copy nodes to a new location
    auto baseNodes = baseSurface->nodes;

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

    auto kdtree = lsSmartPointer<cmKDTree<T, D, VectorType>>::New(points);

    kdtree->build();

    std::vector<hrleCoordType> nearestNodeIDs;
    nearestNodeIDs.reserve(advectedSurface->nodes.size());

    const auto &nodes = advectedSurface->nodes;
    for (size_t i = 0; i < advectedSurface->nodes.size(); ++i) {
      auto nearest = kdtree->nearest(nodes[i]);
      auto id = lut[nearest.first];

      nearestNodeIDs.push_back(lut[nearest.first]);
    }

    baseSurface->getPointData().insertNextScalarData(nodeIDs, "ID");
    advectedSurface->getPointData().insertNextScalarData(nearestNodeIDs,
                                                         "nearestNode");
  }
};

#endif