#ifndef CM_SURFACE_MAPPER_HPP
#define CM_SURFACE_MAPPER_HPP

#include <lsMesh.hpp>
#include <lsMessage.hpp>
#include <lsSmartPointer.hpp>

#include "cmKDTree.hpp"

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

    using CoordType = typename decltype(baseSurface->nodes)::value_type;

    // Copy nodes to a new location
    auto nodes = baseSurface->nodes;
    auto points = lsSmartPointer<decltype(nodes)>::New(nodes);

    auto kdtree = lsSmartPointer<cmKDTree<T, D, CoordType>>::New(points);

    kdtree->build();

    std::vector<double> layerThickness(advectedSurface->nodes.size(), -1);

    for (int i = 0; i < advectedSurface->nodes.size(); ++i) {
      const auto &node = advectedSurface->nodes[i];
      auto nearest = kdtree->nearest(node);
      layerThickness[i] = nearest.second;
    }

    advectedSurface->getCellData().insertNextScalarData(layerThickness,
                                                        "thickness");
  }
};

#endif