#ifndef CM_SURFACE_MAPPER_HPP
#define CM_SURFACE_MAPPER_HPP

//#include <vtkKdTreePointLocator.h>
//#include <vtkOctreePointLocator.h>
#include <vtkKdTree.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <lsMesh.hpp>
#include <lsMessage.hpp>
#include <lsSmartPointer.hpp>

#include "cmKDTree.hpp"

// Finds the closest point on the base surface for each node on the advected
// surface. The corresponding node index is stored in the advected surface's
// scalar data.
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

    auto kdtree = lsSmartPointer<cmKDTree<T,D>>::New(baseSurface->nodes);

    kdtree->build();

    /*
    // Derived from the VTK KdTreePointLocatorClosestPoint example.
    vtkNew<vtkPoints> points;
    std::vector<double> IDs;

    for (int i = 0; i < baseSurface->nodes.size(); ++i) {
      const auto &node = baseSurface->nodes[i];
      points->InsertNextPoint(node[0], node[1], node[2]);
      IDs.push_back(1.0 * i);
    }

    baseSurface->getCellData().insertNextScalarData(IDs, "Origin");

    std::cout << points->GetNumberOfPoints() << std::endl;

    vtkNew<vtkKdTree> tree;
    if constexpr (D == 2) {
      tree->OmitZPartitioning();
    }
    tree->SetMinCells(10);
    tree->BuildLocatorFromPoints(points);

    double bounds[6];
    tree->GetBounds(bounds);

    std::cout << "x: (" << bounds[0] << ", " << bounds[1] << ")\n"
              << "y: (" << bounds[2] << ", " << bounds[3] << ")\n"
              << "z: (" << bounds[4] << ", " << bounds[5] << ")\n";

    T bmin = std::min({bounds[0], bounds[2], bounds[4]});
    T bmax = std::max({bounds[1], bounds[3], bounds[5]});
    T radius = std::sqrt(bmin * bmin + bmax * bmax);

    std::vector<double> closestIDs(advectedSurface->nodes.size(), -1);

    std::cout << advectedSurface->nodes.size() << ", "
              << baseSurface->nodes.size() << std::endl;

    const int numRegions = tree->GetNumberOfRegions();

    // for (int i = 0; i < numRegions; i++) {
    //
    // }

    tree->PrintTree();

    //#pragma omp parallel for
    for (int i = 0; i < advectedSurface->nodes.size(); ++i) {
      const auto &node = advectedSurface->nodes[i];
      double dist;
      T tp[] = {node[0], node[1], node[2]};
      vtkIdType iD = tree->FindClosestPoint(tp, dist);
      // vtkIdType iD = tree->FindClosestPointWithinRadius(2., tp, dist);
      const auto &baseNode = baseSurface->nodes[i];

      // std::cout << "(" << node[0] << ", " << node[1] << ", " << node[2]
      //           << ") -> (" << baseNode[0] << ", " << baseNode[1] << ", "
      //           << baseNode[2] << "): " << dist << "\n";
      // if (dist != 25)
      closestIDs[i] = static_cast<double>(iD);
    }

    advectedSurface->getCellData().insertNextScalarData(closestIDs, "Origin");
    */
  }
};

#endif