#ifndef CM_SURFACE_MAPPER_HPP
#define CM_SURFACE_MAPPER_HPP

#include <vtkKdTreePointLocator.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <lsMesh.hpp>
#include <lsMessage.hpp>
#include <lsSmartPointer.hpp>

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

    // Derived from the VTK KdTreePointLocatorClosestPoint example.
    vtkNew<vtkPoints> points;
    for (const auto &node : baseSurface->nodes) {
      T tmp[] = {node[0], node[1], node[2]};
      points->InsertNextPoint(tmp);
    }

    vtkNew<vtkPolyData> polydata;
    polydata->SetPoints(points);

    vtkNew<vtkKdTreePointLocator> kDTree;
    kDTree->SetDataSet(polydata);
    kDTree->BuildLocator();

    std::vector<double> closestIDs(advectedSurface->nodes.size(), -1);

    for (int i = 0; i < advectedSurface->nodes.size(); ++i) {
      const auto &node = advectedSurface->nodes[i];
      T tp[] = {node[0], node[1], node[2]};
      double dist;
      vtkIdType iD = kDTree->FindClosestPointWithinRadius(10., tp, dist);
      std::cout << static_cast<std::size_t>(iD) << "\n";
      closestIDs[i] = static_cast<double>(iD);
    }

    advectedSurface->getCellData().insertNextScalarData(closestIDs, "Origin");
  }
};

#endif