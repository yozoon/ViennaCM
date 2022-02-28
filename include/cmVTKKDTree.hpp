#ifndef CM_VTK_KD_TREE_HPP
#define CM_VTK_KD_TREE_HPP

#include <limits>
#include <lsSmartPointer.hpp>

#include <vtkKdTree.h>
#include <vtkNew.h>
#include <vtkPoints.h>

template <class T, int D, class VectorType> class cmVTKKDTree {
private:
  lsSmartPointer<std::vector<VectorType>> pPoints;
  vtkNew<vtkPoints> points;
  vtkNew<vtkKdTree> tree;

public:
  cmVTKKDTree() {}

  cmVTKKDTree(lsSmartPointer<std::vector<VectorType>> passedPoints)
      : pPoints(passedPoints) {}

  void build() {
    lsMessage::getInstance()
        .addWarning(
            "cmVTKKdTree does not give the right distance for points "
            "outside of the initial geometries' bounding box at the moment!")
        .print();

    if (pPoints == nullptr) {
      lsMessage::getInstance().addWarning("No points provided!").print();
      return;
    }

    if (points->GetNumberOfPoints() == 0)
      points->Reset();

    for (const auto &pt : *pPoints) {
      points->InsertNextPoint(pt[0], pt[1], pt[2]);
    }

    if constexpr (D == 2)
      tree->OmitZPartitioning();

    tree->BuildLocatorFromPoints(points);
  }

  std::pair<VectorType, T> nearest(const VectorType &x) const {
    double dist;
    double tp[] = {x[0], x[1], x[2]};
    vtkIdType id = tree->FindClosestPoint(tp, dist);
    points->GetPoint(id, tp);

    return {VectorType{tp[0], tp[1], tp[2]}, dist};
  }
};

#endif