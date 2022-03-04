#ifndef CM_VTK_KD_TREE_HPP
#define CM_VTK_KD_TREE_HPP

#include <limits>
#include <lsSmartPointer.hpp>

#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>

#include "cmPointLocator.hpp"

template <class VectorType> class cmVTKKDTree : cmPointLocator<VectorType> {
  using typename cmPointLocator<VectorType>::SizeType;
  using typename cmPointLocator<VectorType>::T;
  using cmPointLocator<VectorType>::D;
  using typename cmPointLocator<VectorType>::DistanceFunctionType;

private:
  lsSmartPointer<std::vector<VectorType>> pPoints;
  vtkSmartPointer<vtkPoints> points;
  vtkSmartPointer<vtkKdTree> tree;

public:
  cmVTKKDTree() {}

  cmVTKKDTree(lsSmartPointer<std::vector<VectorType>> passedPoints)
      : pPoints(passedPoints) {}

  cmVTKKDTree(std::vector<VectorType> &passedPointRef)
      : points(lsSmartPointer<std::vector<VectorType>>::New(passedPointRef)) {}

  void build() override {
    lsMessage::getInstance()
        .addWarning(
            "cmVTKKdTree does not give the right distance for points "
            "outside of the initial geometries' bounding box at the moment! "
            "Also findKNearest is not yet properly implemented.")
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

  std::pair<SizeType, T> findNearest(const VectorType &x) const override {
    double dist;
    double tp[] = {x[0], x[1], x[2]};
    vtkIdType id = tree->FindClosestPoint(tp, dist);

    return {id, dist};
  }

  lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findKNearest(const VectorType &x, const int k) const override {
    auto result = lsSmartPointer<std::vector<std::pair<SizeType, T>>>::New();
    result->reserve(k);
    double tp[] = {x[0], x[1], x[2]};
    vtkSmartPointer<vtkIdList> ids;
    tree->FindClosestNPoints(k, tp, ids);

    // TODO: finish implementation!
    for (vtkIdType i = 0; i < k; i++) {
      vtkIdType id = ids->GetId(i);
      double p[3];
      points->GetPoint(id, p);
    }
    return result;
  }
};

#endif