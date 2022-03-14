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
  vtkSmartPointer<vtkPoints> points = nullptr;
  vtkSmartPointer<vtkKdTree> tree = nullptr;

  static T distance(const VectorType &a, const VectorType &b) {
    T sum{0};
    for (int i = 0; i < D; ++i) {
      T d = b[i] - a[i];
      sum += d * d;
    }
    return std::sqrt(sum);
  }

public:
  cmVTKKDTree() {}

  cmVTKKDTree(std::vector<VectorType> &passedPoints) {
    points = vtkNew<vtkPoints>();
    for (const auto pt : passedPoints) {
      double tp[3] = {pt[0], pt[1], 0.};
      if constexpr (D == 3)
        tp[2] = pt[2];
      points->InsertNextPoint(tp);
    }
  }

  void build() override {
    lsMessage::getInstance()
        .addWarning(
            "cmVTKKdTree does not give the right distance for points "
            "outside of the initial geometries' bounding box at the moment!")
        .print();

    if (points->GetNumberOfPoints() == 0) {
      lsMessage::getInstance()
          .addWarning("No points provided to cmVTKKdTree!")
          .print();
      return;
    }

    if (tree != nullptr) {
      lsMessage::getInstance()
          .addWarning("Tree has already been built!")
          .print();
      return;
    }

    tree = vtkNew<vtkKdTree>();

    if constexpr (D == 2)
      tree->OmitZPartitioning();

    tree->BuildLocatorFromPoints(points);
  }

  std::pair<SizeType, T> findNearest(const VectorType &x) const override {
    double dist;
    double tp[] = {x[0], x[1], 0.};
    if constexpr (D == 3)
      tp[2] = x[2];
    vtkIdType id = tree->FindClosestPoint(tp, dist);
    return {id, dist};
  }

  lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findKNearest(const VectorType &x, const int k) const override {
    auto result = lsSmartPointer<std::vector<std::pair<SizeType, T>>>::New();
    result->reserve(k);
    double tp[] = {x[0], x[1], 0.};
    if constexpr (D == 3)
      tp[2] = x[2];

    vtkNew<vtkIdList> ids;
    tree->FindClosestNPoints(k, tp, ids);

    for (vtkIdType i = 0; i < k; ++i) {
      vtkIdType id = ids->GetId(i);
      double p[3];
      points->GetPoint(id, p);
      VectorType v;
      v[0] = p[0];
      v[1] = p[1];
      if constexpr (D == 3)
        v[2] = p[2];
      result->emplace_back(std::pair{id, distance(x, v)});
    }
    return result;
  }
};

#endif