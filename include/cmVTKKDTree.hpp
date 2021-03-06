#ifndef CM_VTK_KD_TREE_HPP
#define CM_VTK_KD_TREE_HPP

#include <limits>
#include <lsSmartPointer.hpp>

#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>

#include "cmInternal.hpp"
#include "cmPointLocator.hpp"

template <class NumericType, int D, int Dim = D>
class cmVTKKDTree : cmPointLocator<NumericType, D, Dim> {
  using typename cmPointLocator<NumericType, D, Dim>::SizeType;
  using typename cmPointLocator<NumericType, D, Dim>::VectorType;

private:
  vtkSmartPointer<vtkPoints> points = nullptr;
  vtkSmartPointer<vtkKdTree> tree = nullptr;

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
#ifndef NDEBUG
    lsMessage::getInstance()
        .addWarning(
            "cmVTKKdTree does not give the right distance for points "
            "outside of the initial geometries' bounding box at the moment!")
        .print();
#endif

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

  std::pair<SizeType, NumericType>
  findNearest(const VectorType &x) const override {
    double dist;
    double tp[] = {x[0], x[1], 0.};
    if constexpr (D == 3)
      tp[2] = x[2];
    vtkIdType id = tree->FindClosestPoint(tp, dist);
    return {id, dist};
  }

  lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>
  findKNearest(const VectorType &x, const int k) const override {
    auto result =
        lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>::New();
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
      result->emplace_back(std::pair{id, cmInternal::euclideanDistance(x, v)});
    }
    return result;
  }

  lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>
  findNearestWithinRadius(const VectorType &x,
                          const NumericType radius) const override {
    auto result =
        lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>::New();
    double tp[] = {x[0], x[1], 0.};
    if constexpr (D == 3)
      tp[2] = x[2];

    vtkNew<vtkIdList> ids;
    tree->FindPointsWithinRadius(radius, tp, ids);

    for (vtkIdType i = 0; i < ids->GetNumberOfIds(); ++i) {
      vtkIdType id = ids->GetId(i);
      double p[3];
      points->GetPoint(id, p);
      VectorType v;
      v[0] = p[0];
      v[1] = p[1];
      if constexpr (D == 3)
        v[2] = p[2];
      result->emplace_back(std::pair{id, cmInternal::euclideanDistance(x, v)});
    }
    return result;
  }
};

#endif