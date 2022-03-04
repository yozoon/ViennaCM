#ifndef CM_MESH_SIMILARITY_SCORES_HPP
#define CM_MESH_SIMILARITY_SCORES_HPP

#include <algorithm>
#include <array>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>

#include "cmKDTree.hpp"
#include "cmPointLocator.hpp"

template <class T, int D> struct cmMeshSimilarityScore {
  cmMeshSimilarityScore() {}
  cmMeshSimilarityScore(lsSmartPointer<lsMesh<>>) {}
  virtual void setMesh(lsSmartPointer<lsMesh<>>) = 0;
  virtual T calculate(lsSmartPointer<lsMesh<>>) = 0;
};

template <class T, int D, class LocatorType = cmKDTree<std::array<T, 3>>>
class cmChamferDistanceScore : cmMeshSimilarityScore<T, D> {
  static_assert(
      std::is_base_of<cmPointLocator<std::array<T, 3>>, LocatorType>::value,
      "The passed point locator is not a subclass of cmPointLocator.");

  lsSmartPointer<lsMesh<>> firstMesh = nullptr;
  lsSmartPointer<LocatorType> firstLocator = nullptr;
  bool buildLocator = true;

public:
  cmChamferDistanceScore() {}
  cmChamferDistanceScore(lsSmartPointer<lsMesh<>> passedMesh)
      : firstMesh(passedMesh) {}

  void setMesh(lsSmartPointer<lsMesh<>> passedMesh) override {
    firstMesh = passedMesh;
    buildLocator = true;
  }

  T calculate(lsSmartPointer<lsMesh<>> secondMesh) override {
    if (firstMesh == nullptr || secondMesh == nullptr) {
      lsMessage::getInstance()
          .addWarning(
              "One of the meshes provided to cmChamferDistance is null.")
          .print();
      return std::numeric_limits<T>::infinity();
    }
    // Only build the first locator if it hasn't been build yet
    if (buildLocator) {
      buildLocator = false;
      firstLocator = lsSmartPointer<LocatorType>::New(firstMesh->nodes);
      firstLocator->build();
    }

    auto secondLocator = lsSmartPointer<LocatorType>::New(secondMesh->nodes);
    secondLocator->build();

    T sum = 0.;

#pragma omp parallel for default(shared) reduction(+ : sum)
    for (const auto &node : secondMesh->nodes) {
      sum += firstLocator->findNearest(node).second;
    }

#pragma omp parallel for default(shared) reduction(+ : sum)
    for (const auto &node : firstMesh->nodes) {
      sum += secondLocator->findNearest(node).second;
    }

    return sum;
  }
};
#endif