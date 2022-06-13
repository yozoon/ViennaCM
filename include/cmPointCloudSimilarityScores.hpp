#ifndef CM_POINT_CLOUD_SIMILARITY_SCORES_HPP
#define CM_POINT_CLOUD_SIMILARITY_SCORES_HPP

#include <array>
#ifdef _OPENMP
#include <omp.h>
#endif

#include <lsSmartPointer.hpp>

#include <cmKDTree.hpp>
#include <cmPointLocator.hpp>

template <class T, int D, class VectorType = std::array<T, 3>>
struct cmPointCloudSimilarityScore {
  cmPointCloudSimilarityScore() {}

  virtual void setFirstPointCloud(lsSmartPointer<std::vector<VectorType>>) = 0;
  virtual T calculate(lsSmartPointer<std::vector<VectorType>>) = 0;
};

/**
 * Based on the chamfer distance score introduced in "A Point Set Generation
 * Network for 3D Object Reconstruction from a Single Image" by H. Fan et. al.
 * (https://arxiv.org/pdf/1612.00603.pdf)
 */
template <class T, int D, class VectorType = std::array<T, 3>,
          class LocatorType = cmKDTree<VectorType>>
class cmChamferDistanceScore : cmPointCloudSimilarityScore<T, D, VectorType> {
  static_assert(
      std::is_base_of<cmPointLocator<VectorType>, LocatorType>::value,
      "The passed point locator is not a subclass of cmPointLocator.");

  lsSmartPointer<std::vector<VectorType>> firstPointCloud = nullptr;
  lsSmartPointer<LocatorType> firstLocator = nullptr;
  bool buildLocator = true;

public:
  cmChamferDistanceScore() {}
  cmChamferDistanceScore(
      lsSmartPointer<std::vector<VectorType>> passedFirstPointCloud)
      : firstPointCloud(passedFirstPointCloud) {}

  void setFirstPointCloud(
      lsSmartPointer<std::vector<VectorType>> passedFirstPointCloud) override {
    firstPointCloud = passedFirstPointCloud;
    buildLocator = true;
  }

  T calculate(
      lsSmartPointer<std::vector<VectorType>> secondPointCloud) override {
    if (firstPointCloud == nullptr || secondPointCloud == nullptr) {
      lsMessage::getInstance()
          .addWarning(
              "One of the meshes provided to cmChamferDistance is null.")
          .print();
      return std::numeric_limits<T>::infinity();
    }

    // Only build the first locator if it hasn't been build yet
    if (buildLocator) {
      buildLocator = false;
      firstLocator = lsSmartPointer<LocatorType>::New(*firstPointCloud);
      firstLocator->build();
    }

    auto secondLocator = lsSmartPointer<LocatorType>::New(*secondPointCloud);
    secondLocator->build();

    T sum = 0.;

#pragma omp parallel for default(shared) reduction(+ : sum)
    for (const auto &node : *secondPointCloud) {
      sum += firstLocator->findNearest(node).second;
    }

#pragma omp parallel for default(shared) reduction(+ : sum)
    for (const auto &node : *firstPointCloud) {
      sum += secondLocator->findNearest(node).second;
    }

    return sum;
  }
};
#endif