#ifndef CM_KDTREE_HPP
#define CM_KDTREE_HPP

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <lsSmartPointer.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "cmBoundedPQueue.hpp"

int intLog2(int x) {
  int val = 0;
  while (x >>= 1)
    ++val;
  return val;
}

enum struct cmKDTreeDistanceEnum : unsigned { EUCLIDEAN = 0, CUSTOM = 1 };

template <class T, int D, class VectorType> class cmKDTree {
public:
  using SizeType = std::size_t;
  using DistanceFunctionType = T (*)(const VectorType &, const VectorType &);

private:
  SizeType N;
  SizeType treeSize = 0;

  int numThreads = 1;
  int maxParallelDepth = 0;
  int surplusWorkers = 0;

  lsSmartPointer<std::vector<VectorType>> points = nullptr;

  cmKDTreeDistanceEnum distanceType = cmKDTreeDistanceEnum::EUCLIDEAN;
  DistanceFunctionType customDistance = nullptr;
  DistanceFunctionType customReducedDistance = nullptr;

public:
  struct Node {
    VectorType &value;
    int axis;

    Node *left = nullptr;
    Node *right = nullptr;

    Node(VectorType &passedValue, int passedAxis)
        : value(passedValue), axis(passedAxis) {}
  };

private:
  Node *rootNode = nullptr;

  void build(Node *parent, typename std::vector<VectorType>::iterator start,
             typename std::vector<VectorType>::iterator end, int depth,
             bool isLeft) {
    SizeType size = end - start;

    int axis = depth % D;

    if (size > 1) {
      SizeType medianIndex = (size + 1) / 2 - 1;
      std::nth_element(
          start, start + medianIndex, end,
          [&](VectorType &a, VectorType &b) { return a[axis] < b[axis]; });

      Node *current = new Node(*(start + medianIndex), axis);

      if (isLeft)
        parent->left = current;
      else
        parent->right = current;

#ifdef _OPENMP
      bool dontSpawnMoreThreads = depth > maxParallelDepth + 1 ||
                                  (depth == maxParallelDepth + 1 &&
                                   omp_get_thread_num() >= surplusWorkers);
#endif
#pragma omp task final(dontSpawnMoreThreads)
      {
#ifdef _OPENMP
#ifndef NDEBUG
        std::cout << "Task assigned to thread " << omp_get_thread_num()
                  << std::endl;
#endif
#endif
        // Left Subtree
        build(current,             // Use current node as parent
              start,               // Data start
              start + medianIndex, // Data end
              depth + 1,           // Depth
              true                 // Left
        );
      }

      //  Right Subtree
      build(current,                 // Use current node as parent
            start + medianIndex + 1, // Data start
            end,                     // Data end
            depth + 1,               // Depth
            false                    // Right
      );
#pragma omp taskwait
    } else if (size == 1) {
      // Leaf Node
      if (isLeft)
        parent->left = new Node(*start, axis);
      else
        parent->right = new Node(*start, axis);
    }
  }

  void recursiveDelete(Node *node) {
    if (node != nullptr) {
      recursiveDelete(node->left);
      recursiveDelete(node->right);
      delete node;
    }
  }

  static T euclideanReducedDistance(const VectorType &a, const VectorType &b) {
    T sum{0};
    for (int i = 0; i < D; i++) {
      T d = b[i] - a[i];
      sum += d * d;
    }
    return sum;
  }

  static T euclideanDistance(const VectorType &a, const VectorType &b) {
    return std::sqrt(euclideanReducedDistance(a, b));
  }

  T distanceReducedInternal(const VectorType &a, const VectorType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::CUSTOM:
      return customReducedDistance(a, b);
      break;
    default:
      return euclideanReducedDistance(a, b);
    }
  }

  T distanceInternal(const VectorType &a, const VectorType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::CUSTOM:
      return customDistance(a, b);
    default:
      return euclideanDistance(a, b);
    }
  }

  void traverseDown(Node *currentNode, cmBoundedPQueue<T, Node *> &queue,
                    const VectorType &x) const {
    if (currentNode == nullptr)
      return;

    int axis = currentNode->axis;

    // For distance comparison operations we only use the "reduced" aka less
    // compute intensive, but order preserving version of the distance
    // function.
    queue.enqueue(
        std::pair{distanceReducedInternal(x, currentNode->value), currentNode});

    bool isLeft;
    if (x[axis] < currentNode->value[axis]) {
      traverseDown(currentNode->left, queue, x);
      isLeft = true;
    } else {
      traverseDown(currentNode->right, queue, x);
      isLeft = false;
    }

    // If the hypersphere with origin at x and a radius of our current best
    // distance intersects the hyperplane defined by the partitioning of the
    // current node, we also have to search the other subtree, since there could
    // be points closer to x than our current best.
    if (queue.size() < queue.maxSize() ||
        std::abs(x[axis] - currentNode->value[axis]) < queue.worst()) {
      if (isLeft)
        traverseDown(currentNode->right, queue, x);
      else
        traverseDown(currentNode->left, queue, x);
    }
    return;
  }

public:
  cmKDTree() {}

  cmKDTree(lsSmartPointer<std::vector<VectorType>> passedPoints)
      : N(passedPoints != nullptr ? passedPoints->size() : 0),
        points(passedPoints) {}

  void build() {
    if (points == nullptr) {
      lsMessage::getInstance().addWarning("No points provided!").print();
      return;
    }

    if (rootNode != nullptr)
      recursiveDelete(rootNode);

#pragma omp parallel default(none)                                             \
    shared(numThreads, maxParallelDepth, surplusWorkers, std::cout, rootNode,  \
           treeSize, points)
    {
#pragma omp single
      {
#ifdef _OPENMP
        numThreads = omp_get_num_threads();
#endif
        maxParallelDepth = intLog2(numThreads);
        surplusWorkers = numThreads - 1 << maxParallelDepth;
#ifndef NDEBUG
        std::cout << "Starting parallel region with " << numThreads
                  << " parallel workers." << std::endl;
#endif

        SizeType size = points->end() - points->begin();
        SizeType medianIndex = (size + 1) / 2 - 1;

        std::nth_element(
            points->begin(), points->begin() + medianIndex, points->end(),
            [&](VectorType &a, VectorType &b) { return a[0] < b[0]; });

        rootNode = new Node(*(points->begin() + medianIndex), 0);

#ifdef _OPENMP
        bool dontSpawnMoreThreads = 0 > maxParallelDepth + 1 ||
                                    (0 == maxParallelDepth + 1 &&
                                     omp_get_thread_num() >= surplusWorkers);
#endif
#pragma omp task final(dontSpawnMoreThreads)
        {
#ifdef _OPENMP
#ifndef NDEBUG
          std::cout << "Task assigned to thread " << omp_get_thread_num()
                    << std::endl;
#endif
#endif
          // Left Subtree
          build(rootNode,                      // Use rootNode as parent
                points->begin(),               // Data start
                points->begin() + medianIndex, // Data end
                1,                             // Depth
                true                           // Left
          );
        }

        // Right Subtree
        build(rootNode,                          // Use rootNode as parent
              points->begin() + medianIndex + 1, // Data start
              points->end(),                     // Data end
              1,                                 // Depth
              false                              // Right
        );
#pragma omp taskwait
      }
    }
  }

  std::pair<VectorType, T> findNearest(const VectorType &x) const {
    auto queue = cmBoundedPQueue<T, Node *>(1);
    traverseDown(rootNode, queue, x);
    auto best = queue.dequeueBest();
    return {best->value, distanceInternal(x, best->value)};
  }

  lsSmartPointer<std::vector<std::pair<VectorType, T>>>
  findKNearest(const VectorType &x, int k) const {
    auto queue = cmBoundedPQueue<T, Node *>(k);
    traverseDown(rootNode, queue, x);

    auto initial =
        std::pair<VectorType, T>{{0, 0, 0}, std::numeric_limits<T>::infinity()};
    auto result = lsSmartPointer<std::vector<std::pair<VectorType, T>>>::New();

    // TODO: handle cases where k might be larger than the number of available
    // points
    while (!queue.empty()) {
      auto best = queue.dequeueBest();
      result->emplace_back(
          std::pair{best->value, distanceInternal(x, best->value)});
    }
    return result;
  }

  ~cmKDTree() { recursiveDelete(rootNode); }
};

#endif