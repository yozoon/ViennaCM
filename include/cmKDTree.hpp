#ifndef CM_KDTREE_HPP
#define CM_KDTREE_HPP

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include <lsMessage.hpp>
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

enum struct cmKDTreeDistanceEnum : unsigned {
  MANHATTAN = 0,
  EUCLIDEAN = 1,
  CUSTOM = 2
};

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
  lsSmartPointer<std::vector<SizeType>> pointIndices = nullptr;

  cmKDTreeDistanceEnum distanceType = cmKDTreeDistanceEnum::EUCLIDEAN;
  DistanceFunctionType customDistance = nullptr;
  DistanceFunctionType customReducedDistance = nullptr;

public:
  struct Node {
    SizeType &index;
    int axis;

    Node *left = nullptr;
    Node *right = nullptr;

    Node(SizeType &passedIndex, int passedAxis)
        : index(passedIndex), axis(passedAxis) {}
  };

private:
  Node *rootNode = nullptr;

  void build(Node *parent, typename std::vector<SizeType>::iterator start,
             typename std::vector<SizeType>::iterator end, int depth,
             bool isLeft) {
    SizeType size = end - start;

    int axis = depth % D;

    if (size > 1) {
      SizeType medianIndex = (size + 1) / 2 - 1;
      // TODO: We could think about adding custom comparison operations, which
      // would allow for splitting of the data along D arbitrary axes.
      std::nth_element(start, start + medianIndex, end,
                       [&](SizeType &a, SizeType &b) {
                         return (*points)[a][axis] < (*points)[b][axis];
                       });

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

  /****************************************************************************
   * Distance Functions                                                       *
   ****************************************************************************/
  static T manhattanReducedDistance(const VectorType &a, const VectorType &b) {
    T sum{0};
    for (int i = 0; i < D; i++)
      sum += std::abs(b[i] - a[i]);

    return sum;
  }

  static T euclideanReducedDistance(const VectorType &a, const VectorType &b) {
    T sum{0};
    for (int i = 0; i < D; i++) {
      T d = b[i] - a[i];
      sum += d * d;
    }
    return sum;
  }

  static T manhattanDistance(const VectorType &a, const VectorType &b) {
    return manhattanReducedDistance(a, b);
  }

  static T euclideanDistance(const VectorType &a, const VectorType &b) {
    return std::sqrt(euclideanReducedDistance(a, b));
  }

  T distanceReducedInternal(const VectorType &a, const VectorType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::MANHATTAN:
      return manhattanReducedDistance(a, b);
    case cmKDTreeDistanceEnum::CUSTOM:
      return customReducedDistance(a, b);
      break;
    default:
      return euclideanReducedDistance(a, b);
    }
  }

  T distanceInternal(const VectorType &a, const VectorType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::MANHATTAN:
      return manhattanDistance(a, b);
    case cmKDTreeDistanceEnum::CUSTOM:
      return customDistance(a, b);
    default:
      return euclideanDistance(a, b);
    }
  }

  /****************************************************************************
   * Recursive Tree Traversal                                                 *
   ****************************************************************************/
  template <class Q>
  void traverseDown(Node *currentNode, Q &queue, const VectorType &x) const {
    if (currentNode == nullptr)
      return;

    int axis = currentNode->axis;

    // For distance comparison operations we only use the "reduced" aka less
    // compute intensive, but order preserving version of the distance
    // function.
    if constexpr (std::is_same_v<Q, cmBoundedPQueue<T, Node *>>) {
      queue.enqueue(
          std::pair{distanceReducedInternal(x, (*points)[currentNode->index]),
                    currentNode});
    } else {
      T distance = distanceReducedInternal(x, (*points)[currentNode->index]);
      if (distance < queue.second)
        queue = std::pair{currentNode, distance};
    }

    bool isLeft;
    if (x[axis] < (*points)[currentNode->index][axis]) {
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
    if constexpr (std::is_same_v<Q, cmBoundedPQueue<T, Node *>>) {
      if (queue.size() < queue.maxSize() ||
          std::abs(x[axis] - (*points)[currentNode->index][axis]) <
              queue.worst()) {
        if (isLeft)
          traverseDown(currentNode->right, queue, x);
        else
          traverseDown(currentNode->left, queue, x);
      }
    } else {
      if (std::abs(x[axis] - (*points)[currentNode->index][axis]) <
          queue.second) {
        if (isLeft)
          traverseDown(currentNode->right, queue, x);
        else
          traverseDown(currentNode->left, queue, x);
      }
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

    // Initialize the point index vector
    pointIndices = lsSmartPointer<std::vector<SizeType>>::New();
    pointIndices->reserve(points->size());
    // Create a new vector containing the point indices. This is the one we
    // will
    // modify with the sort operations later on. This ensures that the passed
    // points won't be reordered.
    // TODO: This can be parallelized!
    {
      auto pointIterator = pointIndices->begin();
      for (SizeType i = 0; i < points->size(); i++) {
        pointIndices->emplace(pointIterator, i);
        pointIterator++;
      }
    }

#pragma omp parallel default(none)                                             \
    shared(numThreads, maxParallelDepth, surplusWorkers, std::cout, rootNode,  \
           treeSize, points, pointIndices)
    {
      int threadID = 0;
#pragma omp single
      {
#ifdef _OPENMP
        numThreads = omp_get_num_threads();
        threadID = omp_get_thread_num();
#endif
        maxParallelDepth = intLog2(numThreads);
        surplusWorkers = numThreads - 1 << maxParallelDepth;

#ifndef NDEBUG
        std::cout << "Starting parallel region with " << numThreads
                  << " parallel workers." << std::endl;
#endif

        SizeType size = pointIndices->end() - pointIndices->begin();
        SizeType medianIndex = (size + 1) / 2 - 1;

        std::nth_element(pointIndices->begin(),
                         pointIndices->begin() + medianIndex,
                         pointIndices->end(), [&](SizeType &a, SizeType &b) {
                           return (*points)[a][0] < (*points)[b][0];
                         });

        rootNode = new Node(*(pointIndices->begin() + medianIndex), 0);

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
          build(rootNode,                            // Use rootNode as parent
                pointIndices->begin(),               // Data start
                pointIndices->begin() + medianIndex, // Data end
                1,                                   // Depth
                true                                 // Left
          );
        }

        // Right Subtree
        build(rootNode,                                // Use rootNode as parent
              pointIndices->begin() + medianIndex + 1, // Data start
              pointIndices->end(),                     // Data end
              1,                                       // Depth
              false                                    // Right
        );
#pragma omp taskwait
      }
    }
  }

  std::pair<SizeType, T> findNearest(const VectorType &x) const {
    auto best = std::pair{rootNode, std::numeric_limits<T>::infinity()};
    traverseDown(rootNode, best, x);
    return {best.first->index,
            distanceInternal(x, (*points)[best.first->index])};
  }

  lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findKNearest(const VectorType &x, const int k) const {
    auto queue = cmBoundedPQueue<T, Node *>(k);
    traverseDown(rootNode, queue, x);

    auto initial = std::pair<SizeType, T>{rootNode->index,
                                          std::numeric_limits<T>::infinity()};
    auto result = lsSmartPointer<std::vector<std::pair<SizeType, T>>>::New();

    // TODO: handle cases where k might be larger than the number of available
    // points
    while (!queue.empty()) {
      auto best = queue.dequeueBest();
      result->emplace_back(
          std::pair{best->index, distanceInternal(x, (*points)[best->index])});
    }
    return result;
  }

  ~cmKDTree() { recursiveDelete(rootNode); }
};

#endif