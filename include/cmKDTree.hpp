#ifndef CM_KDTREE_HPP
#define CM_KDTREE_HPP

// Inspired by the implementation of a parallelized kD-Tree by Francesco
// Andreuzzi (https://github.com/fAndreuzzi/parallel-kd-tree)
//
// --------------------- BEGIN ORIGINAL COPYRIGHT NOTICE ---------------------//
// MIT License
//
// Copyright (c) 2021 Francesco Andreuzzi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ---------------------- END ORIGINAL COPYRIGHT NOTICE ----------------------//

#include <algorithm>
#include <array>
#include <memory>
#include <vector>

#include <lsMessage.hpp>
#include <lsSmartPointer.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "cmBoundedPQueue.hpp"
#include "cmClampedPQueue.hpp"
#include "cmInternal.hpp"
#include "cmPointLocator.hpp"

enum struct cmKDTreeDistanceEnum : unsigned {
  MANHATTAN = 0,
  EUCLIDEAN = 1,
  CUSTOM = 2
};

template <class Iterator>
typename Iterator::pointer toRawPointer(const Iterator it) {
  return &(*it);
}

template <class VectorType> class cmKDTree : cmPointLocator<VectorType> {
  using typename cmPointLocator<VectorType>::SizeType;
  using typename cmPointLocator<VectorType>::T;
  using cmPointLocator<VectorType>::D;
  using typename cmPointLocator<VectorType>::DistanceFunctionType;

  SizeType N;
  SizeType treeSize = 0;

  int numThreads = 1;
  int maxParallelDepth = 0;
  int surplusWorkers = 0;

  cmKDTreeDistanceEnum distanceType = cmKDTreeDistanceEnum::EUCLIDEAN;
  DistanceFunctionType customDistance = nullptr;
  DistanceFunctionType customReducedDistance = nullptr;
  struct Node {
    VectorType value;
    SizeType index;
    int axis;

    Node *left = nullptr;
    Node *right = nullptr;

    Node(VectorType &passedValue, SizeType passedIndex)
        : value(passedValue), index(passedIndex) {}

    Node(Node &&other) {
      value.swap(other.value);
      index = other.index;
      axis = other.axis;

      left = other.left;
      right = other.right;

      other.left = nullptr;
      other.right = nullptr;
    }

    Node &operator=(Node &&other) {
      value.swap(other.value);
      index = other.index;
      axis = other.axis;

      left = other.left;
      right = other.right;

      other.left = nullptr;
      other.right = nullptr;

      return *this;
    }
  };

  std::vector<Node> nodes;
  Node *rootNode = nullptr;

  void build(Node *parent, typename std::vector<Node>::iterator start,
             typename std::vector<Node>::iterator end, int depth,
             bool isLeft) const {
    SizeType size = end - start;

    int axis = depth % D;

    if (size > 1) {
      SizeType medianIndex = (size + 1) / 2 - 1;
      // TODO: We could think about adding custom comparison operations, which
      // would allow for splitting of the data along D arbitrary axes.
      std::nth_element(
          start, start + medianIndex, end,
          [axis](Node &a, Node &b) { return a.value[axis] < b.value[axis]; });

      Node *current = toRawPointer(start + medianIndex);
      current->axis = axis;

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
      Node *current = toRawPointer(start);
      current->axis = axis;
      // Leaf Node
      if (isLeft)
        parent->left = current;
      else
        parent->right = current;
    }
  }

private:
  T distanceReducedInternal(const VectorType &a, const VectorType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::MANHATTAN:
      return cmInternal::manhattanReducedDistance(a, b);
    case cmKDTreeDistanceEnum::CUSTOM:
      return customReducedDistance(a, b);
      break;
    default:
      return cmInternal::euclideanReducedDistance(a, b);
    }
  }

  T distanceInternal(const VectorType &a, const VectorType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::MANHATTAN:
      return cmInternal::manhattanDistance(a, b);
    case cmKDTreeDistanceEnum::CUSTOM:
      return customDistance(a, b);
    default:
      return cmInternal::euclideanDistance(a, b);
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
    if constexpr (std::is_same_v<Q, cmBoundedPQueue<T, Node *>> ||
                  std::is_same_v<Q, cmClampedPQueue<T, Node *>>) {
      queue.enqueue(std::pair{distanceReducedInternal(x, currentNode->value),
                              currentNode});
    } else {
      T distance = distanceReducedInternal(x, currentNode->value);
      if (distance < queue.second)
        queue = std::pair{currentNode, distance};
    }

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
    bool intersects = false;

    if constexpr (std::is_same_v<Q, cmBoundedPQueue<T, Node *>>) {
      intersects = queue.size() < queue.maxSize() ||
                   std::abs(x[axis] - currentNode->value[axis]) < queue.worst();
    } else if constexpr (std::is_same_v<Q, cmClampedPQueue<T, Node *>>) {
      intersects = std::abs(x[axis] - currentNode->value[axis]) < queue.worst();
    } else {
      intersects = std::abs(x[axis] - currentNode->value[axis]) < queue.second;
    }

    if (intersects) {
      if (isLeft)
        traverseDown(currentNode->right, queue, x);
      else
        traverseDown(currentNode->left, queue, x);
    }
    return;
  }

public:
  cmKDTree() {}

  cmKDTree(std::vector<VectorType> &passedPoints) {
    nodes.reserve(passedPoints.size());
    {
      for (SizeType i = 0; i < passedPoints.size(); ++i) {
        nodes.emplace_back(Node{passedPoints[i], i});
      }
    }
  }

  void build() override {
    if (nodes.size() == 0) {
      lsMessage::getInstance().addWarning("No points provided!").print();
      return;
    }

#pragma omp parallel default(none)                                             \
    shared(numThreads, maxParallelDepth, surplusWorkers, std::cout, rootNode,  \
           treeSize, nodes)
    {
      int threadID = 0;
#pragma omp single
      {
#ifdef _OPENMP
        numThreads = omp_get_num_threads();
        threadID = omp_get_thread_num();
#endif
        maxParallelDepth = cmInternal::intLog2(numThreads);
        surplusWorkers = numThreads - 1 << maxParallelDepth;

        SizeType size = nodes.end() - nodes.begin();
        SizeType medianIndex = (size + 1) / 2 - 1;

        std::nth_element(
            nodes.begin(), nodes.begin() + medianIndex, nodes.end(),
            [](Node &a, Node &b) { return a.value[0] < b.value[0]; });

        rootNode = &nodes[medianIndex];
        rootNode->axis = 0;

#ifdef _OPENMP
        bool dontSpawnMoreThreads = 0 > maxParallelDepth + 1 ||
                                    (0 == maxParallelDepth + 1 &&
                                     omp_get_thread_num() >= surplusWorkers);
#endif
#pragma omp task final(dontSpawnMoreThreads)
        {
          // Left Subtree
          build(rootNode,                    // Use rootNode as parent
                nodes.begin(),               // Data start
                nodes.begin() + medianIndex, // Data end
                1,                           // Depth
                true                         // Left
          );
        }

        // Right Subtree
        build(rootNode,                        // Use rootNode as parent
              nodes.begin() + medianIndex + 1, // Data start
              nodes.end(),                     // Data end
              1,                               // Depth
              false                            // Right
        );
#pragma omp taskwait
      }
    }
  }

  std::pair<SizeType, T> findNearest(const VectorType &x) const override {
    auto best = std::pair{rootNode, std::numeric_limits<T>::infinity()};
    traverseDown(rootNode, best, x);
    return {best.first->index, distanceInternal(x, best.first->value)};
  }

  lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findKNearest(const VectorType &x, const int k) const override {
    auto queue = cmBoundedPQueue<T, Node *>(k);
    traverseDown(rootNode, queue, x);

    auto initial = std::pair<SizeType, T>{rootNode->index,
                                          std::numeric_limits<T>::infinity()};
    auto result = lsSmartPointer<std::vector<std::pair<SizeType, T>>>::New();

    while (!queue.empty()) {
      auto best = queue.dequeueBest();
      result->emplace_back(
          std::pair{best->index, distanceInternal(x, best->value)});
    }
    return result;
  }

  lsSmartPointer<std::vector<std::pair<SizeType, T>>>
  findNearestWithinRadius(const VectorType &x, const T radius) const override {
    auto queue = cmClampedPQueue<T, Node *>(radius);
    traverseDown(rootNode, queue, x);

    auto initial = std::pair<SizeType, T>{rootNode->index,
                                          std::numeric_limits<T>::infinity()};
    auto result = lsSmartPointer<std::vector<std::pair<SizeType, T>>>::New();

    while (!queue.empty()) {
      auto best = queue.dequeueBest();
      result->emplace_back(
          std::pair{best->index, distanceInternal(x, best->value)});
    }
    return result;
  }
};

#endif