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

#include <cmInternal.hpp>
#include <cmPointLocator.hpp>
#include <cmQueues.hpp>

template <class NumericType, int D, int Dim = D>
class cmKDTree : cmPointLocator<NumericType, D, Dim> {
  using typename cmPointLocator<NumericType, D, Dim>::VectorType;
  using typename cmPointLocator<NumericType, D, Dim>::SizeType;

  SizeType N;
  SizeType treeSize = 0;

  int numThreads = 1;
  int maxParallelDepth = 0;
  int surplusWorkers = 0;

  NumericType gridDelta;

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

  template <class Iterator>
  static typename Iterator::pointer toRawPointer(const Iterator it) {
    return &(*it);
  }

  void build(Node *parent, typename std::vector<Node>::iterator start,
             typename std::vector<Node>::iterator end, int depth,
             bool isLeft) const {
    SizeType size = end - start;

    int axis = depth % D;

    if (size > 1) {
      SizeType medianIndex = (size + 1) / 2 - 1;
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
  /****************************************************************************
   * Recursive Tree Traversal                                                 *
   ****************************************************************************/
  void traverseDown(Node *currentNode, std::pair<NumericType, Node *> &best,
                    const VectorType &x) const {
    if (currentNode == nullptr)
      return;

    int axis = currentNode->axis;

    // For distance comparison operations we only use the "reduced" aka less
    // compute intensive, but order preserving version of the distance
    // function.
    NumericType distance =
        cmInternal::euclideanReducedDistance(x, currentNode->value);
    if (distance < best.first)
      best = std::pair{distance, currentNode};

    bool isLeft;
    if (x[axis] < currentNode->value[axis]) {
      traverseDown(currentNode->left, best, x);
      isLeft = true;
    } else {
      traverseDown(currentNode->right, best, x);
      isLeft = false;
    }

    // If the hypersphere with origin at x and a radius of our current best
    // distance intersects the hyperplane defined by the partitioning of the
    // current node, we also have to search the other subtree, since there could
    // be points closer to x than our current best.
    NumericType distanceToHyperplane =
        std::abs(x[axis] - currentNode->value[axis]);
    distanceToHyperplane *= distanceToHyperplane;
    if (distanceToHyperplane < best.first) {
      if (isLeft)
        traverseDown(currentNode->right, best, x);
      else
        traverseDown(currentNode->left, best, x);
    }
    return;
  }

  template <typename Q,
            typename = std::enable_if_t<
                std::is_same_v<Q, cmBoundedPQueue<NumericType, Node *>> ||
                std::is_same_v<Q, cmClampedPQueue<NumericType, Node *>>>>
  void traverseDown(Node *currentNode, Q &queue, const VectorType &x) const {
    if (currentNode == nullptr)
      return;

    int axis = currentNode->axis;

    // For distance comparison operations we only use the "reduced" aka less
    // compute intensive, but order preserving version of the distance
    // function.
    queue.enqueue(
        std::pair{cmInternal::euclideanReducedDistance(x, currentNode->value),
                  currentNode});

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
    NumericType distanceToHyperplane =
        std::abs(x[axis] - currentNode->value[axis]);
    distanceToHyperplane *= distanceToHyperplane;

    bool intersects = false;
    if constexpr (std::is_same_v<Q, cmBoundedPQueue<NumericType, Node *>>) {
      intersects = queue.size() < queue.maxSize() ||
                   distanceToHyperplane < queue.worst();
    } else if constexpr (std::is_same_v<Q,
                                        cmClampedPQueue<NumericType, Node *>>) {
      intersects = distanceToHyperplane < queue.worst();
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

  cmKDTree(std::vector<VectorType> &passedPoints,
           NumericType passedGridDelta = 0.) {
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

  std::pair<SizeType, NumericType>
  findNearest(const VectorType &x) const override {
    auto best =
        std::pair{std::numeric_limits<NumericType>::infinity(), rootNode};
    traverseDown(rootNode, best, x);
    return {best.second->index,
            cmInternal::euclideanDistance(x, best.second->value)};
  }

  lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>
  findKNearest(const VectorType &x, const int k) const override {
    auto queue = cmBoundedPQueue<NumericType, Node *>(k);
    traverseDown(rootNode, queue, x);

    auto initial = std::pair<SizeType, NumericType>{
        rootNode->index, std::numeric_limits<NumericType>::infinity()};
    auto result =
        lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>::New();

    while (!queue.empty()) {
      auto best = queue.dequeueBest();
      result->emplace_back(std::pair{
          best->index, cmInternal::euclideanDistance(x, best->value)});
    }
    return result;
  }

  lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>
  findNearestWithinRadius(const VectorType &x,
                          const NumericType radius) const override {
    auto queue = cmClampedPQueue<NumericType, Node *>(radius);
    traverseDown(rootNode, queue, x);

    auto initial = std::pair<SizeType, NumericType>{
        rootNode->index, std::numeric_limits<NumericType>::infinity()};
    auto result =
        lsSmartPointer<std::vector<std::pair<SizeType, NumericType>>>::New();

    while (!queue.empty()) {
      auto best = queue.dequeueBest();
      result->emplace_back(std::pair{
          best->index, cmInternal::euclideanDistance(x, best->value)});
    }
    return result;
  }
};

#endif