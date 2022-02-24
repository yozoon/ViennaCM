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

int intLog2(int x) {
  int val = 0;
  while (x >>= 1)
    ++val;
  return val;
}

enum struct cmKDTreeDistanceEnum : unsigned { EUCLIDEAN = 0, CUSTOM = 1 };

template <class T, int D, class CoordType> class cmKDTree {
public:
  using SizeType = std::size_t;
  //using CoordType = std::array<T, D>;
  using DistanceFunctionType = T (*)(const CoordType &, const CoordType &);

private:
  SizeType N;
  SizeType treeSize = 0;

  int numThreads = 1;
  int maxParallelDepth = 0;
  int surplusWorkers = 0;

  lsSmartPointer<std::vector<CoordType>> points = nullptr;

  cmKDTreeDistanceEnum distanceType = cmKDTreeDistanceEnum::EUCLIDEAN;
  DistanceFunctionType customDistance = nullptr;
  DistanceFunctionType customReducedDistance = nullptr;

public:
  struct Node {
    CoordType &value;
    int axis;

    Node *parent = nullptr;
    Node *left = nullptr;
    Node *right = nullptr;

    Node(CoordType &passedValue, int passedAxis)
        : value(passedValue), axis(passedAxis) {}
  };

  /** For later use with kNN **/
  // template <class N> struct BoundedMultimap {
  //   using MapType = std::multimap<T, N>;

  // private:
  //   MapType mmap;

  // public:
  //   const SizeType k;

  //   BoundedMultimap(SizeType passedK) : k(passedK) {}

  //   typename MapType::iterator emplace(std::pair<T, N> &&item) {
  //     auto iterator = mmap.emplace(std::forward(item));
  //     if (mmap.size() > k)
  //       mmap.erase(mmap.rbegin());

  //     return iterator;
  //   }

  //   std::optional<N> getMinItem() {
  //     auto item = mmap.begin();
  //     if (item != nullptr) {
  //       mmap.erase(mmap.begin());
  //       return {item->second};
  //     } else {
  //       return {};
  //     }
  //   }

  //   T min() const {
  //     return mmap.empty() ? std::numeric_limits<T>::max() :
  //     mmap.begin()->first;
  //   }

  //   T max() const {
  //     return mmap.empty() ? std::numeric_limits<T>::max()
  //                         : mmap.rbegin()->first;
  //   }
  // };

private:
  Node *rootNode = nullptr;

  void build(Node *parent, typename std::vector<CoordType>::iterator start,
             typename std::vector<CoordType>::iterator end, int depth,
             bool isLeft) {
    SizeType size = end - start;

    int axis = depth % D;
    // TODO: Add bucket size parameter
    if (size > 1) {
      SizeType medianIndex = (size + 1) / 2 - 1;
      std::nth_element(
          start, start + medianIndex, end,
          [&](CoordType &a, CoordType &b) { return a[axis] < b[axis]; });

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

  void recursiveFree(Node *node) {
    if (node != nullptr) {
      if (node->left != nullptr)
        recursiveFree(node->left);
      if (node->right != nullptr)
        recursiveFree(node->right);

      delete node;
    }
  }

  static T euclideanReducedDistance(const CoordType &a, const CoordType &b) {
    T sum{0};
    for (int i = 0; i < D; i++) {
      T d = b[i] - a[i];
      sum += d * d;
    }
    return sum;
  }

  static T euclideanDistance(const CoordType &a, const CoordType &b) {
    return std::sqrt(euclideanReducedDistance(a, b));
  }

  T distanceReducedInternal(const CoordType &a, const CoordType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::CUSTOM:
      return customReducedDistance(a, b);
      break;
    default:
      return euclideanReducedDistance(a, b);
    }
  }

  T distanceInternal(const CoordType &a, const CoordType &b) const {
    switch (distanceType) {
    case cmKDTreeDistanceEnum::CUSTOM:
      return customDistance(a, b);
    default:
      return euclideanDistance(a, b);
    }
  }

  std::pair<Node *, T> &traverseDown(Node *currentNode,
                                     std::pair<Node *, T> &best,
                                     const CoordType &x) const {
    if (currentNode == nullptr)
      return best;

    int axis = currentNode->axis;

    // For distance comparison operations we only use the "reduced" aka less
    // compute intensive, but order preserving version of the distance
    // function.
    T distance = distanceReducedInternal(x, currentNode->value);
    if (distance < best.second)
      best = {currentNode, distance};

    bool isLeft;
    if (x[axis] < currentNode->value[axis]) {
      best = traverseDown(currentNode->left, best, x);
      isLeft = true;
    } else {
      best = traverseDown(currentNode->right, best, x);
      isLeft = false;
    }

    // If the hypersphere with origin at x and a radius of our current best
    // distance intersects the hyperplane defined by the partitioning of the
    // current node, we also have to search the other subtree, since there could
    // be points closer to x than our current best.
    if (std::abs(x[axis] - currentNode->value[axis]) < best.second) {
      if (isLeft)
        best = traverseDown(currentNode->right, best, x);
      else
        best = traverseDown(currentNode->left, best, x);
    }
    return best;
  }

public:
  cmKDTree() {}

  cmKDTree(lsSmartPointer<std::vector<CoordType>> passedPoints)
      : N(passedPoints != nullptr ? passedPoints->size() : 0),
        points(passedPoints) {}

  void build() {
    if (points == nullptr) {
      std::cout << "No points available" << std::endl;
      return;
    }

    if (rootNode != nullptr) {
      std::cout << "Tree has already been built" << std::endl;
      return;
    }

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
            [&](CoordType &a, CoordType &b) { return a[0] < b[0]; });

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

  std::pair<CoordType, T> nearest(const CoordType &x) const {
    auto best = std::pair{rootNode, std::numeric_limits<T>::infinity()};
    best = traverseDown(rootNode, best, x);

    return {best.first->value, distanceInternal(x, best.first->value)};
  }

  ~cmKDTree() { recursiveFree(rootNode); }
};