#include <algorithm>
#include <array>
#include <vector>

#include <lsMessage.hpp>

template <class T, int D> class cmKDTree {
  using PointType = std::array<T, 3>;

  std::vector<PointType> points;
  size_t N;

  std::array<std::vector<size_t>, D> indices;
  size_t maxDepth = std::numeric_limits<size_t>::max();

  struct Node {
    T value;
    Node *leftChild = nullptr;
    Node *rightChild = nullptr;
    Node() {}
    Node(T passedValue, Node *passedLeftChild, Node *passedRightChild)
        : value(passedValue), rightChild(passedRightChild),
          leftChild(passedLeftChild) {}
  };

  Node *rootNode = nullptr;

  int nextAxis(int axis) const { return (axis + 1) % D; }

  void recurse(Node *self, const std::vector<size_t> &indices, size_t depth,
               int axis) {
    if (indices.size() == 1)
      return;
    if (depth >= maxDepth)
      return;

    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
      return points[a][axis] < points[b][axis];
    });

    size_t median = indices.size() / 2;

    Node *left = new Node();
    Node *right = new Node();

    int newAxis = nextAxis(axis);

    recurse(left,
            std::vector<size_t>(indices.begin(), indices.begin() + median),
            depth + 1, newAxis);

    recurse(right, std::vector<size_t>(indices.begin() + median, indices.end()),
            depth + 1, newAxis);

    self->value = points[median][axis];
    self->leftChild = left;
    self->rightChild = right;
  }

  void recursiveFree(Node *node) {
    if (node->leftChild != nullptr)
      recursiveFree(node->leftChild);
    if (node->rightChild != nullptr)
      recursiveFree(node->rightChild);
    free(node);
  }

public:
  cmKDTree() {}
  cmKDTree(std::vector<PointType> &passedPoints)
      : points(passedPoints), N(points.size()) {}

  void build() {
    if (points.size() == 0) {
      lsMessage::getInstance()
          .addWarning("No points have been provided.")
          .print();
      return;
    }
    std::vector<size_t> indices;
    // Initialize the first index array with incrementing values
    for (unsigned i = 0; i < N; i++)
      indices.emplace_back(i);

    //recurse(rootNode, indices, 0, 0);

    /*
    // Copy content of first vector to the others
    indices[1] = indices[0];
    if constexpr (D == 3)
      indices[2] = indices[0];

    // Sort the index vectors based on the coordinate value in each dimension
    for (int i = 0; i < D; i++)
      std::sort(indices[i].begin(), indices[i].end(), [&](size_t a, size_t b) {
        return points[a][i] < points[b][i];
      });

    for (int i = 0; i < D; i++)
      std::cout << "Index of first point in axis " << i << " is "
                << indices[i][0] << std::endl;

    // Initialize index bounds of root node
    rootNode.indexBounds = {0};
    rootNode.items = N;
    */
  }

  void findNearestNeighbor(PointType &x) const {}

  ~cmKDTree() { recursiveFree(rootNode); }
};