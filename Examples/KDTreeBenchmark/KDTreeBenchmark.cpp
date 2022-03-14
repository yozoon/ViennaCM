#include <array>
#include <iostream>
#include <random>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <lsSmartPointer.hpp>

#include "cmKDTree.hpp"
#include "cmVTKKDTree.hpp"

inline double getTime() {
#ifdef _OPENMP
  return omp_get_wtime();
#else
  return std::chrono::duration<double>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
#endif
}

template <class T, int D> std::vector<std::array<T, D>> generatePoints(int N) {
  std::random_device rd;
  std::vector<std::array<T, D>> data(N);

#pragma omp parallel default(none) shared(N, data, rd)
  {
    int numThreads = 1;
#ifdef _OPENMP
    numThreads = omp_get_num_threads();
#endif

    auto engine = std::default_random_engine(rd());
    std::normal_distribution<> d{0, 10};
#pragma omp for
    for (int i = 0; i < N; ++i) {
      if constexpr (D == 3) {
        data[i] = std::array<T, D>{d(engine), d(engine), d(engine)};
      } else {
        data[i] = std::array<T, D>{d(engine), d(engine)};
      }
    }
  }

  return data;
}

int main(int argc, char *argv[]) {
  using NumericType = double;
  static constexpr int D = 3;

  // The number of points in the tree
  int N = 1'000'000;
  if (argc > 1) {
    int tmp = std::atoi(argv[1]);
    if (tmp > 0)
      N = tmp;
  }

  // The number of points to query the tree with
  int M = 100'000;
  if (argc > 2) {
    int tmp = std::atoi(argv[2]);
    if (tmp > 0)
      M = tmp;
  }

  // Training Point generation
  std::cout << "Generating Training Points..." << std::endl;
  auto points = generatePoints<NumericType, D>(N);

  // Testing points generation
  std::cout << "Generating Testing Points..." << std::endl;
  auto testPoints = generatePoints<NumericType, D>(M);

  {
    // Custom Tree
    std::cout << "\nCM KDTree\n========\nGrowing Tree..." << std::endl;

    auto startTime = getTime();
    auto tree =
        lsSmartPointer<cmKDTree<std::array<NumericType, D>>>::New(points);
    tree->build();
    auto endTime = getTime();

    std::cout << "Tree grew in " << endTime - startTime << "s" << std::endl;

    // Nearest Neighbors
    std::cout << "Finding Nearest Neighbors..." << std::endl;
    startTime = getTime();
    for (const auto pt : testPoints)
      auto result = tree->findNearest(pt);

    std::cout << M << " nearest neighbor queries completed in "
              << getTime() - startTime << "s" << std::endl;
  }
  {
    std::cout << "\nVTK KDTree\n========\nGrowing Tree..." << std::endl;

    // VTK Tree
    auto startTime = getTime();
    auto vtkTree =
        lsSmartPointer<cmVTKKDTree<std::array<NumericType, D>>>::New(points);
    vtkTree->build();
    auto endTime = getTime();

    std::cout << "Tree grew in " << endTime - startTime << "s" << std::endl;

    // Nearest Neighbors with VTK Tree
    std::cout << "Finding Nearest Neighbors..." << std::endl;
    startTime = getTime();
    for (const auto pt : testPoints)
      auto result = vtkTree->findNearest(pt);

    endTime = getTime();
    std::cout << M << " nearest neighbor queries completed in "
              << endTime - startTime << "s" << std::endl;
  }
}
