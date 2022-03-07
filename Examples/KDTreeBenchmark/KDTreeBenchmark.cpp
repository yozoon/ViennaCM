#include <array>
#include <iostream>
#include <random>
#include <vector>

#include <lsSmartPointer.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "cmKDTree.hpp"
#include "cmVTKKDTree.hpp"

inline double get_time() {
#ifdef _OPENMP
  return omp_get_wtime();
#else
  return std::chrono::duration<double>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
#endif
}

template <class T, int D>
lsSmartPointer<std::vector<std::array<T, D>>> generatePoints(int N) {
  auto data = lsSmartPointer<std::vector<std::array<T, D>>>::New();
  data->reserve(N);
  std::random_device rd{};
  std::mt19937 gen{rd()};

  std::normal_distribution<> d{0, 10};
  auto it = data->begin();
  for (int i = 0; i < N; i++) {
    data->emplace(it, std::array<T, D>{d(gen), d(gen), d(gen)});
    it++;
  }
  return data;
}

int main(int argc, char *argv[]) {
  using NumericType = double;
  constexpr int D = 3;

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

  // Custom Tree
  std::cout << "\nCM KDTree\n========\nGrowing Tree..." << std::endl;

  auto startTime = get_time();
  auto tree =
      lsSmartPointer<cmKDTree<std::array<NumericType, D>>>::New(*points);
  tree->build();

  std::cout << "Tree grew in " << get_time() - startTime << "s" << std::endl;

  // Nearest Neighbors
  std::cout << "Finding Nearest Neighbors..." << std::endl;
  startTime = get_time();
  for (const auto pt : *testPoints)
    auto result = tree->findNearest(pt);

  std::cout << M << " nearest neighbor queries completed in "
            << get_time() - startTime << "s" << std::endl;

  std::cout << "\nVTK KDTree\n========\nGrowing Tree..." << std::endl;

  // VTK Tree
  startTime = get_time();
  auto vtkTree =
      lsSmartPointer<cmVTKKDTree<std::array<NumericType, D>>>::New(*points);
  vtkTree->build();

  std::cout << "Tree grew in " << get_time() - startTime << "s" << std::endl;

  // Nearest Neighbors with VTK Tree
  std::cout << "Finding Nearest Neighbors..." << std::endl;
  startTime = get_time();
  for (const auto pt : *testPoints)
    auto result = vtkTree->findNearest(pt);

  std::cout << M << " nearest neighbor queries completed in "
            << get_time() - startTime << "s" << std::endl;
}
