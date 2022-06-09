#pragma once

#include <vector>

struct cmGraph {
  std::vector<float> x;
  std::vector<long> edgeIndex;
  std::vector<float> edgeLengths;
  std::vector<float> edgeAngles;
  std::vector<float> edgeSourceConnection;
};