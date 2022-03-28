#include <iostream>

#include <lsDomain.hpp>
#include <lsGeometries.hpp>
#include <lsMakeGeometry.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>
#include <lsVTKWriter.hpp>

#include "cmExtractNormalThickness.hpp"
#include "cmInternal.hpp"

// Builds a disk mesh along one sidewall + topThickness
template <class NumericType, int D>
std::pair<lsSmartPointer<lsMesh<>>, std::array<unsigned, 3>>
createExtractionMesh(NumericType depth, NumericType topDiameter,
                     NumericType taperAngle, NumericType stickingProbability,
                     NumericType duration, NumericType gridDelta) {
  auto mesh = lsSmartPointer<lsMesh<>>::New();
  std::array<unsigned, 3> breaks;

  std::array<NumericType, 3> origin = {0., 0., 0.};

  NumericType taperAngleRadians = cmInternal::radians(taperAngle);

  NumericType topThickness = duration * stickingProbability;
  NumericType length = (depth + topThickness) / std::cos(taperAngleRadians);
  unsigned N = length / gridDelta;

  NumericType step[3] = {gridDelta * std::sin(taperAngleRadians),
                         gridDelta * std::cos(taperAngleRadians), 0.};

  std::vector<std::array<NumericType, 3>> normals(
      N, std::array<NumericType, 3>{std::cos(taperAngleRadians),
                                    std::sin(taperAngleRadians), 0.});

  NumericType offX = topThickness * std::tan(taperAngleRadians);

  // Left sidewall
  breaks[0] = 0;
  for (unsigned i = 0; i < N; ++i) {
    std::array<unsigned, 1> vertex;
    vertex[0] = i + 1;
    mesh->insertNextVertex(vertex);

    std::array<NumericType, 3> node{0.};

    node[0] = origin[0] - offX - topDiameter / 2 + i * step[0];
    node[1] = origin[1] + topThickness - i * step[1];

    mesh->insertNextNode(node);
  }

  // // Right sidewall
  // breaks[1] = ;
  // for (unsigned i = N; i < 2 * N; ++i) {
  //   std::array<unsigned, 1> vertex;
  //   vertex[0] = i + 1;
  //   mesh->insertNextVertex(vertex);

  //   std::array<NumericType, 3> node{0.};

  //   node[0] = origin[0] + offX + topDiameter / 2 - i * step[0];
  //   node[1] = origin[1] + topThickness - i * step[1];

  //   mesh->insertNextNode(node);
  // }

  mesh->getCellData().insertNextVectorData(normals, "Normals");
  return {mesh, {}};
}

int main() {
  constexpr int D = 2;
  using NumericType = double;

  // Trench was generated using:
  // - depth: 50
  // - topDiameter: 20
  // - taperAngle: 15
  // - sticking probability: 0.5
  // - duration: 30s
  auto [extractionMesh, breaks] =
      createExtractionMesh<NumericType, D>(50, 20, 5, 0.5, 30, 0.5);

  // auto baseLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  // lsReader<NumericType, D>(baseLayer, "first.lvst").apply();

  // auto baseMesh = lsSmartPointer<lsMesh<>>::New();
  // lsToDiskMesh<NumericType, D>(baseLayer, baseMesh).apply();

  auto depoLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(depoLayer, "second.lvst").apply();

  auto depoMesh = lsSmartPointer<lsMesh<>>::New();
  lsToDiskMesh<NumericType, D>(depoLayer, depoMesh).apply();

  auto extractor =
      lsSmartPointer<cmExtractNormalThickness<NumericType, D>>::New(
          extractionMesh, depoMesh, depoLayer->getGrid().getGridDelta());
  extractor->apply();

  lsVTKWriter<NumericType>(extractor->getBaseMesh(), "layerThickness.vtp")
      .apply();

  lsVTKWriter<NumericType>(extractor->getSecondMesh(), "depo.vtp").apply();

  return 0;
}