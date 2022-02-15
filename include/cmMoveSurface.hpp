#include <hrleSparseIterator.hpp>
#include <lsCalculateCurvatures.hpp>
#include <lsCalculateNormalVectors.hpp>
#include <lsDomain.hpp>
#include <lsExpand.hpp>
#include <lsFromSurfaceMesh.hpp>
#include <lsGeometries.hpp>
#include <lsMakeGeometry.hpp>
#include <lsToDiskMesh.hpp>

#include <unordered_map>

#include "cmVectorHash.hpp"

template <class T, int D> class cmMoveSurface {
private:
public:
  lsSmartPointer<lsDomain<T, D>> levelSet = nullptr;
  lsSmartPointer<lsDomain<T, D>> newLevelSet = nullptr;

  const T thickness = 0.;
  static constexpr T maxValue = 0.5;

  cmMoveSurface() {}

  cmMoveSurface(lsSmartPointer<lsDomain<T, D>> passedLevelSet,
                lsSmartPointer<lsDomain<T, D>> passedNewLevelSet,
                T passedThickness)
      : levelSet(passedLevelSet), newLevelSet(passedNewLevelSet),
        thickness(passedThickness) {}

  void apply() {
    auto surfaceMesh = lsSmartPointer<lsMesh<hrleCoordType>>::New();
    auto pointIdTranslator =
        lsSmartPointer<typename lsToDiskMesh<T, D>::TranslatorType>::New();

    lsToDiskMesh<T, D, hrleCoordType>(levelSet, surfaceMesh, pointIdTranslator)
        .apply();

    const auto &normals = *(surfaceMesh->cellData.getVectorData(
        lsCalculateNormalVectors<T, D>::normalVectorsLabel));

    auto pointCloud = lsSmartPointer<lsPointCloud<T, D>>::New();
    std::unordered_map<hrleVectorType<T, D>, bool, cmVectorHash<T, D>> pointMap;
    unsigned count = 0;
    for (const auto &node : surfaceMesh->nodes) {
      hrleVectorType<T, D> point;
      for (int i = 0; i < D; i++)
        point[i] = node[i];

      count++;

      pointCloud->insertNextPoint(point);
      if (pointMap.find(point) == pointMap.end()) {
        pointMap.insert(std::make_pair(point, true));
      } else {
        std::cout << "Duplicate!\n";
      }
    }

    auto mesh = lsSmartPointer<lsMesh<>>::New();

    //! TODO: We need a different approach to generate a mesh from the point
    //! cloud, since we don't actually want a convex hull
    lsConvexHull<T, D>(mesh, pointCloud).apply();

    // Now create a new LS from the mesh
    lsFromSurfaceMesh<T, D>(newLevelSet, mesh, true).apply();
  }
};