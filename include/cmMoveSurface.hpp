#include <hrleSparseIterator.hpp>
#include <lsCalculateNormalVectors.hpp>
#include <lsDomain.hpp>
#include <lsExpand.hpp>
#include <lsFromSurfaceMesh.hpp>
#include <lsGeometries.hpp>
#include <lsMakeGeometry.hpp>
#include <lsToDiskMesh.hpp>

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
    for (const auto &node : surfaceMesh->nodes) {
      hrleVectorType<T, D> point(node[0], node[1]);
      if constexpr (D == 3)
        point[2] = node[2];

      pointCloud->insertNextPoint(point);
    }

    auto mesh = lsSmartPointer<lsMesh<>>::New();

    std::cout << surfaceMesh->nodes.size() << ", " << pointCloud->size()
              << std::endl;

    //! TODO: We need a different approach to generate a mesh from the point
    //! cloud, since we don't actually want a convex hull
    lsConvexHull<T, D>(mesh, pointCloud).apply();

    // Now create a new LS from the mesh
    // lsFromSurfaceMesh<T, D>(newLevelSet, mesh, false).apply();
  }
};