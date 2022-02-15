#include <algorithm>

#include <vtkExtractSurface.hpp>
#include <vtkNew.hpp>
#include <vtkPointData.hpp>
#include <vtkSignedDistance.h>

#include <lsGeometries.hpp>
#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>

template <class T, int D> class cmPointsToMesh {
public:
  lsSmartPointer<lsMesh<T>> mesh = nullptr;
  lsSmartPointer<lsPointCloud<T, D>> pointCloud = nullptr;
  lsSmartPointer<hrleVectorType<T, D>> normals = nullptr;

  cmPointsToMesh() {}

  cmPointsToMesh(lsSmartPointer<lsMesh<T>> passedMesh,
                 lsSmartPointer<lsPointCloud<T, D>> passedPointCloud)
      : mesh(passedMesh), pointCloud(passedPointCloud) {}

  cmPointsToMesh(lsSmartPointer<lsMesh<T>> passedMesh,
                 lsSmartPointer<lsPointCloud<T, D>> passedPointCloud,
                 lsSmartPointer<hrleVectorType<T, D>> passedNormals)
      : mesh(passedMesh), pointCloud(passedPointCloud), normals(passedNormals) {
  }

  void setMesh(lsSmartPointer<lsMesh<T>> passedMesh) { mesh = passedMesh; }

  void setPointCloud(lsSmartPointer<lsPointCloud<T, D>> passedPointCloud) {
    pointCloud = passedPointCloud;
  }

  void setNormals(lsSmartPointer<hrleVectorType<T, D>> passedNormals) {
    normals = passedNormals;
  }

  void apply() {

    vtkNew<vtkSignedDistance> distance;

    if (normals == nullptr) {
      // Estimate Normals
      vtkNew<vtkPCANormalEstimation> normals;
      normals->SetInputData(polyData);
      normals->SetSampleSize(sampleSize);
      normals->SetNormalOrientationToGraphTraversal();
      normals->FlipNormalsOn();
      distance->SetInputConnection(normals->GetOutputPort());
    } else {
      
    }

    int dimension = 256;
    double radius;
    radius = std::max(std::max(range[0], range[1]), range[2]) /
             static_cast<double>(dimension) * 4; // ~4 voxels

    distance->SetRadius(radius);
    distance->SetDimensions(dimension, dimension, dimension);
    distance->SetBounds(bounds[0] - range[0] * .1, bounds[1] + range[0] * .1,
                        bounds[2] - range[1] * .1, bounds[3] + range[1] * .1,
                        bounds[4] - range[2] * .1, bounds[5] + range[2] * .1);

    surface->SetInputConnection(distance->GetOutputPort());
    surface->SetRadius(radius * .99);
    surface->Update();
  }
};
