#pragma once

#include <mutex>
#include <numeric>
#include <shared_mutex>
#include <vector>

#include <lsCalculateNormalVectors.hpp>
#include <lsDomain.hpp>
#include <lsGeometricAdvectDistributions.hpp>

#include "ViewFactorProcessData.hpp"

inline static std::shared_mutex lock;

template <class T, int D, class mapType>
class ViewFactorDistribution : public lsGeometricAdvectDistribution<T, D> {
public:
  ViewFactorProcessDataType<T> data;
  T closingTime;
  std::vector<T> topThickness;
  mapType &map;
  const lsSmartPointer<lsDomain<T, D>> passedLevelSet;

  double dot(double vector1[D], double vector2[D]) const {
    return (vector1[0] * vector2[0] + vector1[1] * vector2[1]);
  }

  double mag(double vector[D]) const {
    return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
  }

  T viewfactor(double point[D], double normal[D], double start[D],
               double end[D]) const {

    double tanStart[D] = {start[0] - point[0],
                          start[1] - point[1]}; // set point at 0,0
    double tanEnd[D] = {end[0] - point[0],
                        end[1] - point[1]}; // set point at 0,0

    if ((tanEnd[0] == 0) && (tanEnd[1] == 0))
      tanEnd[0] = -1.;

    double adjS = dot(
        normal, tanStart); // since cosX = dot(v1,v2)/mag(v1)*mag(v2) = adj/hyp
    double hypS = mag(normal) * mag(tanStart);
    double oppS = std::sqrt(hypS * hypS - adjS * adjS);

    double adjE = dot(normal, tanEnd);
    double hypE = mag(normal) * mag(tanEnd);
    double oppE = std::sqrt(hypE * hypE - adjE * adjE);

    double vf = 0.;

    if (tanStart[0] * tanEnd[0] < 0.) {
      double cosN = std::abs(normal[0]) /
                    std::sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
      vf = cosN - (oppS / hypS + oppE / hypE) / 2.;
    } else {
      vf = (oppE / hypE - oppS / hypS) / 2.;
    }

    return std::abs(vf);
  }

  T drdt(double t, double r, double radX, // double &radXRate,
         const std::array<hrleCoordType, 3> &initial,
         const std::array<hrleCoordType, 3> &normal) const {

    T vf = 0.;
    T d = data.trenchDiameter;           // start opening diameter
    double radY = data.topRate * t / 2.; // ellipse height radius
    double tanStart[D];                  // tangent on right ellipse
    double point[D];
    double norm[D];
    double tanEnd[D] = {-d / 2. + radX, radY + 1.};
    // Deal with the top points with vf=1 separately
    if (initial[1] > -0.1 * data.gridDelta) {
      norm[0] = -initial[0] / std::abs(initial[0]);
      norm[1] = 0.;
      point[0] = -d / 2. + r;
      point[1] = 0.;
    } else {
      norm[0] = normal[0];
      norm[1] = normal[1];
      point[0] = initial[0] + r * norm[0]; // X location of current point
      point[1] = initial[1] + r * norm[1]; // Y location of current point
    }
    tanStart[0] = d / 2. - radX;
    tanStart[1] = radY;
    tanEnd[0] = -d / 2. + radX;
    tanEnd[1] = radY;

    vf = viewfactor(point, norm, tanStart, tanEnd);
    return vf * data.topRate;
  }

  T getTopThickness(T curT) const {

    //    return curT*data.topRate;
    int cur = (int)(curT / data.timeStep + 0.5);
    return topThickness[cur];
  }

  T getThickness(const std::array<hrleCoordType, 3> &initial,
                 const std::array<hrleCoordType, 3> &normal) const {
    T curT = 0;
    T thickness = 0.;
    T delT = data.timeStep; // time step

    // Forward euler
    bool contains = false;
    {
      std::shared_lock<std::shared_mutex> r_lock(lock);
      auto it = map.find(initial);
      contains = it != map.end();
      if (contains)
        thickness = it->second;
    }
    if (!contains) {
      while (curT < data.processTime) {
        thickness +=
            delT *
            drdt(curT, thickness, getTopThickness(curT), initial, normal) *
            data.topRate;
        curT += delT;
      }
      std::unique_lock<std::shared_mutex> w_lock(lock);
      map.insert(std::make_pair(initial, thickness));
    }

    return thickness;
  }

  ViewFactorDistribution(ViewFactorProcessDataType<T> &processData,
                         mapType &map,
                         const lsSmartPointer<lsDomain<T, D>> passedLevelSet)
      : data(processData), closingTime(data.trenchDiameter / data.topRate),
        map(map), passedLevelSet(passedLevelSet) {

    std::cout << "Process time = " << data.processTime << "s\n";
    std::cout << "Closing time = " << closingTime << "s\n";

    int timeSize = data.processTime / data.timeStep + 1.5;
    topThickness.resize(timeSize);

    T vf = 0.5; // + data.sidewallNormal[1];
    T rate = vf * data.topRate;
    topThickness[0] = 0.;
    T normTop[D] = {1., 0};
    T d = data.trenchDiameter;
    T radX = 0;
    // i = curT/timeStep -> timeStep = curT/topThickness.size() -> curT =
    // i*timeStep
    for (int i = 1; i < topThickness.size(); ++i) {
      T curT = i * data.timeStep;
      T radX = topThickness[i - 1] + rate * data.timeStep;
      T radY = data.topRate * curT / 2.;
      T point[D] = {-d / 2. + radX, radY};
      T tanStart[D] = {d - 2. * radX, 2. * radY};
      T tanEnd[D] = {-d / 2. + radX, radY + 1.};
      vf = viewfactor(point, normTop, tanStart, tanEnd);
      rate = vf * data.topRate;
      topThickness[i] = radX;
    }
  }

  bool isInside(const std::array<hrleCoordType, 3> &initial,
                const std::array<hrleCoordType, 3> &candidate,
                double eps = 0.) const {
    hrleCoordType dot = 0.;
    for (unsigned i = 0; i < D; ++i) {
      double tmp = candidate[i] - initial[i];
      dot += tmp * tmp;
    }

    if (std::sqrt(dot) <= std::abs(data.topRate * data.processTime) + eps)
      return true;
    else
      return false;
  }

  T getSignedDistance(const std::array<hrleCoordType, 3> &initial,
                      const std::array<hrleCoordType, 3> &candidate,
                      unsigned long initialPointId) const {

    const auto &normal = (*(passedLevelSet->getPointData().getVectorData(
        lsCalculateNormalVectors<T, D>::normalVectorsLabel)))[initialPointId];

    // T topTh = data.topRate * data.processTime / 2.;
    T topTh = topThickness[topThickness.size() - 1];
    T radius = getThickness(initial, normal);
    T radius2 = radius * radius;

    T distance = std::numeric_limits<T>::max();
    std::array<hrleCoordType, D> v;
    for (unsigned i = 0; i < D; ++i)
      v[i] = candidate[i] - initial[i];

    T ellipseTop = data.topRate * data.processTime;
    T ellipseMid = data.topRate * data.processTime / 2.;

    if (std::abs(radius) <= data.gridDelta) {
      distance = std::max(std::abs(v[0]), std::abs(v[1])) - std::abs(radius);
    } else if (initial[1] > -0.1 * data.gridDelta) { // Top
      T distX = std::numeric_limits<T>::max();
      T distY = std::numeric_limits<T>::max();

      if (v[1] > ellipseMid) { // Top ellipse
        T x = v[0];
        T y = v[1] - ellipseMid;
        if (std::abs(x) <= topTh)
          distY = std::abs(y) -
                  ellipseMid * std::sqrt((1. - x * x / (topTh * topTh)));
        if (std::abs(y) <= ellipseMid)
          distX = std::abs(x) -
                  topTh * std::sqrt((1. - y * y / (ellipseMid * ellipseMid)));
        distance = (std::abs(distY) < std::abs(distX)) ? distY : distX;
      } else if (v[1] >= 0.) { // Tangent between top and bottom ellipse
        distance =
            std::abs(v[0]) - (radius + v[1] * (topTh - radius) / ellipseMid);
      } else { // Bottom ellipse
        T x = v[0];
        T y = v[1];
        if (std::abs(x) <= radius)
          distY = std::abs(y) -
                  ellipseMid * std::sqrt((1. - x * x / (radius * radius / 4)));
        if (std::abs(y) <= ellipseMid)
          distX = std::abs(x) -
                  radius * std::sqrt((1. - y * y / (ellipseMid * ellipseMid)));
        distance = (std::abs(distY) < std::abs(distX)) ? distY : distX;
      }
    } else { // Sidewall
      for (unsigned i = 0; i < D; ++i) {
        T y = (v[(i + 1) % D]);
        T z = 0;
        if (D == 3)
          z = (v[(i + 2) % D]);
        T x = radius2 - y * y - z * z;
        if (x < 0.)
          continue;
        T dirRadius = std::abs(v[i]) - std::sqrt(x);
        if (std::abs(dirRadius) < std::abs(distance))
          distance = dirRadius;
      }
    }
    return distance;
  }

  std::array<hrleCoordType, 6> getBounds() const {
    std::array<hrleCoordType, 6> bounds{};
    for (unsigned i = 0; i < D - 1; ++i) {
      bounds[2 * i] = -data.topRate * data.processTime;
      bounds[2 * i + 1] = data.topRate * data.processTime;
    }
    bounds[2 * (D - 1)] = -data.topRate * data.processTime;
    bounds[2 * D - 1] = data.topRate * data.processTime;
    return bounds;
  }
};
