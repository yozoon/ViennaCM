#pragma once

#include <array>
#include <unordered_map>

#include <lsDomain.hpp>
#include <lsSmartPointer.hpp>

#include "cmVectorHash.hpp"

#include "ViewFactorDistribution.hpp"
#include "ViewFactorProcessData.hpp"

template <class T, int D> class ViewFactorProcess {
  lsSmartPointer<lsDomain<T, D>> substrate;
  ViewFactorProcessDataType<T> processData;

public:
  ViewFactorProcess() {}

  ViewFactorProcess(lsSmartPointer<lsDomain<T, D>> passedSubstrate)
      : substrate(passedSubstrate) {
    processData.gridDelta = substrate->getGrid().getGridDelta();
  }

  void setSubstrate(lsSmartPointer<lsDomain<T, D>> levelSet) {
    substrate = levelSet;
    processData.gridDelta = substrate->getGrid().getGridDelta();
  }

  void setTrenchDiameter(T trenchDiameter) {
    processData.trenchDiameter = trenchDiameter;
  }

  void setTrenchDepth(T trenchDepth) { processData.trenchDepth = trenchDepth; }

  void setSidewallNormal(hrleVectorType<double, 2> sidewallNormal) {
    processData.sidewallNormal = sidewallNormal;
  }

  void setTopRate(T topRate) { processData.topRate = topRate; }

  void setProcessTime(T processTime) { processData.processTime = processTime; }

  void setTimeStep(T timeStep) { processData.timeStep = timeStep; }

  void apply() {
    using mapType =
        std::unordered_map<std::array<hrleCoordType, 3>, double,
                           cmVectorHash<std::array<hrleCoordType, 3>>>;
    mapType map;

    auto dist = lsSmartPointer<ViewFactorDistribution<T, D, mapType>>::New(
        processData, map, substrate);
    lsGeometricAdvect<T, D, true>(substrate, dist).apply();
  }
};