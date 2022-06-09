#pragma once

#include <array>

template <class T> struct ViewFactorProcessDataType {
  double gridDelta = 0.;
  T trenchDiameter;
  T trenchDepth;
  T topRate;
  T processTime;
  T timeStep;
  hrleVectorType<double, 2> sidewallNormal;
};