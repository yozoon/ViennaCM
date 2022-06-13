#pragma once

#include <string>

#include <lsSmartPointer.hpp>

#include <cmGraphData.hpp>

template <class NumericType, int D> class cmGraphWriter {
  lsSmartPointer<cmGraph> mGraph = nullptr;
  std::string baseName;

public:
  cmGraphWriter() {}

  cmGraphWriter(lsSmartPointer<cmGraph> pGraph) : mGraph(pGraph) {}

  cmGraphWriter(lsSmartPointer<cmGraph> pGraph, std::string passedBaseName)
      : mGraph(pGraph), baseName(passedBaseName) {}

  void setGraph(lsSmartPointer<cmGraph> pGraph) { mGraph = pGraph; }

  /// set file name for file to write
  void setFileName(std::string passedBaseName) { baseName = passedBaseName; }

  void apply() {
    // check graph
    if (graph == nullptr) {
      lsMessage::getInstance()
          .addWarning("No graph was passed to cmGraphWriter. Not writing.")
          .print();
      return;
    }
    // check filename
    if (baseName.empty()) {
      lsMessage::getInstance()
          .addWarning("No base name specified for cmGraphWriter. Not writing.")
          .print();
      return;
    }
  }
};