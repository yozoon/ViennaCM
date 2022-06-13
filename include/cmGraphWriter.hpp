#pragma once

#include <string>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>
#include <lsVTKWriter.hpp>

#include <cmGraphData.hpp>

template <class NumericType> class cmGraphWriter {
  lsSmartPointer<cmGraphData<NumericType>> mGraphData = nullptr;
  std::string fileName;

public:
  cmGraphWriter() {}

  cmGraphWriter(lsSmartPointer<cmGraphData<NumericType>> pGraphData)
      : mGraphData(pGraphData) {}

  cmGraphWriter(lsSmartPointer<cmGraphData<NumericType>> pGraphData,
                std::string passedFileName)
      : mGraphData(pGraphData), fileName(passedFileName) {}

  void setGraph(lsSmartPointer<cmGraphData<NumericType>> pGraphData) {
    mGraphData = pGraphData;
  }

  /// set file name for file to write
  void setFileName(std::string passedFileName) { fileName = passedFileName; }

  void writeVTK() {
    auto mesh = lsSmartPointer<lsMesh<NumericType>>::New();
    for (auto &node : mGraphData->getNodes()) {
      mesh->insertNextNode(node);
    }
    auto &edges = mGraphData->getEdges();
    for (unsigned int i = 0; i < edges.size(); i += 2) {
      std::array<unsigned, 2> line{static_cast<unsigned>(edges[i]),
                                   static_cast<unsigned>(edges[i + 1])};
      mesh->insertNextLine(line);
    }

    auto &nodeData = mGraphData->getNodeData();
    for (unsigned i = 0; i < nodeData.size(); ++i) {
      mesh->getPointData().insertNextScalarData(
          nodeData[i], mGraphData->getNodeDataLabel(i));
    }

    auto &edgeData = mGraphData->getEdgeData();
    for (unsigned i = 0; i < edgeData.size(); ++i) {
      mesh->getCellData().insertNextScalarData(edgeData[i],
                                               mGraphData->getEdgeDataLabel(i));
    }
    lsVTKWriter<NumericType>(mesh, fileName).apply();
  }

  void apply() {
    // check graph
    if (mGraphData == nullptr) {
      lsMessage::getInstance()
          .addWarning("No graph data was passed to cmGraphWriter. Not writing.")
          .print();
      return;
    }
    // check filename
    if (fileName.empty()) {
      lsMessage::getInstance()
          .addWarning("No file name specified for cmGraphWriter. Not writing.")
          .print();
      return;
    }

    writeVTK();
  }
};