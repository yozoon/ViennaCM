#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>
#include <lsVTKWriter.hpp>

#ifdef WITH_MSGPACK
#include <msgpack.hpp>
#ifdef WITH_GZIP
#include <cmGzipStreams.hpp>
#endif
#endif

#include <cmGraphData.hpp>

enum cmFileFormatEnum : unsigned {
  VTK_LEGACY = 0,
  VTP = 1,
  MULTI = 2,
  MSGPACK = 3,
  MSGPACK_GZIP = 4,
  AUTO = 5,
};

template <class NumericType> class cmGraphWriter {
  lsSmartPointer<cmGraphData<NumericType>> mGraphData = nullptr;
  std::string fileName;
  cmFileFormatEnum fileFormat = cmFileFormatEnum::AUTO;

public:
  cmGraphWriter() {}

  cmGraphWriter(lsSmartPointer<cmGraphData<NumericType>> pGraphData)
      : mGraphData(pGraphData) {}

  cmGraphWriter(lsSmartPointer<cmGraphData<NumericType>> pGraphData,
                std::string passedFileName)
      : mGraphData(pGraphData), fileName(passedFileName) {}

  cmGraphWriter(lsSmartPointer<cmGraphData<NumericType>> pGraphData,
                cmFileFormatEnum pFileFormat, std::string passedFileName)
      : mGraphData(pGraphData), fileName(passedFileName),
        fileFormat(pFileFormat) {}

  void setGraph(lsSmartPointer<cmGraphData<NumericType>> pGraphData) {
    mGraphData = pGraphData;
  }

  /// set file name for file to write
  void setFileName(std::string pFileName) { fileName = pFileName; }

  void setFileFormat(cmFileFormatEnum pFileFormat) { fileFormat = pFileFormat; }

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
    switch (fileFormat) {
    case cmFileFormatEnum::VTK_LEGACY:
      lsVTKWriter<NumericType>(mesh, lsFileFormatEnum::VTK_LEGACY, fileName)
          .apply();
      break;
    case cmFileFormatEnum::VTP:
      lsVTKWriter<NumericType>(mesh, lsFileFormatEnum::VTP, fileName).apply();
      break;

    default:
      break;
    }
  }

  void writeMulti() {
    std::ofstream nodeFile(fileName + ".nodes");
    if (!nodeFile.is_open()) {
      lsMessage::getInstance()
          .addWarning("Couldn't open node file for writing.")
          .print();
      return;
    }

    const auto &nodes = mGraphData->getNodes();
    auto &nodeData = mGraphData->getNodeData();
    std::vector<typename std::remove_reference<
        decltype(nodeData)>::type::value_type::iterator>
        nodeDataIterators;

    for (auto &nd : nodeData) {
      nodeDataIterators.push_back(std::begin(nd));
    }

    // Write header
    nodeFile << "x,y,z";
    for (unsigned i = 0; i < nodeData.size(); ++i)
      nodeFile << ',' << mGraphData->getNodeDataLabel(i);
    nodeFile << '\n';

    // Write content
    for (unsigned i = 0; i < nodes.size(); ++i) {
      // Node coordinates
      auto &node = nodes[i];
      nodeFile << static_cast<float>(node[0]) << ','
               << static_cast<float>(node[1]) << ','
               << static_cast<float>(node[2]);
      // Node data
      if (!nodeDataIterators.empty())
        for (auto &it : nodeDataIterators) {
          nodeFile << ',' << static_cast<float>(*it);
          ++it;
        }
      nodeFile << '\n';
    }
    nodeFile.close();

    std::ofstream edgeFile(fileName + ".edges");
    if (!edgeFile.is_open()) {
      lsMessage::getInstance()
          .addWarning("Couldn't open edge file for writing.")
          .print();
      return;
    }

    const auto &edges = mGraphData->getEdges();
    auto &edgeData = mGraphData->getEdgeData();
    std::vector<typename std::remove_reference<
        decltype(edgeData)>::type::value_type::iterator>
        edgeDataIterators;

    for (auto &ed : edgeData) {
      edgeDataIterators.push_back(std::begin(ed));
    }

    // Write header
    edgeFile << "from,to";
    for (unsigned i = 0; i < edgeData.size(); ++i)
      edgeFile << ',' << mGraphData->getEdgeDataLabel(i);
    edgeFile << '\n';

    // Write content
    for (unsigned i = 0; i < edges.size() / 2; ++i) {
      // From and to
      edgeFile << edges[2 * i] << "," << edges[2 * i + 1];
      // Edge data
      if (!edgeDataIterators.empty())
        for (auto &it : edgeDataIterators) {
          edgeFile << ',' << static_cast<float>(*it);
          ++it;
        }
      edgeFile << '\n';
    }
    edgeFile.close();
  }

#ifdef WITH_MSGPACK
  void writeMsgpack(bool compress) {
    msgpack::sbuffer buffer;

    msgpack::packer<msgpack::sbuffer> pk(&buffer);
    // Two top level keys: nodes and edges
    pk.pack_map(2);

    // 1: Nodes
    pk.pack(std::string("nodes"));

    const auto &nodes = mGraphData->getNodes();
    auto &nodeData = mGraphData->getNodeData();

    // Positions array
    pk.pack_map(1 + nodeData.size());
    pk.pack(std::string("pos"));
    pk.pack(nodes);

    // Node Data
    for (unsigned i = 0; i < nodeData.size(); ++i) {
      pk.pack(mGraphData->getNodeDataLabel(i));
      pk.pack(nodeData[i]);
    }

    // 2: Edges
    pk.pack(std::string("edges"));

    const auto &edges = mGraphData->getEdges();
    auto &edgeData = mGraphData->getEdgeData();

    // Connectivity
    pk.pack_map(2 + edgeData.size());
    pk.pack(std::string("from"));
    pk.pack_array(edges.size() / 2);
    for (unsigned i = 0; i < edges.size() / 2; ++i) {
      pk.pack(edges[2 * i]);
    }
    pk.pack(std::string("to"));
    pk.pack_array(edges.size() / 2);
    for (unsigned i = 0; i < edges.size() / 2; ++i) {
      pk.pack(edges[2 * i + 1]);
    }

    // Edge Data
    for (unsigned i = 0; i < edgeData.size(); ++i) {
      pk.pack(mGraphData->getEdgeDataLabel(i));
      pk.pack(edgeData[i]);
    }

    if (!compress) {
      // Uncompressed write
      std::ofstream of(fileName);
      of.write(buffer.data(), buffer.size());
    }
#ifdef WITH_GZIP
    else {
      // Write using gzip compression
      gzip_ofstream gzip(fileName);
      gzip.write(buffer.data(), buffer.size());
    }
#endif
  }
#endif

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

    if (fileFormat == cmFileFormatEnum::AUTO) {
      auto dotPos = fileName.rfind('.');
      if (dotPos == std::string::npos) {
        fileFormat = cmFileFormatEnum::MULTI;
      } else {
        auto ending = fileName.substr(dotPos);
        if (ending == ".vtk") {
          fileFormat = cmFileFormatEnum::VTK_LEGACY;
        } else if (ending == ".vtp") {
          fileFormat = cmFileFormatEnum::VTP;
        } else if (ending == ".msgpack") {
          fileFormat = cmFileFormatEnum::MSGPACK;
        } else if (ending == ".gz") {
          fileFormat = cmFileFormatEnum::MSGPACK_GZIP;
        } else {
          lsMessage::getInstance()
              .addWarning("No valid file format found based on the file ending "
                          "passed to lsVTKWriter. Not writing.")
              .print();
          return;
        }
      }
    }

    switch (fileFormat) {
    case cmFileFormatEnum::VTK_LEGACY:
    case cmFileFormatEnum::VTP:
      writeVTK();
      break;
    case cmFileFormatEnum::MULTI:
      writeMulti();
      break;
#ifdef WITH_MSGPACK
    case cmFileFormatEnum::MSGPACK:
      writeMsgpack(false);
      break;
#ifdef WITH_GZIP
    case cmFileFormatEnum::MSGPACK_GZIP:
      writeMsgpack(true);
      break;
#else
    case cmFileFormatEnum::MSGPACK_GZIP:
      lsMessage::getInstance()
          .addWarning(
              "cmGraphWriter was built without GZIP support. Falling back "
              "to uncompressed MSGPACK.")
          .print();
      writeMsgpack(false);
      break;
#endif
#else
    case cmFileFormatEnum::MSGPACK:
    case cmFileFormatEnum::MSGPACK_GZIP:
#endif
    default:
      lsMessage::getInstance()
          .addWarning(
              "No valid file format set for cmGraphWriter. Not writing.")
          .print();
    }
  }
};