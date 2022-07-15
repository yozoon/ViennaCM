#ifndef CM_GRAPH_READER_HPP
#define CM_GRAPH_READER_HPP

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>
#include <lsVTKWriter.hpp>

#ifdef WITH_MSGPACK
#include <msgpack.hpp>
#ifdef WITH_GZIP
#include <cmGzipUtils.hpp>
#endif
#endif

#include <cmFileFormats.hpp>
#include <cmGraphData.hpp>

template <class NumericType> class cmGraphReader {
  lsSmartPointer<cmGraphData<NumericType>> mGraphData = nullptr;
  std::string fileName;
  cmFileFormatEnum fileFormat = cmFileFormatEnum::AUTO;

public:
  cmGraphReader() {}

  cmGraphReader(lsSmartPointer<cmGraphData<NumericType>> pGraphData)
      : mGraphData(pGraphData) {}

  cmGraphReader(lsSmartPointer<cmGraphData<NumericType>> pGraphData,
                std::string passedFileName)
      : mGraphData(pGraphData), fileName(passedFileName) {}

  cmGraphReader(lsSmartPointer<cmGraphData<NumericType>> pGraphData,
                cmFileFormatEnum pFileFormat, std::string passedFileName)
      : mGraphData(pGraphData), fileName(passedFileName),
        fileFormat(pFileFormat) {}

  void setGraph(lsSmartPointer<cmGraphData<NumericType>> pGraphData) {
    mGraphData = pGraphData;
  }

  /// set file name for file to write
  void setFileName(std::string pFileName) { fileName = pFileName; }

  void setFileFormat(cmFileFormatEnum pFileFormat) { fileFormat = pFileFormat; }

#ifdef WITH_MSGPACK
  void readMsgpack(bool compress) {
    std::vector<char> buffer;
    std::ifstream ifstr(fileName, std::ios::binary);
    if (!compress) {
      // Uncompressed read
      if (!ifstr.is_open()) {
        std::cerr << "Error reading file." << std::endl;
        return;
      }
      ifstr.seekg(0, std::ios::end);
      size_t inputSize = ifstr.tellg();
      ifstr.seekg(0, std::ios::beg);
      buffer.resize(inputSize);
      ifstr.read(&buffer[0], inputSize);
    }
#ifdef WITH_GZIP
    else {
      // Read compressed file
      readCompressed(ifstr, std::back_inserter(buffer));
    }
#else
    else {
      return;
    }
#endif

    // deserialize it.
    msgpack::object_handle oh = msgpack::unpack(buffer.data(), buffer.size());

    // print the deserialized object.
    msgpack::object obj = oh.get();

    if (obj.type != msgpack::type::MAP) {
      std::cerr
          << "Provided msgpack file does not appear to be a valid graph file."
          << std::endl;
      return;
    }

    auto root = obj.as<std::unordered_map<std::string, msgpack::object>>();

    // Clear the content of the provided graph data instance
    mGraphData->clear();

    // Load node positions and data
    auto nodesMap =
        root["nodes"].as<std::unordered_map<std::string, msgpack::object>>();

    // Set node positions
    auto nodes = nodesMap["pos"].as<std::vector<std::array<NumericType, 3>>>();
    mGraphData->setNodes(nodes);

    // Set node Data
    size_t numNodeData = nodesMap.size() - 1;
    mGraphData->setNumberOfNodeData(numNodeData);

    unsigned i = 0;
    for (auto &[k, v] : nodesMap) {
      if (k == std::string("pos"))
        continue;
      // We need to copy the object instead of using a reference before we can
      // convert it
      msgpack::object obj = v;
      auto data = obj.as<std::vector<NumericType>>();
      mGraphData->setNodeData(i, std::move(data), k);
      ++i;
    }

    // Load edges and edge data
    auto edgesMap =
        root["edges"].as<std::unordered_map<std::string, msgpack::object>>();

    // Set edges
    auto from = edgesMap["from"].as<std::vector<long>>();
    auto to = edgesMap["to"].as<std::vector<long>>();

    assert(from.size() == to.size() &&
           "Edge `from` and to `array` sizes dont match.");

    std::vector<long> edges;
    edges.reserve(from.size() * 2);

    // Read the edges into the edges array in a strided format
    for (unsigned i = 0; i < from.size(); ++i) {
      edges.push_back(from[i]);
      edges.push_back(to[i]);
    }
    mGraphData->setEdges(std::move(edges));

    // Set edge data
    size_t numEdgeData = edgesMap.size() - 2;
    mGraphData->setNumberOfEdgeData(numEdgeData);

    i = 0;
    for (auto &[k, v] : edgesMap) {
      if (k == std::string("from") || k == std::string("to"))
        continue;
      // We need to copy the object instead of using a reference before we can
      // convert it
      msgpack::object obj = v;
      auto data = obj.as<std::vector<NumericType>>();
      mGraphData->setEdgeData(i, std::move(data), k);
      ++i;
    }
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
                          "passed to cmGraphReader. Not reading.")
              .print();
          return;
        }
      }
    }

    switch (fileFormat) {
#ifdef WITH_MSGPACK
    case cmFileFormatEnum::MSGPACK:
      readMsgpack(false);
      break;
#ifdef WITH_GZIP
    case cmFileFormatEnum::MSGPACK_GZIP:
      readMsgpack(true);
      break;
#else
    case cmFileFormatEnum::MSGPACK_GZIP:
      lsMessage::getInstance()
          .addWarning(
              "cmGraphReader was built without GZIP support. Not reading.")
          .print();
      break;
#endif
#else
    case cmFileFormatEnum::MSGPACK:
    case cmFileFormatEnum::MSGPACK_GZIP:
#endif
    case cmFileFormatEnum::VTK_LEGACY:
    case cmFileFormatEnum::VTP:
    case cmFileFormatEnum::MULTI:
      lsMessage::getInstance()
          .addWarning("cmGraphReader cannot yet read the provided file format. "
                      "Not reading.")
          .print();
      break;
    default:
      lsMessage::getInstance()
          .addWarning(
              "No valid file format set for cmGraphReader. Not reading.")
          .print();
    }
  }
};

#endif