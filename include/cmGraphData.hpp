#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

template <typename NumericType> class cmGraphData {
private:
  using vectorDataType = std::vector<NumericType>;

  std::vector<std::array<NumericType, 3>> nodes;
  std::vector<long> edges;

  std::vector<vectorDataType> nodeData;
  std::vector<std::string> nodeDataLabels;
  std::vector<vectorDataType> edgeData;
  std::vector<std::string> edgeDataLabels;

public:
  cmGraphData() {}

  cmGraphData(const cmGraphData &otherData)
      : nodes(otherData.nodes), edges(otherData.edges),
        nodeData(otherData.nodeData), edgeData(otherData.edgeData),
        nodeDataLabels(otherData.nodeDataLabels),
        edgeDataLabels(otherData.edgeDataLabels) {}

  cmGraphData(cmGraphData &&otherData)
      : nodes(std::move(otherData.nodes)), edges(std::move(otherData.edges)),
        nodeData(std::move(otherData.nodeData)),
        edgeData(std::move(otherData.edgeData)),
        nodeDataLabels(std::move(otherData.nodeDataLabels)),
        edgeDataLabels(std::move(otherData.edgeDataLabels)) {}

  // Assignment operators

  cmGraphData &operator=(const cmGraphData &otherData) {
    nodes = otherData.nodes;
    edges = otherData.edges;

    nodeData = otherData.nodeData;
    edgeData = otherData.edgeData;
    nodeDataLabels = otherData.nodeDataLabels;
    edgeDataLabels = otherData.edgeDataLabels;
    return *this;
  }

  cmGraphData &operator=(cmGraphData &&otherData) {
    nodes = std::move(otherData.nodes);
    edges = std::move(otherData.edges);

    nodeData = std::move(otherData.nodeData);
    edgeData = std::move(otherData.edgeData);
    nodeDataLabels = std::move(otherData.nodeDataLabels);
    edgeDataLabels = std::move(otherData.edgeDataLabels);
    return *this;
  }

  // Edge and Node setters

  void setNodes(std::vector<std::array<NumericType, 3>> &n) { nodes = n; }

  void setNodes(std::vector<std::array<NumericType, 3>> &&n) {
    nodes = std::move(n);
  }

  void setEdges(std::vector<long> &e) { edges = e; }

  void setEdges(std::vector<long> &&e) { edges = std::move(e); }

  // Appending operations

  void appendNodes(const std::vector<std::array<NumericType, 3>> &vec) {
    nodes.insert(nodes.end(), vec.begin(), vec.end());
  }

  void appendEdges(const std::vector<long> &vec) {
    edges.insert(edges.end(), vec.begin(), vec.end());
  }

  void appendNodeData(int num, const std::vector<NumericType> &vec) {
    nodeData[num].insert(nodeData[num].end(), vec.begin(), vec.end());
  }

  void appendEdgeData(int num, const std::vector<NumericType> &vec) {
    edgeData[num].insert(edgeData[num].end(), vec.begin(), vec.end());
  }

  void clear() {
    nodes.clear();
    edges.clear();
    nodeData.clear();
    nodeDataLabels.clear();
    edgeData.clear();
    edgeDataLabels.clear();
  }

  // Add edge
  void addEdge(long i, long j) {
    edges.push_back(i);
    edges.push_back(j);
  }

  // Edge data push_back
  void pushBackEdgeData(int num, NumericType value) {
    edgeData[num].push_back(value);
  }

  // Node and Edge data size initialization

  void setNumberOfNodeData(int size) {
    nodeData.clear();
    nodeData.resize(size);
    nodeDataLabels.resize(size, "nodeData");
  }

  void setNumberOfEdgeData(int size) {
    edgeData.clear();
    edgeData.resize(size);
    edgeDataLabels.resize(size, "edgeData");
  }

  // Node data setters

  void setNodeData(int num, std::vector<NumericType> &vector,
                   std::string label = "nodeData") {
    if (num >= nodeData.size())
      lsMessage::getInstance()
          .addError("Setting node data in cmGraphData out of range.")
          .print();
    nodeData[num] = vector;
    nodeDataLabels[num] = label;
  }

  void setNodeData(int num, std::vector<NumericType> &&vector,
                   std::string label = "nodeData") {
    if (num >= nodeData.size())
      lsMessage::getInstance()
          .addError("Setting node data in cmGraphData out of range.")
          .print();
    nodeData[num] = std::move(vector);
    nodeDataLabels[num] = label;
  }

  void setNodeData(int num, size_t size, NumericType value,
                   std::string label = "nodeData") {
    if (num >= nodeData.size())
      lsMessage::getInstance()
          .addError("Setting node data in cmGraphData out of range.")
          .print();
    nodeData[num].resize(size, value);
    nodeDataLabels[num] = label;
  }

  // Edge data setters

  void setEdgeData(int num, std::vector<NumericType> &vector,
                   std::string label = "edgeData") {
    if (num >= edgeData.size())
      lsMessage::getInstance()
          .addError("Setting edge data in cmGraphData out of range.")
          .print();
    edgeData[num] = vector;
    edgeDataLabels[num] = label;
  }

  void setEdgeData(int num, std::vector<NumericType> &&vector,
                   std::string label = "edgeData") {
    if (num >= edgeData.size())
      lsMessage::getInstance()
          .addError("Setting edge data in cmGraphData out of range.")
          .print();
    edgeData[num] = std::move(vector);
    edgeDataLabels[num] = label;
  }

  // Node getters

  std::vector<std::array<NumericType, 3>> &getNodes() { return nodes; }

  const std::vector<std::array<NumericType, 3>> &getNodes() const {
    return nodes;
  }

  // Edge getters

  std::vector<long> &getEdges() { return edges; }

  const std::vector<long> &getEdges() const { return edges; }

  // Node data getters

  vectorDataType &getNodeData(int i) { return nodeData[i]; }

  const vectorDataType &getNodeData(int i) const { return nodeData[i]; }

  vectorDataType &getNodeData(std::string label) {
    int idx = getNodeDataIndex(label);
    return nodeData[idx];
  }

  std::vector<vectorDataType> &getNodeData() { return nodeData; }

  const std::vector<vectorDataType> &getNodeData() const { return nodeData; }

  // Edge data getters

  vectorDataType &getEdgeData(int i) { return edgeData[i]; }

  const vectorDataType &getEdgeData(int i) const { return edgeData[i]; }

  vectorDataType &getEdgeData(std::string label) {
    int idx = getEdgeDataIndex(label);
    return edgeData[idx];
  }

  std::vector<vectorDataType> &getEdgeData() { return edgeData; }

  const std::vector<vectorDataType> &getEdgeData() const { return edgeData; }

  // Node Label / Index lookup

  std::string getNodeDataLabel(int i) const {
    if (i >= nodeDataLabels.size())
      lsMessage::getInstance()
          .addError("Getting node data label in cmGraphData out of range.")
          .print();
    return nodeDataLabels[i];
  }

  int getNodeDataIndex(std::string label) {
    for (int i = 0; i < nodeDataLabels.size(); ++i) {
      if (nodeDataLabels[i] == label) {
        return i;
      }
    }
    lsMessage::getInstance()
        .addError("Can not find node data label in cmGraphData.")
        .print();
    return -1;
  }

  // Edge Label / Index lookup

  std::string getEdgeDataLabel(int i) const {
    if (i >= edgeDataLabels.size())
      lsMessage::getInstance()
          .addError("Getting edge data label in cmGraphData out of range.")
          .print();
    return edgeDataLabels[i];
  }

  int getEdgeDataIndex(std::string label) {
    for (int i = 0; i < edgeDataLabels.size(); ++i) {
      if (edgeDataLabels[i] == label) {
        return i;
      }
    }
    lsMessage::getInstance()
        .addError("Can not find edge data label in cmGraphData.")
        .print();
    return -1;
  }
};