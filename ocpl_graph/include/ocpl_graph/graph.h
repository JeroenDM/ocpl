#pragma once

#include <limits>
#include <vector>
#include <memory>

namespace ocpl
{

const double INF = std::numeric_limits<double>::infinity();

typedef std::vector<double> JointPositions;
typedef std::vector<std::vector<JointPositions>> GraphData;

struct Node
{
  int path_index;
  int sample_index;
  JointPositions* jv = nullptr;
  double shortest_distance = INF;
  Node* parent = nullptr;
  bool visited = false;
};

class Graph
{
  GraphData raw_data_;
  std::vector<std::vector<Node>> nodes_;
  int num_path_points_ = 0;
  bool shortest_path_is_found = false;

public:
  Graph() = default;
  Graph(std::vector<std::vector<JointPositions>>& path_joint_poses);
  ~Graph() = default;

  double last_path_cost;
  void addPathPointData(std::vector<JointPositions>& joint_poses);
  const std::vector<std::vector<Node>>& getNodes() const;
  std::vector<Node*> getNeighbors(Node* node);
  std::vector<Node*> getShortestPath();
  void runMultiSourceDijkstra();
};

struct sortNodesFunction
{
  bool operator()(Node* n1, Node* n2) const
  {
    return (*n2).shortest_distance < (*n1).shortest_distance;
  }
};

double L1NormCost(Node n1, Node n2);
double sumSquaredCost(Node n1, Node n2);

} // namespace ocpl
