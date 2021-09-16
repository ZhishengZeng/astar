/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-16 15:26:34
 * @FilePath: /AStar/include/Model.h
 */

#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "Direction.h"
#include "GridMap.h"
#include "Node.h"

#define SHOWRESULT 1

namespace astar {
class Model
{
 public:
  Model() {}
  ~Model()
  {
    _start_node = nullptr;
    _end_node = nullptr;
    _curr_node = nullptr;
  }

  void buildMap(int x_size, int y_size);
  void addNodeCost(const Coordinate& coord, const double cost);
  void addObstacle(const Coordinate& coord, const char type);
  void enableDiagonalRouting();
  void disableDiagonalRouting();
  void enableTurningBack();
  void disableTurningBack();
  std::vector<Coordinate> getPath(const Coordinate& start_coord, const Coordinate& end_coord);

 private:
  // input
  GridMap<Node> _grid_map;
  std::vector<std::pair<Coordinate, double>> _cost_list;
  std::vector<std::pair<Coordinate, NodeType>> _obs_list;
  bool _turning_back = true;
  bool _routing_diagonal = false;
  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _curr_node = nullptr;
  double _min_node_cost = 0;
  std::vector<Coordinate> _offset_list;
  std::priority_queue<Node*, std::vector<Node*>, cmpNodeCost> _open_list;

  void addCostToMap();
  void addObsToMap();
  Node* setNode(const Coordinate& coord, const NodeType& node_type);
  void setStartNode(const Coordinate& coord);
  void setEndNode(const Coordinate& coord);
  void initStartNode();
  void updateEstCost(Node* node);
  double caculateEstCost(Node* a, Node* b);
  void updateParentByCurr(Node* node);
  double getCurrWalkingCost(Node* node);
  void addNodeToOpenList(Node* node);
  void initOffsetList();
  void getMinCostNodeInOpenList();
  void addNeighborNodesToOpenList();
  void getNeighborNodesByCurr(std::vector<Node*>& neighbor_node_list);
  bool isLegalNeighbor(int x, int y);
  bool isCurrBetterParent(Node* node);
  void showResult();
  void setOnPath(const bool on_path);
  void printGridMap();
  void printNode(Node& node);
  void reportResult();
  std::vector<Coordinate> getPathCoord();
  Direction getDirection(Node* start_node, Node* end_node);
  bool isHorizontal(Coordinate& start_coord, Coordinate& end_coord);
  bool isVertical(Coordinate& start_coord, Coordinate& end_coord);
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
