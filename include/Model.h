/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 16:51:37
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

#include "Config.h"
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
    _optimal_node = nullptr;
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
  Config _config;
  // database
  GridMap<Node> _grid_map;
  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _optimal_node = nullptr;
  std::priority_queue<Node*, std::vector<Node*>, cmpNodeCost> _search_queue;
  // object
  double _min_node_cost = 0;
  std::vector<Coordinate> _offset_list;

  void init(const Coordinate& start_coord, const Coordinate& end_coord);
  void initGridMap();
  void addCostToGridMap();
  void addObsToGridMap();
  Node* setNode(const Coordinate& coord, const NodeType& node_type);
  void addStartNodeToGridMap(const Coordinate& coord);
  void initStartNode();
  void updateEstCost(Node* node);
  double caculateEstCost(Node* a, Node* b);
  void updateParentByOptimalNode(Node* node);
  double getWalkingCost(Node* node1, Node* node2);
  void addNodeToSearchQueue(Node* node);
  void addEndNodeToGridMap(const Coordinate& coord);
  void initOffsetList();
  void updateOptimalNode();
  void expandSearchQueue();
  std::vector<Node*> getNeighborsByOptimalNode();
  bool checkCoord(const int x, const int y);
  bool checkNode(const int x, const int y);
  bool needReplaceParentNode(Node* node);
  ////////////
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
