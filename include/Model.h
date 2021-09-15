/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 14:24:17
 * @FilePath: /AStar/include/Model.h
 */

#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <vector>

#include "Direction.h"
#include "GridMap.h"
#include "Node.h"

#define SHOWRESULT 1
#define DIAGONAL 0

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

  void setMapSize(int x_size, int y_size);
  void setObstacle(const std::vector<Coordinate>& obs_coord_list);
  Node* setNode(const Coordinate& coord, const NodeType& node_type);
  void setRoutingMode(bool routing_diagonal, bool turning_back);
  std::vector<Coordinate> findPath(const Coordinate& start_coord,
                                   const Coordinate& end_coord);
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

 private:
  GridMap<Node> _grid_map;
  std::priority_queue<Node*, std::vector<Node*>, cmpNodeCost> _open_list;
  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _curr_node = nullptr;
  std::vector<Coordinate> _offset_list;
  bool _routing_diagonal;
  bool _turning_back;
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
