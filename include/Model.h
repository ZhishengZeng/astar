/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 16:19:28
 * @Description:
 * @FilePath: /AStarextension-output-#3
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

#define SHOWSTEPBYSTEP 0
#define SHOWRESULT 1
#define DIAGONAL 0

namespace astar {
class Model
{
 public:
  Model() {}
  ~Model() { freeModel(); }

  void setMapSize(int x_size, int y_size);
  Node* setNode(const Coordinate& coord, const NodeType& node_type);
  void setObstacle(const std::vector<Coordinate>& obs_coord_list);
  std::vector<Coordinate> findPath(const Coordinate& start_coord,
                                   const Coordinate& end_coord);
  void setStartNode(const Coordinate& coord);
  void setEndNode(const Coordinate& coord);
  void initStartNode();
  void updateEstCost(Node* node);
  void updateParentByCurr(Node* node);
  double getCurrWalkingCost(Node* node);
  void addNodeToOpenList(Node* node);
  void getMinCostNodeInOpenList();
  bool reachEndNode();
  bool isNoWhere();
  void addNeighborNodesToOpenList();
  void getNeighborNodesByCurr(std::vector<Node*>& neighbor_node_list);
  bool isLegalNeighbor(int x, int y);
  bool isCurrBetterParent(Node* node);
  void showResult();
  void setOnPath(const bool on_path);
  void reportResult();
  void printGridMap();
  void freeModel();
  std::vector<Coordinate> getPathCoord();

 private:
  GridMap<Node> _grid_map;
  std::priority_queue<Node*, std::vector<Node*>, cmpNodeCost> _open_list;
  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _curr_node = nullptr;
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
