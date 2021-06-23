/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 16:19:28
 * @Description:
 * @FilePath: /AStarextension-output-#3
 */
#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#define SHOWSTEPBYSTEP 0

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <vector>

#include "GridMap.h"
#include "Node.h"
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
    while (!_open_list.empty()) {
      Node* a = _open_list.top();
      _open_list.pop();
      delete a;
      a = nullptr;
    }
  }

  void setMapSize(int x_grid_num, int y_grid_num);
  Node* setNode(const Coordinate& coord, const NodeType& node_type);
  void setObstacle(const std::vector<Coordinate>& obs_coord_list);
  void findPath(const Coordinate& start_coord, const Coordinate& end_coord);
  void setStartNode(const Coordinate& coord);
  void setEndNode(const Coordinate& coord);
  void initStartNode();
  void updateEstCost(Node* node);
  void updateParentByCurr(Node* node);
  void addNodeToOpenList(Node* node);
  void getMinCostNodeInOpenList();
  bool reachEndNode();
  bool isNoWhere();
  void addNeighborNodesToOpenList();
  void getNeighborNodesByCurr(std::vector<Node*>& neighbor_node_list);
  bool currIsBetterThan(Node* node);
  void showResult();
  void setOnPath(const bool on_path);
  void printGridMap();

 private:
  GridMap<Node> _grid_map;
  std::priority_queue<Node*, std::vector<Node*>, cmpNode> _open_list;
  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _curr_node = nullptr;
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
