/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-03 21:18:27
 * @FilePath: /astar/include/Model.h
 */

#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
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
    _path_head_node = nullptr;
  }

  void buildMap(int x_size, int y_size);
  void addNodeCost(const Coordinate& coord, const double cost);
  void addObstacle(const Coordinate& coord, ObsType type);
  void setLogVerbose(int level);
  void enableTurningBack();
  void disableTurningBack();
  std::vector<Coordinate> getPath(const Coordinate& start_coord, const Coordinate& end_coord);

 private:
  Config _config;
  // database
  GridMap<Node> _grid_map;
  std::priority_queue<Node*, std::vector<Node*>, cmpNodeCost> _open_queue;

  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _path_head_node = nullptr;
  // object
  double _min_node_cost = 0;
  std::vector<Coordinate> _offset_list;

  void init(const Coordinate& start_coord, const Coordinate& end_coord);
  void initGridMap();
  void addStartNodeToGridMap(const Coordinate& coord);
  void addEndNodeToGridMap(const Coordinate& coord);
  void addCostToGridMap();
  void legalizeCost();
  void addObsToGridMap();
  void initStartNode();
  void updateEstCost(Node* node);
  double caculateEstCost(Node* a, Node* b);
  void initOffsetList();
  void updatePathHead();
  void expandSearching();
  std::vector<Node*> getNeighborsByPathHead();
  bool isInvaild(Node* node);
  bool needReplaceParentNode(Node* node);
  void updateParentByPathHead(Node* node);
  void reportResult();
  std::vector<Coordinate> getPathCoord();
  Direction getDirection(Node* start_node, Node* end_node);
  bool isHorizontal(Coordinate& start_coord, Coordinate& end_coord);
  bool isVertical(Coordinate& start_coord, Coordinate& end_coord);
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
