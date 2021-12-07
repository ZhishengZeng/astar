/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-07 22:33:35
 * @FilePath: /astar/include/Model.h
 */

#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#include <assert.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <vector>

#include "Config.h"
#include "Direction.h"
#include "GridMap.h"
#include "Node.h"

#define LENGTH_UNIT 1
#define COST_UNIT 1
#define CORNER_UNIT 1

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
  std::vector<Coordinate> _offset_list;

  void init(const Coordinate& start_coord, const Coordinate& end_coord);
  void initGridMap();
  void addStartNodeToGridMap(const Coordinate& coord);
  void addEndNodeToGridMap(const Coordinate& coord);
  void addCostToGridMap();
  void addObsToGridMap();
  void initStartNode();
  void updateEstCost(Node* node);
  void updateOpen(Node* node);
  double caculateEstCost(Node* a, Node* b);
  void initOffsetList();
  void updatePathHead();
  void expandSearching();
  std::vector<Node*> getNeighborsByPathHead();
  bool touchByHead(Node* node);
  bool needReplaceParentNode(Node* node);
  double getCertSumByHead(Node* node);
  void updateParentByPathHead(Node* node);
  void reportResult();
  void printResult();
  std::vector<Coordinate> getCoordPath();
  std::vector<Coordinate> getFinalInflectionPath();
  Direction getDirection(Node* start_node, Node* end_node);
  bool isHorizontal(Coordinate& start_coord, Coordinate& end_coord);
  bool isVertical(Coordinate& start_coord, Coordinate& end_coord);
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
