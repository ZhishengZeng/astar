/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-25 18:16:28
 * @FilePath: /astar/include/Model.h
 */

#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#include <assert.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <vector>

#include "Config.h"
#include "Direction2d.h"
#include "GridMap.h"
#include "Node.h"
#include "Orientation2d.h"

namespace astar {
class Model
{
 public:
  Model() {}
  ~Model()
  {
    _start = nullptr;
    _end = nullptr;
    _path_head = nullptr;
  }

  void buildMap(const int x_size, const int y_size);
  void addOBS(const Coordinate& coord, const Direction2d& direction_2d);
  void addCost(const Coordinate& coord, const Direction2d& direction_2d, const double cost);
  void setLogVerbose(const int level = 0);
  void enableTurningBack();
  void disableTurningBack();
  std::vector<Coordinate> getPath(const Coordinate& start_coord, const Coordinate& end_coord);

 private:
  Config _config;
  // database
  GridMap<Node> _grid_map;
  std::priority_queue<Node*, std::vector<Node*>, cmpNodeCost> _open_queue;

  Node* _start = nullptr;
  Node* _end = nullptr;
  Node* _path_head = nullptr;
  // object
  int _min_cost = COST_UNIT;
  std::vector<Coordinate> _offset_list;

  void init(const Coordinate& start_coord, const Coordinate& end_coord);
  void initGridMap();
  void addStartNodeToGridMap(const Coordinate& coord);
  void addEndNodeToGridMap(const Coordinate& coord);
  void addObsToGridMap();
  void legalizeCostMap();
  void addCostToGridMap();
  void initStartNode();
  void initOffsetList();
  void expandSearching();
  std::vector<Node*> getNeighborsByPathHead();
  bool replaceParentNode(Node* start, Node* end);
  void reportResult();
  std::vector<Coordinate> getFinalInflectionPath();
  // Plot;
  void plotResult();
  std::vector<Coordinate> getCoordPath();
  // Calculate known cost;
  double getKnowCost(Node* start, Node* end);
  double getKnowCornerCost(Node* start, Node* end);
  // Calculate estimate cost
  double getEstimateCost(Node* start, Node* end);
  double getEstimateCornerCost(Node* start, Node* end);
  std::vector<std::vector<Node*>> tryRouting(Node* start, Node* end);
  std::vector<std::vector<Node*>> routingStraight(Node* start, Node* end);
  bool passCheckingSegment(Node* start, Node* end);
  std::vector<std::vector<Node*>> routingLShape(Node* start, Node* end);
  int getMinCornerNum(std::vector<std::vector<Node*>>& coord_path_list);
  // base
  Direction2d getDirection2d(Node* start, Node* end);
  Orientation2d getOrientation2d(Node* start, Node* end);
  bool isStraight(Node* start, Node* end);
  bool isHorizontal(Node* start, Node* end);
  bool isVertical(Node* start, Node* end);
  bool isPoint(Node* start, Node* end);
  int getManhattanDistance(Node* start, Node* end);
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_
