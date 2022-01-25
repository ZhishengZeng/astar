/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-25 18:18:18
 * @FilePath: /astar/include/Node.h
 */

#ifndef ASTAR_INCLUDE_NODE_H_
#define ASTAR_INCLUDE_NODE_H_

#include "Coordinate.h"
#include "ObsType.h"
#include "SearchState.h"

#define LENGTH_UNIT 1
#define COST_UNIT 0.001
#define CORNER_UNIT 1

namespace astar {

class Node
{
 public:
  Node() {}
  ~Node() {}
  // getter
  Coordinate& get_coord() { return _coord; }
  std::map<Direction2d, bool>& get_obs_map() { return _obs_map; }
  std::map<Direction2d, double>& get_cost_map() { return _cost_map; }
  Node* get_parent_node() { return _parent_node; }
  double get_known_cost() const { return _known_cost; }
  double get_estimated_cost() const { return _estimated_cost; }
  // setter
  void set_coord(const int x, const int y)
  {
    _coord.set_x(x);
    _coord.set_y(y);
  }
  void set_obs_map(const std::map<Direction2d, bool>& obs_map) { _obs_map = obs_map; }
  void set_cost_map(const std::map<Direction2d, double>& cost_map) { _cost_map = cost_map; }
  void set_search_state(SearchState search_state) { _search_state = search_state; }
  void set_parent_node(Node* parent_node) { _parent_node = parent_node; }
  void set_known_cost(const double known_cost) { _known_cost = known_cost; }
  void set_estimated_cost(const double estimated_cost) { _estimated_cost = estimated_cost; }

  // function
  bool isOBS(Direction2d direction_2d) { return _obs_map[direction_2d]; }
  double getCost(Direction2d direction_2d) { return _cost_map[direction_2d]; }
  bool isOpen() { return _search_state == SearchState::kOpen; }
  bool isClose() { return _search_state == SearchState::kClose; }
  double getTotalCost() { return (_known_cost + _estimated_cost); }

 private:
  Coordinate _coord;
  std::map<Direction2d, bool> _obs_map;
  std::map<Direction2d, double> _cost_map;
  SearchState _search_state = SearchState::kNone;
  Node* _parent_node = nullptr;
  // the known cost of this_node start start (include self)
  double _known_cost = 0.0;
  // the estimate cost of this_node to end
  double _estimated_cost = 0.0;
};

struct cmpNodeCost
{
  bool operator()(Node* a, Node* b)
  {
    if (std::abs(a->getTotalCost() - b->getTotalCost()) <= COST_UNIT) {
      return a->get_estimated_cost() > b->get_estimated_cost();
    } else {
      return a->getTotalCost() > b->getTotalCost();
    }
  }
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_NODE_H_