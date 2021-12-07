/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-06 12:36:25
 * @FilePath: /astar/include/Node.h
 */

#ifndef ASTAR_INCLUDE_NODE_H_
#define ASTAR_INCLUDE_NODE_H_

#include "Coordinate.h"
#include "ObsType.h"
#include "SearchState.h"

namespace astar {

class Node
{
 public:
  Node() {}
  ~Node() {}
  // getter
  Coordinate& get_coord() { return _coord; }
  std::vector<ObsType>& get_obs_list() { return _obs_list; }
  Node* get_parent_node() { return _parent_node; }
  double get_self_cost() const { return _self_cost; }
  double get_cert_cost() const { return _cert_cost; }
  double get_est_cost() const { return _est_cost; }
  // setter
  void set_coord(const int x, const int y)
  {
    _coord.set_x(x);
    _coord.set_y(y);
  }
  void set_obs_list(const std::vector<ObsType>& obs_list) { _obs_list = obs_list; }
  void set_parent_node(Node* parent_node) { _parent_node = parent_node; }
  void set_self_cost(const double self_cost) { _self_cost = self_cost; }
  void set_cert_cost(const double cert_cost) { _cert_cost = cert_cost; }
  void set_est_cost(const double est_cost) { _est_cost = est_cost; }

  // function
  void setOpen() { _search_state = SearchState::kOpen; }
  void setClose() { _search_state = SearchState::kClose; }
  bool isOpen() const { return _search_state == SearchState::kOpen; }
  bool isClose() const { return _search_state == SearchState::kClose; }
  double getTotalCost() const { return (_self_cost + _cert_cost + _est_cost); }

 private:
  Coordinate _coord;
  std::vector<ObsType> _obs_list;
  SearchState _search_state = SearchState::kNone;
  Node* _parent_node = nullptr;
  // self cost
  double _self_cost = 0;
  // the certain cost of this_node from start_node
  double _cert_cost = 0;
  // the estimate cost of this_node to end_node
  double _est_cost = 0;
};

struct cmpNodeCost
{
  bool operator()(Node* a, Node* b) { return a->getTotalCost() >= b->getTotalCost(); }
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_NODE_H_