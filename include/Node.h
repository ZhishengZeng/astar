/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-08 11:15:10
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
  std::set<ObsType>& get_obs_set() { return _obs_set; }
  Node* get_parent_node() { return _parent_node; }
  double get_self_cost() const { return _self_cost; }
  double get_cert_sum() const { return _cert_sum; }
  double get_est_sum() const { return _est_sum; }
  // setter
  void set_coord(const int x, const int y)
  {
    _coord.set_x(x);
    _coord.set_y(y);
  }
  void set_obs_set(const std::set<ObsType>& obs_set) { _obs_set = obs_set; }
  void set_parent_node(Node* parent_node) { _parent_node = parent_node; }
  void set_self_cost(const double self_cost) { _self_cost = self_cost; }
  void set_cert_sum(const double cert_sum) { _cert_sum = cert_sum; }
  void set_est_sum(const double est_sum) { _est_sum = est_sum; }

  // function
  void setOpen() { _search_state = SearchState::kOpen; }
  void setClose() { _search_state = SearchState::kClose; }
  bool isOpen() const { return _search_state == SearchState::kOpen; }
  bool isClose() const { return _search_state == SearchState::kClose; }
  double getTotalCost() const { return (_cert_sum + _self_cost + _est_sum); }

 private:
  Coordinate _coord;
  std::set<ObsType> _obs_set;
  SearchState _search_state = SearchState::kNone;
  Node* _parent_node = nullptr;
  // self cost
  double _self_cost = 0.0;
  // the certain cost of this_node from start_node
  double _cert_sum = 0.0;
  // the estimate cost of this_node to end_node
  double _est_sum = 0.0;
};

struct cmpNodeCost
{
  bool operator()(Node* a, Node* b)
  {
    if (std::abs(a->getTotalCost() - b->getTotalCost()) <= COST_UNIT) {
      return (a->get_self_cost() + a->get_est_sum()) > (b->get_self_cost() + b->get_est_sum());
    } else {
      return a->getTotalCost() > b->getTotalCost();
    }
  }
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_NODE_H_