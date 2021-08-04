/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 16:49:08
 * @Description:
 * @FilePath: /AStar/include/Node.h
 */

#ifndef ASTAR_INCLUDE_NODE_H_
#define ASTAR_INCLUDE_NODE_H_

#include "Coordinate.h"

namespace astar {

enum class NodeType
{
  kObs = -1,
  kNone = 0,
  kStart = 1,
  kEnd = 2
};

enum class NodeState
{
  kNone = 0,
  kOpen = 1,
  kClose = 2
};

class Node
{
 public:
  Node() {}
  ~Node() { _parent_node = nullptr; }

  Coordinate& get_coord() { return _coord; }
  NodeType& get_type() { return _type; }
  NodeState& get_state() { return _state; }
  Node* get_parent_node() { return _parent_node; }
  double get_known_cost() const { return _known_cost; }
  double get_est_cost() const { return _est_cost; }
  double get_total_cost() const { return (_known_cost + _est_cost); }
  bool get_on_path() { return _on_path; }

  void set_coord(const int x,const int y)
  {
    _coord.set_x(x);
    _coord.set_y(y);
  }
  void set_type(const NodeType& type) { _type = type; }
  void set_state(const NodeState& state) { _state = state; }
  void set_parent_node(Node* parent_node) { _parent_node = parent_node; }
  void set_known_cost(const double known_cost) { _known_cost = known_cost; }
  void set_est_cost(const double est_cost) { _est_cost = est_cost; }
  void set_on_path(const bool on_path) { _on_path = on_path; }
  // function
  bool isStart() { return _type == NodeType::kStart; }
  bool isEnd() { return _type == NodeType::kEnd; }
  bool isObs() { return _type == NodeType::kObs; }
  bool isOpen() { return _state == NodeState::kOpen; }
  bool isClose() { return _state == NodeState::kClose; }

 private:
  Coordinate _coord;
  NodeType _type = NodeType::kNone;
  NodeState _state = NodeState::kNone;
  Node* _parent_node = nullptr;
  double _known_cost = 0;
  double _est_cost = 0;
  bool _on_path = false;
};

struct cmpNodeCost
{
  bool operator()(Node* a, Node* b)
  {
    return a->get_total_cost() >= b->get_total_cost();
  }
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_NODE_H_