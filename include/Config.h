/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 11:51:30
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-10-06 15:43:22
 * @FilePath: /AStar/include/Config.h
 */
#ifndef ASTAR_INCLUDE_CONFIG_H_
#define ASTAR_INCLUDE_CONFIG_H_

#include <vector>

#include "Coordinate.h"
#include "NodeType.h"
namespace astar {

class Config
{
 private:
  int _map_x_size;
  int _map_y_size;
  std::vector<std::pair<Coordinate, double>> _coord_cost_list;
  std::vector<std::pair<Coordinate, NodeType>> _coord_type_list;
  int _log_verbose = 0;
  bool _turning_back = true;
  bool _routing_diagonal = false;

 public:
  Config() {}
  ~Config() {}

  // getter
  int get_map_x_size() const { return _map_x_size; }
  int get_map_y_size() const { return _map_y_size; }
  std::vector<std::pair<Coordinate, double>>& get_coord_cost_list() { return _coord_cost_list; }
  std::vector<std::pair<Coordinate, NodeType>>& get_coord_type_list() { return _coord_type_list; }
  int get_log_verbose() const { return _log_verbose; }
  bool get_turning_back() const { return _turning_back; }
  bool get_routing_diagonal() const { return _routing_diagonal; }

  // setter
  void set_map_x_size(const int map_x_size) { _map_x_size = map_x_size; }
  void set_map_y_size(const int map_y_size) { _map_y_size = map_y_size; }
  void set_log_verbose(const int log_verbose) { _log_verbose = log_verbose; }
  void set_turning_back(const bool turning_back) { _turning_back = turning_back; }
  void set_routing_diagonal(const bool routing_diagonal) { _routing_diagonal = routing_diagonal; }
  // function
  bool isTurningBack() { return _turning_back; }
  bool isRoutingDiagonal() { return _routing_diagonal; }
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_CONFIG_H_