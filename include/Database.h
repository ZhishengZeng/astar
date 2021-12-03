/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 11:51:30
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 12:56:38
 * @FilePath: /astar/include/Database.h
 */
#ifndef ASTAR_INCLUDE_DATABASE_H_
#define ASTAR_INCLUDE_DATABASE_H_

#include <vector>

#include "Coordinate.h"
#include "ObsType.h"
namespace astar {

class Database
{
 private:
  int _map_x_size;
  int _map_y_size;
  std::vector<std::pair<Coordinate, double>> _coord_cost_list;
  std::vector<std::pair<Coordinate, ObsType>> _coord_type_list;
  bool _turning_back = true;
  bool _routing_diagonal = false;

 public:
  Database() {}
  ~Database() {}

  // getter
  int get_map_x_size() const { return _map_x_size; }
  int get_map_y_size() const { return _map_y_size; }
  std::vector<std::pair<Coordinate, double>>& get_coord_cost_list() { return _coord_cost_list; }
  std::vector<std::pair<Coordinate, ObsType>>& get_coord_type_list() { return _coord_type_list; }
  bool get_turning_back() const { return _turning_back; }
  bool get_routing_diagonal() const { return _routing_diagonal; }

  // setter
  void set_map_x_size(const int map_x_size) { _map_x_size = map_x_size; }
  void set_map_y_size(const int map_y_size) { _map_y_size = map_y_size; }
  void set_turning_back(const bool turning_back) { _turning_back = turning_back; }
  void set_routing_diagonal(const bool routing_diagonal) { _routing_diagonal = routing_diagonal; }
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_DATABASE_H_