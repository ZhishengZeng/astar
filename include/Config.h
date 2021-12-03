/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 11:51:30
 * @Description:
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-03 21:16:19
 * @FilePath: /astar/include/Config.h
 */
#ifndef ASTAR_INCLUDE_CONFIG_H_
#define ASTAR_INCLUDE_CONFIG_H_

#include <vector>

#include "Coordinate.h"
#include "ObsType.h"
namespace astar {

class Config
{
 private:
  int _map_x_size;
  int _map_y_size;
  std::map<Coordinate, double, cmpCoordinate> _coord_cost_map;
  std::map<Coordinate, std::vector<ObsType>, cmpCoordinate> _coord_obs_map;
  int _log_verbose = 0;
  bool _turning_back = true;

 public:
  Config() {}
  ~Config() {}

  // getter
  int get_map_x_size() const { return _map_x_size; }
  int get_map_y_size() const { return _map_y_size; }
  std::map<Coordinate, double, cmpCoordinate>& get_coord_cost_map() { return _coord_cost_map; }
  std::map<Coordinate, std::vector<ObsType>, cmpCoordinate>& get_coord_obs_map() { return _coord_obs_map; }
  int get_log_verbose() const { return _log_verbose; }
  bool get_turning_back() const { return _turning_back; }
  // setter
  void set_map_x_size(const int map_x_size) { _map_x_size = map_x_size; }
  void set_map_y_size(const int map_y_size) { _map_y_size = map_y_size; }
  void set_log_verbose(const int log_verbose) { _log_verbose = log_verbose; }
  void set_turning_back(const bool turning_back) { _turning_back = turning_back; }
  // function
  bool isTurningBack() { return _turning_back; }
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_CONFIG_H_