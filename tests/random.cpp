/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-20 12:31:43
 * @Description:
 * @FilePath: /AStar/tests/random.cpp
 */
#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
int main()
{
  int x_grid_num = 800;
  int y_grid_num = 800;
  srand((unsigned) time(NULL));
  std::set<astar::Coordinate, astar::cmpCoordinate> coord_set;
  while ((int) coord_set.size() < (x_grid_num * y_grid_num / 4)) {
    coord_set.insert({rand() % x_grid_num, rand() % y_grid_num});
  }
  std::vector<astar::Coordinate> coord_list(coord_set.begin(), coord_set.end());
  std::vector<astar::Coordinate> obs_coord_list;
  for (size_t i = 1; i < coord_list.size() - 1; i++) {
    obs_coord_list.push_back(coord_list[i]);
  }

  astar::Model astar_model;
  astar_model.setMapSize(x_grid_num, y_grid_num);
  astar_model.setObstacle(obs_coord_list);
  astar_model.findPath(coord_list.front(), coord_list.back());

  return 0;
}
