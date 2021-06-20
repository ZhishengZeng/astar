/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 19:03:30
 * @LastEditTime: 2021-06-19 18:00:46
 * @LastEditors: Please set LastEditors
 * @Description:
 * @FilePath: /AStar/main.cpp
 */
#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
int main()
{
  int x_grids = 20;
  int y_grids = 40;
  srand((unsigned) time(NULL));
  std::set<astar::Coordinate, astar::cmpCoordinate> coord_set;
  while ((int) coord_set.size() < (x_grids * y_grids / 4)) {
    coord_set.insert({rand() % x_grids, rand() % y_grids});
  }
  std::vector<astar::Coordinate> coord_list(coord_set.begin(), coord_set.end());
  std::vector<astar::Coordinate> obs_coord_list;
  for (size_t i = 1; i < coord_list.size() - 1; i++) {
    obs_coord_list.push_back(coord_list[i]);
  }

  astar::Model astar_model;
  astar_model.setMapSize(x_grids, y_grids);
  astar_model.setObstacle(obs_coord_list);
  astar_model.findPath(coord_list.front(), coord_list.back());

  return 0;
}
