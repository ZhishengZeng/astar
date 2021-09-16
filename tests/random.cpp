/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-16 15:05:05
 * @FilePath: /AStar/tests/random.cpp
 */

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
#include "Util.h"
int main()
{
  double start_time, end_time;
  start_time = astar::Util::microtime();

  int x_size = 5000;
  int y_size = 5000;
  srand((unsigned) time(NULL));
  std::set<astar::Coordinate, astar::cmpCoordinate> coord_set;
  while ((int) coord_set.size() < (x_size * y_size / 4)) {
    coord_set.insert({rand() % x_size, rand() % y_size});
  }
  std::vector<astar::Coordinate> coord_list(coord_set.begin(), coord_set.end());

  end_time = astar::Util::microtime();
  std::cout << "[AStar Info] Create example time:" << (end_time - start_time)
            << std::endl;
  start_time = end_time;

  astar::Model astar_model;
  astar_model.buildMap(x_size, y_size);

  std::vector<astar::Coordinate> obs_coord_list;
  for (size_t i = 1; i < coord_list.size() - 1; i++) {
    astar_model.addObstacle(coord_list[i], 'A');
  }
  astar_model.getPath(coord_list.front(), coord_list.back());

  end_time = astar::Util::microtime();
  std::cout << "[AStar Info] Run time:" << (end_time - start_time) << std::endl;
  return 0;
}
