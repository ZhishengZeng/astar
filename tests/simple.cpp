/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-10-06 16:02:10
 * @FilePath: /AStar/tests/simple.cpp
 */

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
int main()
{
  astar::Model astar_model;
  astar_model.buildMap(6, 9);
  astar_model.setLogVerbose(0);
  astar_model.disableDiagonalRouting();
  astar_model.disableTurningBack();
  astar_model.addObstacle({0, 0}, 'O');
  astar_model.addObstacle({1, 0}, 'O');
  std::vector<astar::Coordinate> path_list = astar_model.getPath({0, 0}, {5, 8});
  return 0;
}
