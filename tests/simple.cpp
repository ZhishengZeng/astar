/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-10-12 21:56:17
 * @FilePath: /astar/tests/simple.cpp
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
  astar_model.addObstacle({0, 0}, astar::NodeType::kOmniObs);
  astar_model.addObstacle({1, 0}, astar::NodeType::kOmniObs);
  astar_model.setLogVerbose(2);
  std::vector<astar::Coordinate> path_list = astar_model.getPath({0, 0}, {5, 8});
  return 0;
}
