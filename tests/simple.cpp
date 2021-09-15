/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 17:17:29
 * @FilePath: /AStar/tests/simple.cpp
 */

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
int main()
{
  astar::GridMap<double> cost_map;

  astar::Model astar_model;
  astar_model.buildMap(4, 3);
  astar_model.addNodeCost({{{1, 0}, 10}, {{1, 1}, 10}, {{2, 0}, 10}, {{2, 1}, 10}, {{3, 0}, 10}, {{3, 1}, 10}});
  astar_model.addObstacle({{2, 0}, {1, 2}});
  astar_model.disableDiagonalRouting();
  astar_model.enableTurningBack();
  astar_model.findPath({1, 0}, {3, 2});

  return 0;
}
