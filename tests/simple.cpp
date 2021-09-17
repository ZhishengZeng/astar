/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 11:19:55
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
  astar_model.buildMap(2, 2);
  astar_model.addObstacle({0, 1}, 'H');
  astar_model.addObstacle({1, 0}, 'V');
  astar_model.disableDiagonalRouting();
  astar_model.enableTurningBack();
  astar_model.getPath({0, 0}, {1, 1});

  return 0;
}
