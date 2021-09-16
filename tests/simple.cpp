/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 00:02:22
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
  astar_model.buildMap(5, 3);
  astar_model.addObstacle({2, 2}, 'H');
  astar_model.addObstacle({1, 2}, 'V');
  astar_model.disableDiagonalRouting();
  astar_model.enableTurningBack();
  astar_model.getPath({4, 2}, {0, 2});

  return 0;
}
