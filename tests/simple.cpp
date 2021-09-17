/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 15:53:59
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
  astar_model.buildMap(8, 4);
  astar_model.addObstacle({0, 0}, 'O');
  astar_model.addObstacle({0, 1}, 'O');
  astar_model.addObstacle({1, 1}, 'O');
  astar_model.addObstacle({3, 1}, 'O');
  astar_model.addObstacle({4, 1}, 'O');
  astar_model.addObstacle({4, 3}, 'O');
  astar_model.addObstacle({5, 2}, 'O');
  astar_model.addObstacle({5, 3}, 'O');
  astar_model.addObstacle({6, 2}, 'O');
  astar_model.addObstacle({6, 3}, 'O');
  astar_model.addObstacle({7, 2}, 'O');
  astar_model.addObstacle({7, 3}, 'O');
  astar_model.getPath({0, 3}, {7, 0});

  return 0;
}
