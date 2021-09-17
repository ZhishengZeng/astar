/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 19:20:57
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
  astar_model.addObstacle({0, 0}, 'O');
  astar_model.addObstacle({0, 1}, 'O');
  astar_model.getPath({0, 0}, {0, 1});

  return 0;
}
