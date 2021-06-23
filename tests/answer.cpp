/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-20 12:31:43
 * @Description:
 * @FilePath: /AStar/tests/answer.cpp
 */
#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
int main()
{
  astar::Model astar_model;
  astar_model.setMapSize(8, 4);
  astar_model.setObstacle({{0, 0},
                           {0, 1},
                           {1, 1},
                           {3, 1},
                           {4, 1},
                           {4, 3},
                           {5, 2},
                           {5, 3},
                           {6, 2},
                           {6, 3},
                           {7, 2},
                           {7, 3}});
  astar_model.findPath({0, 3}, {7, 0});
  return 0;
}
