/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 14:10:39
 * @FilePath: /AStar/tests/simple.cpp
 */

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
int main()
{
  {
    astar::Model astar_model;
    astar_model.setMapSize(8, 5);
    astar_model.setObstacle({{2, 1},
                             {0, 3},
                             {1, 3},
                             {3, 2},
                             {4, 2},
                             {4, 4},
                             {5, 3},
                             {5, 3},
                             {6, 2},
                             {6, 3},
                             {7, 2},
                             {7, 3}});
    astar_model.setRoutingMode(true, true);
    astar_model.findPath({0, 4}, {7, 0});
  }

  {
    astar::Model astar_model;
    astar_model.setMapSize(8, 5);
    astar_model.setObstacle({{2, 1},
                             {0, 3},
                             {1, 3},
                             {3, 2},
                             {4, 2},
                             {4, 4},
                             {5, 3},
                             {5, 3},
                             {6, 2},
                             {6, 3},
                             {7, 2},
                             {7, 3}});
    astar_model.setRoutingMode(false, true);
    astar_model.findPath({0, 4}, {7, 0});
  }

  {
    astar::Model astar_model;
    astar_model.setMapSize(8, 5);
    astar_model.setObstacle({{2, 1},
                             {0, 3},
                             {1, 3},
                             {3, 2},
                             {4, 2},
                             {4, 4},
                             {5, 3},
                             {5, 3},
                             {6, 2},
                             {6, 3},
                             {7, 2},
                             {7, 3}});
    astar_model.setRoutingMode(true, false);
    astar_model.findPath({0, 4}, {7, 0});
  }

  {
    astar::Model astar_model;
    astar_model.setMapSize(8, 5);
    astar_model.setObstacle({{2, 1},
                             {0, 3},
                             {1, 3},
                             {3, 2},
                             {4, 2},
                             {4, 4},
                             {5, 3},
                             {5, 3},
                             {6, 2},
                             {6, 3},
                             {7, 2},
                             {7, 3}});
    astar_model.setRoutingMode(false, false);
    astar_model.findPath({0, 4}, {7, 0});
  }

  return 0;
}
