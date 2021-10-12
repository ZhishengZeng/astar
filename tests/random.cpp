/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-10-12 21:53:52
 * @FilePath: /astar/tests/random.cpp
 */

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
#include "Util.h"
int main()
{
  int n = 10000;

  while (n--) {
    double start_time, end_time;
    start_time = astar::Util::microtime();

    int x_size = 15;
    int y_size = 15;
    srand((unsigned) time(NULL));
    std::set<astar::Coordinate, astar::cmpCoordinate> coord_set;
    while ((int) coord_set.size() < (x_size * y_size / 2)) {
      coord_set.insert({rand() % x_size, rand() % y_size});
    }
    std::vector<astar::Coordinate> coord_list(coord_set.begin(), coord_set.end());

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Create example time:" << (end_time - start_time) << std::endl;
    start_time = end_time;

    astar::Model astar_model;
    astar_model.buildMap(x_size, y_size);

    std::vector<astar::Coordinate> obs_coord_list;
    for (size_t i = 1; i < coord_list.size() - 1; i++) {
      int select = rand() % 3;
      astar::NodeType type;
      type = astar::NodeType::kOmniObs;
      if (select == 0) {
        type = astar::NodeType::kHObs;
      } else if (select == 1) {
        type = astar::NodeType::kVObs;
      }
      astar_model.addObstacle(coord_list[i], type);
    }
    std::vector<astar::Coordinate> path = astar_model.getPath(coord_list.front(), coord_list.back());

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Run time:" << (end_time - start_time) << std::endl;

    if (path.size() == 0) {
      exit(1);
    }
  }
  return 0;
}
