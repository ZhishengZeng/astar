/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-11-13 18:45:37
 * @FilePath: /astar/tests/random_case_run.cpp
 */

#include <unistd.h>

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
#include "Util.h"
int main()
{
#if 1
  astar::Model astar_model;

  astar_model.setLogVerbose(2);

  astar_model.buildMap(5, 4);

  astar_model.addNodeCost({0, 0}, 0.9);
  astar_model.addNodeCost({0, 1}, 0.6);
  astar_model.addNodeCost({0, 2}, 0.7);
  astar_model.addNodeCost({0, 3}, 0.3);

  astar_model.addNodeCost({1, 0}, 0.7);
  astar_model.addNodeCost({1, 1}, 0.1);
  astar_model.addNodeCost({1, 2}, 0.9);
  astar_model.addNodeCost({1, 3}, 0.5);

  astar_model.addNodeCost({2, 0}, 0.4);
  astar_model.addNodeCost({2, 1}, 0.6);
  astar_model.addNodeCost({2, 2}, 0.4);
  astar_model.addNodeCost({2, 3}, 0.1);

  astar_model.addNodeCost({3, 0}, 0.2);
  astar_model.addNodeCost({3, 1}, 0.7);
  astar_model.addNodeCost({3, 2}, 0.3);
  astar_model.addNodeCost({3, 3}, 0.2);

  astar_model.addNodeCost({4, 0}, 0.3);
  astar_model.addNodeCost({4, 1}, 0.4);
  astar_model.addNodeCost({4, 2}, 0.7);
  astar_model.addNodeCost({4, 3}, 0.9);

  // astar_model.addNodeCost({0, 0}, 3);
  // astar_model.addNodeCost({0, 1}, 3);
  // astar_model.addNodeCost({0, 2}, 3);
  // astar_model.addNodeCost({0, 3}, 1);

  // astar_model.addNodeCost({1, 0}, 3);
  // astar_model.addNodeCost({1, 1}, 1);
  // astar_model.addNodeCost({1, 2}, 3);
  // astar_model.addNodeCost({1, 3}, 2);

  // astar_model.addNodeCost({2, 0}, 2);
  // astar_model.addNodeCost({2, 1}, 3);
  // astar_model.addNodeCost({2, 2}, 2);
  // astar_model.addNodeCost({2, 3}, 1);

  // astar_model.addNodeCost({3, 0}, 1);
  // astar_model.addNodeCost({3, 1}, 3);
  // astar_model.addNodeCost({3, 2}, 1);
  // astar_model.addNodeCost({3, 3}, 1);

  // astar_model.addNodeCost({4, 0}, 1);
  // astar_model.addNodeCost({4, 1}, 2);
  // astar_model.addNodeCost({4, 2}, 3);
  // astar_model.addNodeCost({4, 3}, 3);

  std::vector<astar::Coordinate> path = astar_model.getPath({0, 0}, {4, 3});

  return 0;

#else

  int n = 5;

  while (n--) {
    std::cout << "\n**************************************" << std::endl;
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
    astar_model.setLogVerbose(2);
    astar_model.buildMap(x_size, y_size);

    std::vector<astar::Coordinate> obs_coord_list;
    for (size_t i = 1; i < coord_list.size() - 1; i++) {
      int select = rand() % 3;
      astar::ObsType type;
      type = astar::ObsType::kOmniObs;
      if (select == 0) {
        type = astar::ObsType::kHObs;
      } else if (select == 1) {
        type = astar::ObsType::kVObs;
      }
      astar_model.addObstacle(coord_list[i], type);
    }
    std::vector<astar::Coordinate> path = astar_model.getPath(coord_list.front(), coord_list.back());

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Run time:" << (end_time - start_time) << std::endl;
    std::cout << "**************************************" << std::endl;
    sleep(1);
  }
  return 0;

#endif
}
