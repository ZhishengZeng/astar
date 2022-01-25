/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-25 16:28:45
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
  int n = 1;

  while (n--) {
    std::cout << "\n**************************************" << std::endl;
    double start_time, end_time;
    start_time = astar::Util::microtime();

    int x_size = 8;
    int y_size = 8;
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
      astar::Direction2d type = (astar::Direction2d)(rand() % 4 + 1);
      astar_model.addOBS(coord_list[i], type);
    }

    std::vector<std::pair<astar::Coordinate, double>> coord_cost_list;
    for (size_t i = 1; i < coord_list.size() - 1; i++) {
      double cost = rand() % 4 + 1;
      astar::Direction2d type = (astar::Direction2d)(rand() % 4 + 1);
      astar_model.addCost(coord_list[i], type, cost);
    }

    std::vector<astar::Coordinate> path = astar_model.getPath(coord_list.front(), coord_list.back());

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Run time:" << (end_time - start_time) << std::endl;
    std::cout << "**************************************" << std::endl;
    sleep(1);
  }
  return 0;
}
