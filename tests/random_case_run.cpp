/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-07 16:22:16
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

    int x_size = 15;
    int y_size = 15;
    // srand((unsigned) time(NULL));
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
      astar::ObsType type = (astar::ObsType)(rand() % 4 + 1);
      astar_model.addObstacle(coord_list[i], type);
    }
    std::vector<astar::Coordinate> path = astar_model.getPath(coord_list.front(), coord_list.back());

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Run time:" << (end_time - start_time) << std::endl;
    std::cout << "**************************************" << std::endl;
    sleep(1);
  }
  return 0;
}
