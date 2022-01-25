/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-25 19:03:15
 * @FilePath: /astar/tests/random_case_run.cpp
 */

#include <unistd.h>

#include <iostream>
#include <list>
#include <set>
#include <vector>

#include "Model.h"
#include "Util.h"

std::vector<astar::Coordinate> getRandomCoordList(int x_size, int y_size, int size)
{
  std::set<astar::Coordinate, astar::cmpCoordinate> coord_set;
  while ((int) coord_set.size() < size) {
    coord_set.insert({rand() % x_size, rand() % y_size});
  }

  return std::vector<astar::Coordinate>(coord_set.begin(), coord_set.end());
}

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
    std::vector<astar::Coordinate> random_obs_list = getRandomCoordList(x_size, y_size, x_size * y_size / 4);
    std::vector<astar::Coordinate> random_cost_list = getRandomCoordList(x_size, y_size, x_size * y_size / 2);
    std::vector<astar::Coordinate> random_term_list = getRandomCoordList(x_size, y_size, 2);

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Create example time:" << (end_time - start_time) << std::endl;
    start_time = end_time;

    astar::Model astar_model;
    astar_model.setLogVerbose(2);
    astar_model.buildMap(x_size, y_size);

    std::vector<astar::Coordinate> obs_coord_list;
    for (size_t i = 0; i < random_obs_list.size(); i++) {
      astar_model.addOBS(random_obs_list[i], (astar::Direction2d)(rand() % 4 + 1));
      astar_model.addOBS(random_obs_list[i], (astar::Direction2d)(rand() % 4 + 1));
    }

    std::vector<std::pair<astar::Coordinate, double>> coord_cost_list;
    for (size_t i = 0; i < random_cost_list.size(); i++) {
      astar_model.addCost(random_cost_list[i], (astar::Direction2d)(rand() % 4 + 1), rand() % 4 + 1);
      astar_model.addCost(random_cost_list[i], (astar::Direction2d)(rand() % 4 + 1), rand() % 4 + 1);
      astar_model.addCost(random_cost_list[i], (astar::Direction2d)(rand() % 4 + 1), rand() % 4 + 1);
      astar_model.addCost(random_cost_list[i], (astar::Direction2d)(rand() % 4 + 1), rand() % 4 + 1);
    }

    std::vector<astar::Coordinate> path = astar_model.getPath(random_obs_list.front(), random_obs_list.back());

    end_time = astar::Util::microtime();
    std::cout << "[astar Info] Run time:" << (end_time - start_time) << std::endl;
    std::cout << "**************************************" << std::endl;
    sleep(1);
  }
  return 0;
}
