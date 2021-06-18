/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 19:03:30
 * @LastEditTime: 2021-06-18 19:17:40
 * @LastEditors: ZengZhisheng
 * @Description:
 * @FilePath: /AStar/main.cpp
 */
#include <iostream>
#include <list>
#include <vector>

#include "Model.h"
int main()
{
  astar::Model astar_model;
  astar_model.initMap(15, 29);
  astar_model.setStartPoint(1, 1);
  astar_model.setEndPoint(14, 17);
  astar_model.setObsPoint(2, 7);
  astar_model.run();
  astar_model.showResult();
  return 0;
}
