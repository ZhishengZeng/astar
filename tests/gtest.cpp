/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 14:10:48
 * @FilePath: /AStar/tests/gtest.cpp
 */

#include "Model.h"
#include "gtest/gtest.h"

namespace {

TEST(RoutingTest, Reached)
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
  EXPECT_TRUE(astar_model.findPath({0, 3}, {7, 0}).size());
}

TEST(RoutingTest, NoWhere)
{
  astar::Model astar_model;
  astar_model.setMapSize(8, 4);
  astar_model.setObstacle({{0, 0},
                           {0, 1},
                           {1, 1},
                           {3, 0},
                           {3, 1},
                           {4, 1},
                           {4, 3},
                           {5, 1},
                           {5, 2},
                           {5, 3},
                           {6, 2},
                           {6, 3},
                           {7, 2},
                           {7, 3}});
  EXPECT_FALSE(astar_model.findPath({0, 3}, {7, 0}).size());
}

TEST(RoutingTest, routing_diagonal_turning_back)
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
  EXPECT_FALSE(astar_model.findPath({0, 4}, {7, 0}).size());
}

}  // namespace
