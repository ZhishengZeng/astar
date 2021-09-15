/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 17:19:17
 * @FilePath: /AStar/tests/gtest.cpp
 */

#include "Model.h"
#include "gtest/gtest.h"

namespace {

TEST(RoutingTest, Reached)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 4);
  astar_model.addObstacle(
      {{0, 0}, {0, 1}, {1, 1}, {3, 1}, {4, 1}, {4, 3}, {5, 2}, {5, 3}, {6, 2}, {6, 3}, {7, 2}, {7, 3}});
  EXPECT_TRUE(astar_model.findPath({0, 3}, {7, 0}).size());
}

TEST(RoutingTest, NoWhere)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 4);
  astar_model.addObstacle(
      {{0, 0}, {0, 1}, {1, 1}, {3, 0}, {3, 1}, {4, 1}, {4, 3}, {5, 1}, {5, 2}, {5, 3}, {6, 2}, {6, 3}, {7, 2}, {7, 3}});
  EXPECT_FALSE(astar_model.findPath({0, 3}, {7, 0}).size());
}

TEST(RoutingTest, Reached1)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 5);
  astar_model.addObstacle(
      {{2, 1}, {0, 3}, {1, 3}, {3, 2}, {4, 2}, {4, 4}, {5, 3}, {5, 3}, {6, 2}, {6, 3}, {7, 2}, {7, 3}});
  astar_model.disableDiagonalRouting();
  astar_model.enableTurningBack();
  EXPECT_TRUE(astar_model.findPath({0, 4}, {7, 0}).size());
}

TEST(RoutingTest, NoWhere1)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 5);
  astar_model.addObstacle(
      {{2, 1}, {0, 3}, {1, 3}, {3, 2}, {4, 2}, {4, 4}, {5, 3}, {5, 3}, {6, 2}, {6, 3}, {7, 2}, {7, 3}});
  astar_model.disableDiagonalRouting();
  astar_model.disableTurningBack();
  EXPECT_FALSE(astar_model.findPath({0, 4}, {7, 0}).size());
}

}  // namespace
