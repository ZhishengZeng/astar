/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-16 15:05:16
 * @FilePath: /AStar/tests/gtest.cpp
 */

#include "Model.h"
#include "gtest/gtest.h"

namespace {

TEST(RoutingTest, Reached)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 4);
  astar_model.addObstacle({0, 0}, 'A');
  astar_model.addObstacle({0, 1}, 'A');
  astar_model.addObstacle({1, 1}, 'A');
  astar_model.addObstacle({3, 1}, 'A');
  astar_model.addObstacle({4, 1}, 'A');
  astar_model.addObstacle({4, 3}, 'A');
  astar_model.addObstacle({5, 2}, 'A');
  astar_model.addObstacle({5, 3}, 'A');
  astar_model.addObstacle({6, 2}, 'A');
  astar_model.addObstacle({6, 3}, 'A');
  astar_model.addObstacle({7, 2}, 'A');
  astar_model.addObstacle({7, 3}, 'A');
  EXPECT_TRUE(astar_model.getPath({0, 3}, {7, 0}).size());
}

TEST(RoutingTest, NoWhere)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 4);
  astar_model.addObstacle({0, 0}, 'A');
  astar_model.addObstacle({0, 1}, 'A');
  astar_model.addObstacle({1, 1}, 'A');
  astar_model.addObstacle({3, 0}, 'A');
  astar_model.addObstacle({3, 1}, 'A');
  astar_model.addObstacle({4, 1}, 'A');
  astar_model.addObstacle({4, 3}, 'A');
  astar_model.addObstacle({5, 2}, 'A');
  astar_model.addObstacle({5, 3}, 'A');
  astar_model.addObstacle({6, 2}, 'A');
  astar_model.addObstacle({6, 3}, 'A');
  astar_model.addObstacle({7, 2}, 'A');
  astar_model.addObstacle({7, 3}, 'A');
  EXPECT_FALSE(astar_model.getPath({0, 3}, {7, 0}).size());
}

TEST(RoutingTest, Reached1)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 5);
  astar_model.addObstacle({2, 1}, 'A');
  astar_model.addObstacle({0, 3}, 'A');
  astar_model.addObstacle({1, 3}, 'A');
  astar_model.addObstacle({3, 2}, 'A');
  astar_model.addObstacle({4, 2}, 'A');
  astar_model.addObstacle({4, 4}, 'A');
  astar_model.addObstacle({5, 3}, 'A');
  astar_model.addObstacle({6, 2}, 'A');
  astar_model.addObstacle({6, 3}, 'A');
  astar_model.addObstacle({7, 2}, 'A');
  astar_model.addObstacle({7, 3}, 'A');
  astar_model.disableDiagonalRouting();
  astar_model.enableTurningBack();
  EXPECT_TRUE(astar_model.getPath({0, 4}, {7, 0}).size());
}

TEST(RoutingTest, NoWhere1)
{
  astar::Model astar_model;
  astar_model.buildMap(8, 5);
  astar_model.addObstacle({2, 1}, 'A');
  astar_model.addObstacle({0, 3}, 'A');
  astar_model.addObstacle({1, 3}, 'A');
  astar_model.addObstacle({3, 2}, 'A');
  astar_model.addObstacle({4, 2}, 'A');
  astar_model.addObstacle({4, 4}, 'A');
  astar_model.addObstacle({5, 3}, 'A');
  astar_model.addObstacle({6, 2}, 'A');
  astar_model.addObstacle({6, 3}, 'A');
  astar_model.addObstacle({7, 2}, 'A');
  astar_model.addObstacle({7, 3}, 'A');
  astar_model.disableDiagonalRouting();
  astar_model.disableTurningBack();
  EXPECT_FALSE(astar_model.getPath({0, 4}, {7, 0}).size());
}

}  // namespace
