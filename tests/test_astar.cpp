/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-09 14:10:11
 * @FilePath: /astar/tests/test_astar.cpp
 */

#include "Model.h"
#include "gtest/gtest.h"

namespace {

TEST(RoutingTest, Reached1)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  EXPECT_TRUE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, Reached2)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  EXPECT_TRUE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, Reached3)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({3, 0}, astar::ObsType::kNorthObs);
  astar_model.addObstacle({3, 0}, astar::ObsType::kSouthObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kNorthObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kSouthObs);
  EXPECT_TRUE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, Reached4)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({3, 0}, astar::ObsType::kNorthObs);
  astar_model.addObstacle({3, 0}, astar::ObsType::kSouthObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kNorthObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kSouthObs);
  EXPECT_TRUE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, Reached5)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({0, 0}, astar::ObsType::kSouthObs);
  astar_model.addObstacle({0, 0}, astar::ObsType::kNorthObs);
  EXPECT_TRUE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, Reached6)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({0, 0}, astar::ObsType::kSouthObs);
  astar_model.addObstacle({0, 0}, astar::ObsType::kNorthObs);
  EXPECT_TRUE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, NoWhere1)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({3, 0}, astar::ObsType::kWestObs);
  astar_model.addObstacle({3, 0}, astar::ObsType::kEastObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kWestObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kEastObs);
  EXPECT_FALSE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, NoWhere2)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({3, 0}, astar::ObsType::kWestObs);
  astar_model.addObstacle({3, 0}, astar::ObsType::kEastObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kWestObs);
  astar_model.addObstacle({5, 0}, astar::ObsType::kEastObs);
  EXPECT_FALSE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, NoWhere3)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({0, 0}, astar::ObsType::kWestObs);
  astar_model.addObstacle({0, 0}, astar::ObsType::kEastObs);
  EXPECT_FALSE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, NoWhere4)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addObstacle({0, 0}, astar::ObsType::kWestObs);
  astar_model.addObstacle({0, 0}, astar::ObsType::kEastObs);
  EXPECT_FALSE(astar_model.getPath({9, 0}, {0, 0}).size());
}

}  // namespace
