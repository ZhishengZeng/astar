/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-02-09 16:16:24
 * @FilePath: /astar/tests/test_astar.cpp
 */

#include "AS_Model.hpp"
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
  astar_model.addOBS({3, 0}, astar::Orientation::kNorth);
  astar_model.addOBS({3, 0}, astar::Orientation::kSouth);
  astar_model.addOBS({5, 0}, astar::Orientation::kNorth);
  astar_model.addOBS({5, 0}, astar::Orientation::kSouth);
  EXPECT_TRUE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, Reached4)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({3, 0}, astar::Orientation::kNorth);
  astar_model.addOBS({3, 0}, astar::Orientation::kSouth);
  astar_model.addOBS({5, 0}, astar::Orientation::kNorth);
  astar_model.addOBS({5, 0}, astar::Orientation::kSouth);
  EXPECT_TRUE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, Reached5)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({0, 0}, astar::Orientation::kSouth);
  astar_model.addOBS({0, 0}, astar::Orientation::kNorth);
  EXPECT_TRUE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, Reached6)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({0, 0}, astar::Orientation::kSouth);
  astar_model.addOBS({0, 0}, astar::Orientation::kNorth);
  EXPECT_TRUE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, Reached7)
{
  astar::Model astar_model;
  astar_model.buildMap(1, 1);
  astar_model.setLogVerbose(2);
  astar_model.getPath({0, 0}, {0, 0});
}

TEST(RoutingTest, NoWhere1)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({3, 0}, astar::Orientation::kWest);
  astar_model.addOBS({3, 0}, astar::Orientation::kEast);
  astar_model.addOBS({5, 0}, astar::Orientation::kWest);
  astar_model.addOBS({5, 0}, astar::Orientation::kEast);
  EXPECT_FALSE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, NoWhere2)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({3, 0}, astar::Orientation::kWest);
  astar_model.addOBS({3, 0}, astar::Orientation::kEast);
  astar_model.addOBS({5, 0}, astar::Orientation::kWest);
  astar_model.addOBS({5, 0}, astar::Orientation::kEast);
  EXPECT_FALSE(astar_model.getPath({9, 0}, {0, 0}).size());
}

TEST(RoutingTest, NoWhere3)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({0, 0}, astar::Orientation::kWest);
  astar_model.addOBS({0, 0}, astar::Orientation::kEast);
  EXPECT_FALSE(astar_model.getPath({0, 0}, {9, 0}).size());
}

TEST(RoutingTest, NoWhere4)
{
  astar::Model astar_model;
  astar_model.buildMap(10, 1);
  astar_model.setLogVerbose(2);
  astar_model.addOBS({0, 0}, astar::Orientation::kWest);
  astar_model.addOBS({0, 0}, astar::Orientation::kEast);
  EXPECT_FALSE(astar_model.getPath({9, 0}, {0, 0}).size());
}

}  // namespace
