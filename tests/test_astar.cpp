/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-06 15:33:44
 * @FilePath: /astar/tests/test_astar.cpp
 */

#include "Model.h"
#include "gtest/gtest.h"

namespace {

TEST(RoutingTest, Reached)
{
  astar::Model astar_model;
  astar_model.buildMap(4, 3);
  EXPECT_TRUE(astar_model.getPath({0, 0}, {3, 2}).size());
}

}  // namespace
