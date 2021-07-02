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
  EXPECT_TRUE(astar_model.findPath({0, 3}, {7, 0}));
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
  EXPECT_FALSE(astar_model.findPath({0, 3}, {7, 0}));
}

}  // namespace
