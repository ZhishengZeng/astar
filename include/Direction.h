// Copyright 2021 Author ZengZhisheng
#ifndef ASTAR_INCLUDE_DIRECTION_H_
#define ASTAR_INCLUDE_DIRECTION_H_
namespace astar {

enum class Direction
{
  kNone = 0,
  kHorizontal = 1,
  kVertical = 2,
  kDiagonal = 3
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_DIRECTION_H_
