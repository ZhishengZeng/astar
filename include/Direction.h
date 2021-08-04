/*
 * @Author: ZengZhisheng
 * @Date: 2021-08-04 13:00:49
 * @Description:
 * @FilePath: /AStar/include/Direction.h
 */

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
