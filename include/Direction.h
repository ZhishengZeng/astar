/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 14:10:51
 * @FilePath: /astar/include/Direction.h
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
