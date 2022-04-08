/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-24 13:14:37
 * @FilePath: /astar/include/Direction2d.h
 */

#ifndef ASTAR_INCLUDE_DIRECTION2D_H_
#define ASTAR_INCLUDE_DIRECTION2D_H_

namespace astar {

enum class Direction2d
{
  kNone = 0,
  kEast = 1,
  kWest = 2,
  kSouth = 3,
  kNorth = 4
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_DIRECTION2D_H_
