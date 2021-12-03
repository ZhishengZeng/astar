/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-03 14:25:44
 * @FilePath: /astar/include/Direction.h
 */

#ifndef ASTAR_INCLUDE_DIRECTION_H_
#define ASTAR_INCLUDE_DIRECTION_H_
namespace astar {

enum class Direction
{
  kNone = 0,
  kH= 1,
  kV = 2
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_DIRECTION_H_
