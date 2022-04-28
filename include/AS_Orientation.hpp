/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-04-10 18:27:46
 * @FilePath: /astar/include/Orientation.hpp
 */

#ifndef ASTAR_INCLUDE_ORIENTATION_H_
#define ASTAR_INCLUDE_ORIENTATION_H_

namespace astar {

enum class Orientation
{
  kNone = 0,
  kEast = 1,
  kWest = 2,
  kSouth = 3,
  kNorth = 4
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_ORIENTATION_H_
