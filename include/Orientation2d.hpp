/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-24 13:14:37
 * @FilePath: /astar/include/Orientation2d.h
 */

#ifndef ASTAR_INCLUDE_ORIENTATION2D_H_
#define ASTAR_INCLUDE_ORIENTATION2D_H_

namespace astar {

enum class Orientation2d
{
  kNone = 0,
  kHorizontal = 1,
  kVertical = 2,
  kOblique = 3
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_ORIENTATION2D_H_
