/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 12:03:43
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 12:04:23
 * @FilePath: /astar/include/NodeState.h
 */
#ifndef ASTAR_INCLUDE_NODESTATE_H_
#define ASTAR_INCLUDE_NODESTATE_H_

#include "Coordinate.h"

namespace astar {

enum class NodeState
{
  kNone = 0,
  kOpen = 1,
  kClose = 2
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_NODESTATE_H_