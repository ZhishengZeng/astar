/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 12:03:44
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 15:20:33
 * @FilePath: /AStar/include/NodeType.h
 */
#ifndef ASTAR_INCLUDE_NODETYPE_H_
#define ASTAR_INCLUDE_NODETYPE_H_

#include "Coordinate.h"

namespace astar {

enum class NodeType
{
  kNone = 0,
  kStart = 1,
  kEnd = 2,
  kHObs = 3,
  kVObs = 4,
  kOmniObs = 5
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_NODETYPE_H_