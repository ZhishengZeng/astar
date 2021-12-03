/*
 * @Author: your name
 * @Date: 2021-12-01 18:48:58
 * @LastEditTime: 2021-12-01 18:49:18
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /astar/include/NodeState.h
 */
/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 12:03:43
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-17 12:04:23
 * @FilePath: /astar/include/NodeState.h
 */
#ifndef ASTAR_INCLUDE_SEARCHSTATE_H_
#define ASTAR_INCLUDE_SEARCHSTATE_H_

#include "Coordinate.h"

namespace astar {

enum class SearchState
{
  kNone = 0,
  kOpen = 1,
  kClose = 2
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_SEARCHSTATE_H_