/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 14:10:50
 * @FilePath: /AStar/include/Util.h
 */

#ifndef ASTAR_INCLUDE_UTIL_H_
#define ASTAR_INCLUDE_UTIL_H_
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

namespace astar {

class Util
{
 public:
  static double microtime();
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_UTIL_H_
