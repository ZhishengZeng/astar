/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-19 16:31:43
 * @Description:
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
