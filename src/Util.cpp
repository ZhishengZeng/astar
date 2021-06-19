/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-19 16:31:43
 * @Description:
 * @FilePath: /AStar/util/Util.cpp
 */

#include "Util.h"

namespace astar {

double Util::microtime()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);
  return tv.tv_sec + tv.tv_usec / 1000000.00;
}

}  // namespace astar
