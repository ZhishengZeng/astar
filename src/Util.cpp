/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-04-08 11:38:27
 * @FilePath: /astar/src/Util.cpp
 */

#include "Util.hpp"

namespace astar {

double Util::microtime()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);
  return tv.tv_sec + tv.tv_usec / 1000000.00;
}

}  // namespace astar
