/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-24 23:55:37
 * @FilePath: /astar/include/Util.h
 */

#ifndef ASTAR_INCLUDE_UTIL_H_
#define ASTAR_INCLUDE_UTIL_H_
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <map>
#include <set>

namespace astar {

class Util
{
 public:
  static double microtime();

  template <typename Key, typename Compare = std::less<Key>>
  static bool exist(const std::set<Key, Compare>& set, const Key& key)
  {
    return (set.find(key) != set.end());
  }

  template <typename Key, typename Value, typename Compare = std::less<Key>>
  static bool exist(const std::map<Key, Value, Compare>& map, const Key& key)
  {
    return (map.find(key) != map.end());
  }
};

}  // namespace astar
#endif  // ASTAR_INCLUDE_UTIL_H_
