/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 12:03:44
 * @Description:
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-01 18:05:27
 * @FilePath: /astar/include/ObsType.h
 */
#ifndef ASTAR_INCLUDE_OBSTYPE_H_
#define ASTAR_INCLUDE_OBSTYPE_H_

namespace astar {

enum class ObsType
{
  kNone = 0,
  kHObs = 1,
  kVObs = 2,
  kOmniObs = 3
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_OBSTYPE_H_