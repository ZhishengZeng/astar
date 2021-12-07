/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 12:03:44
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-06 15:03:09
 * @FilePath: /astar/include/ObsType.h
 */
#ifndef ASTAR_INCLUDE_OBSTYPE_H_
#define ASTAR_INCLUDE_OBSTYPE_H_

namespace astar {

enum class ObsType
{
  kNone = 0,
  kHLeftObs = 1,
  kHRightObs = 2,
  kVBottomObs = 3,
  kVTopObs = 4
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_OBSTYPE_H_