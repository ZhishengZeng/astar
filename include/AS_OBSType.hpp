/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-17 12:03:44
 * @Description:
 * @LastEditors: ZhishengZeng zhishengzeng@163.com
 * @LastEditTime: 2022-06-08 12:17:29
 * @FilePath: /astar/include/OBSType.h
 */
#ifndef ASTAR_INCLUDE_OBSTYPE_H_
#define ASTAR_INCLUDE_OBSTYPE_H_

namespace astar {

enum class OBSType
{
  kNone = 0,
  kEastOBS = 1,
  kWestOBS = 2,
  kSouthOBS = 3,
  kNorthOBS = 4
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_OBSTYPE_H_