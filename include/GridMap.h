/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 19:56:49
 * @Description:
 * @FilePath: /AStar/include/GridMap.h
 */

#ifndef ASTAR_INCLUDE_MAT_H_
#define ASTAR_INCLUDE_MAT_H_
#include "malloc.h"
namespace astar {

template <typename T>
class GridMap
{
 private:
  int _x_grids = 0;
  int _y_grids = 0;
  T** _t = nullptr;

 public:
  GridMap() {}
  ~GridMap() {}

  T* operator[](int i) { return _t[i]; }
  // getter
  int get_x_grids() const { return _x_grids; }
  int get_y_grids() const { return _y_grids; }
  // setter

  // function
  void resize(int x_grids, int y_grids)
  {
    _t = new T*[x_grids];
    _t[0] = new T[x_grids * y_grids];
    for (int i = 1; i < x_grids; i++) {
      _t[i] = _t[i - 1] + y_grids;
    }

    _x_grids = x_grids;
    _y_grids = y_grids;
  }
};
}  // namespace astar

#endif  // ASTAR_INCLUDE_MAT_H_