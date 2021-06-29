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
 public:
  GridMap() {}
  GridMap(const GridMap& other)
  {
    _x_grid_num = other._x_grid_num;
    _y_grid_num = other._y_grid_num;
    initArray();
    for (size_t i = 0; i < other._x_grid_num; i++) {
      for (size_t j = 0; j < other._y_grid_num; j++) {
        _array[i][j] = other._array[i][j];
      }
    }
  }
  GridMap(GridMap&& other)
  {
    _x_grid_num = other._x_grid_num;
    _y_grid_num = other._y_grid_num;
    _array = other._array;
    other._array = nullptr;
  }
  ~GridMap() { freeArray(); }
  GridMap& operator=(const GridMap& other)
  {
    _x_grid_num = other._x_grid_num;
    _y_grid_num = other._y_grid_num;
    initArray();
    for (size_t i = 0; i < other._x_grid_num; i++) {
      for (size_t j = 0; j < other._y_grid_num; j++) {
        _array[i][j] = other._array[i][j];
      }
    }
    return (*this);
  }
  GridMap& operator=(GridMap&& other)
  {
    _x_grid_num = other._x_grid_num;
    _y_grid_num = other._y_grid_num;
    _array = other._array;
    other._array = nullptr;
    return (*this);
  }
  T* operator[](const size_t i) { return _array[i]; }
  // getter
  size_t get_x_grid_num() const { return _x_grid_num; }
  size_t get_y_grid_num() const { return _y_grid_num; }
  // setter

  // function
  void init(size_t x_grid_num = 0, size_t y_grid_num = 0);
  void initArray();
  void freeArray();

 private:
  size_t _x_grid_num = 0;
  size_t _y_grid_num = 0;
  T** _array = nullptr;
};

template <typename T>
inline void GridMap<T>::init(size_t x_grid_num, size_t y_grid_num)
{
  _x_grid_num = x_grid_num;
  _y_grid_num = y_grid_num;
  initArray();
}

template <typename T>
inline void GridMap<T>::initArray()
{
  freeArray();
  _array = new T*[_x_grid_num];
  _array[0] = new T[_x_grid_num * _y_grid_num];
  for (size_t i = 1; i < _x_grid_num; i++) {
    _array[i] = _array[i - 1] + _y_grid_num;
  }
}

template <typename T>
inline void GridMap<T>::freeArray()
{
  delete[] _array;
  _array = nullptr;
}
}  // namespace astar

#endif  // ASTAR_INCLUDE_MAT_H_