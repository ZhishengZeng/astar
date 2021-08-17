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
    _x_size = other._x_size;
    _y_size = other._y_size;
    _array = cloneArray(other._array, other._x_size, other._y_size);
  }
  GridMap(GridMap&& other)
  {
    _x_size = other._x_size;
    _y_size = other._y_size;
    _array = other._array;
    other._array = nullptr;
  }
  ~GridMap() { freeArray(_array); }
  GridMap& operator=(const GridMap& other)
  {
    _x_size = other._x_size;
    _y_size = other._y_size;
    _array = cloneArray(other._array, other._x_size, other._y_size);
    return (*this);
  }
  GridMap& operator=(GridMap&& other)
  {
    _x_size = other._x_size;
    _y_size = other._y_size;
    _array = other._array;
    other._array = nullptr;
    return (*this);
  }
  T* operator[](const size_t i) { return _array[i]; }
  // getter
  size_t get_x_size() const { return _x_size; }
  size_t get_y_size() const { return _y_size; }
  // setter

  // function
  void init(size_t x_size = 0, size_t y_size = 0);
  T** initArray(size_t x_size, size_t y_size);
  T** cloneArray(T** other_array, size_t x_size, size_t y_size);
  void freeArray(T** array);
  bool isEmpty();

 private:
  size_t _x_size = 0;
  size_t _y_size = 0;
  T** _array = nullptr;
};

template <typename T>
inline void GridMap<T>::init(size_t x_size, size_t y_size)
{
  _x_size = x_size;
  _y_size = y_size;
  _array = initArray(x_size, y_size);
}

template <typename T>
inline T** GridMap<T>::initArray(size_t x_size, size_t y_size)
{
  T** array = new T*[x_size];
  array[0] = new T[x_size * y_size];
  for (size_t i = 1; i < x_size; i++) {
    array[i] = array[i - 1] + y_size;
  }
  return array;
}

template <typename T>
inline T** GridMap<T>::cloneArray(T** other_array, size_t x_size, size_t y_size)
{
  T** array = initArray(x_size, y_size);
  for (size_t i = 0; i < x_size; i++) {
    for (size_t j = 0; j < y_size; j++) {
      array[i][j] = other_array[i][j];
    }
  }
  return array;
}

template <typename T>
inline void GridMap<T>::freeArray(T** array)
{
  delete[] array[0];
  array[0] = nullptr;
  for (size_t i = 1; i < _x_size; i++) {
    array[i] = nullptr;
  }
  delete array;
  array = nullptr;
}

template <typename T>
inline bool GridMap<T>::isEmpty()
{
  return _x_size == 0 || _y_size == 0;
}

}  // namespace astar

#endif  // ASTAR_INCLUDE_MAT_H_