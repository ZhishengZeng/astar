/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-11-10 13:54:54
 * @FilePath: /astar/include/GridMap.h
 */

#ifndef ASTAR_INCLUDE_MAT_H_
#define ASTAR_INCLUDE_MAT_H_
#include <malloc.h>

#include <cassert>

namespace astar {

template <typename T>
class GridMap
{
 public:
  explicit GridMap() {}
  explicit GridMap(int x_size, int y_size) { init(x_size, y_size); }
  GridMap(const GridMap& other)
  {
    _x_size = other._x_size;
    _y_size = other._y_size;
    initArray();
    cloneArray(other._array);
  }
  GridMap(GridMap&& other)
  {
    _x_size = std::move(other._x_size);
    _y_size = std::move(other._y_size);
    _array = std::move(other._array);
    other._array = nullptr;
  }
  ~GridMap() { free(); }
  GridMap& operator=(const GridMap& other)
  {
    freeArray();
    _x_size = other._x_size;
    _y_size = other._y_size;
    initArray();
    cloneArray(other._array);
    return (*this);
  }
  GridMap& operator=(GridMap&& other)
  {
    freeArray();
    _x_size = std::move(other._x_size);
    _y_size = std::move(other._y_size);
    _array = std::move(other._array);
    other._array = nullptr;
    return (*this);
  }
  T* operator[](const int i)
  {
    assert(0 <= i && i < _x_size);
    return _array[i];
  }
  // getter
  int get_x_size() const { return _x_size; }
  int get_y_size() const { return _y_size; }
  // setter

  // function
  void init(int x_size, int y_size);
  void free();
  bool isEmpty();
  bool inScope(int x, int y);

 private:
  int _x_size = 0;
  int _y_size = 0;
  T** _array = nullptr;
  // function
  void initByValue(int x_size, int y_size, T value);
  void initArray();
  void cloneArray(T** other_array);
  void freeArray();
  void refreshValue(T value);
};

template <typename T>
inline void GridMap<T>::init(int x_size, int y_size)
{
  if constexpr (std::is_same<T, int>::value) {
    initByValue(x_size, y_size, 0);
  } else if constexpr (std::is_same<T, double>::value) {
    initByValue(x_size, y_size, 0.0);
  } else {
    T value;
    initByValue(x_size, y_size, value);
  }
}

template <typename T>
void GridMap<T>::free()
{
  _x_size = 0;
  _y_size = 0;
  freeArray();
}

template <typename T>
inline bool GridMap<T>::isEmpty()
{
  return _x_size == 0 || _y_size == 0;
}

template <typename T>
inline bool GridMap<T>::inScope(int x, int y)
{
  return 0 <= x && x < _x_size && 0 <= y && y < _y_size;
}

template <typename T>
inline void GridMap<T>::cloneArray(T** other_array)
{
  for (int i = 0; i < _x_size; i++) {
    for (int j = 0; j < _y_size; j++) {
      _array[i][j] = other_array[i][j];
    }
  }
}

template <typename T>
inline void GridMap<T>::initByValue(int x_size, int y_size, T value)
{
  freeArray();
  _x_size = x_size;
  _y_size = y_size;
  initArray();
  refreshValue(value);
}

template <typename T>
inline void GridMap<T>::freeArray()
{
  if (_array) {
    delete[] _array[0];
  }
  delete[] _array;
  _array = nullptr;
}

template <typename T>
inline void GridMap<T>::initArray()
{
  assert(_x_size >= 0 && _y_size >= 0);
  if (_x_size == 0 || _y_size == 0) {
    _array = nullptr;
  }
  _array = new T*[_x_size];
  _array[0] = new T[_x_size * _y_size];
  for (int i = 1; i < _x_size; i++) {
    _array[i] = _array[i - 1] + _y_size;
  }
}

template <typename T>
inline void GridMap<T>::refreshValue(T value)
{
  for (int i = 0; i < _x_size; i++) {
    for (int j = 0; j < _y_size; j++) {
      _array[i][j] = value;
    }
  }
}

}  // namespace astar

#endif  // ISR_GRIDMAP_H_