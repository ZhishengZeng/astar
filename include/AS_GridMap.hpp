/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-04-28 21:44:17
 * @FilePath: /astar/include/AS_GridMap.hpp
 */

#ifndef ASTAR_INCLUDE_GRIDMAP_H_
#define ASTAR_INCLUDE_GRIDMAP_H_

#include <cassert>

namespace astar {

template <typename T>
class GridMap
{
 public:
  explicit GridMap() {}
  explicit GridMap(int x_size, int y_size) { init(x_size, y_size); }
  explicit GridMap(int x_size, int y_size, T value) { init(x_size, y_size, value); }
  GridMap(const GridMap& other)
  {
    _x_size = other._x_size;
    _y_size = other._y_size;
    initMap();
    copyMap(other._map);
  }
  GridMap(GridMap&& other)
  {
    _x_size = std::move(other._x_size);
    _y_size = std::move(other._y_size);
    _map = std::move(other._map);
    other._map = nullptr;
  }
  ~GridMap() { free(); }
  GridMap& operator=(const GridMap& other)
  {
    freeMap();
    _x_size = other._x_size;
    _y_size = other._y_size;
    initMap();
    copyMap(other._map);
    return (*this);
  }
  GridMap& operator=(GridMap&& other)
  {
    freeMap();
    _x_size = std::move(other._x_size);
    _y_size = std::move(other._y_size);
    _map = std::move(other._map);
    other._map = nullptr;
    return (*this);
  }

  template <typename U>
  class Proxy
  {
   public:
    Proxy(int y_size, U* array) : _y_size(y_size), _array(array) {}
    U& operator[](const int i)
    {
      if (i < 0 || _y_size <= i) {
        std::cout << "[GridMap<T> Error] The grid map index y is out of bounds!" << std::endl;
        assert(false);
      }
      return _array[i];
    }

   private:
    int _y_size = 0;
    U* _array = nullptr;
  };

  Proxy<T> operator[](const int i)
  {
    if (i < 0 && _x_size <= i) {
      std::cout << "[GridMap<T> Error] The grid map index x is out of bounds!" << std::endl;
      assert(false);
    }
    return Proxy<T>(_y_size, _map[i]);
  }
  // getter
  int get_x_size() const { return _x_size; }
  int get_y_size() const { return _y_size; }
  // setter

  // function
  void init(int x_size, int y_size);
  void init(int x_size, int y_size, T value);
  void free();
  bool isEmpty();
  bool inScope(int x, int y);

 private:
  int _x_size = 0;
  int _y_size = 0;
  T** _map = nullptr;
  // function
  void initMap();
  void copyMap(T** other_map);
  void freeMap();
  void deassign(T value);
};

// public

template <typename T>
inline void GridMap<T>::init(int x_size, int y_size)
{
  if constexpr (std::is_same<T, int>::value) {
    init(x_size, y_size, 0);
  } else if constexpr (std::is_same<T, double>::value) {
    init(x_size, y_size, 0.0);
  } else {
    T value;
    init(x_size, y_size, value);
  }
}

template <typename T>
inline void GridMap<T>::init(int x_size, int y_size, T value)
{
  freeMap();
  _x_size = x_size;
  _y_size = y_size;
  initMap();
  deassign(value);
}

template <typename T>
void GridMap<T>::free()
{
  _x_size = 0;
  _y_size = 0;
  freeMap();
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

// private

template <typename T>
inline void GridMap<T>::initMap()
{
  if (_x_size < 0 || _y_size < 0) {
    std::cout << "[GridMap<T> Error] The map size setting error!" << std::endl;
    assert(false);
  }
  if (_x_size == 0 || _y_size == 0) {
    _map = nullptr;
  }
  _map = new T*[_x_size];
  _map[0] = new T[_x_size * _y_size];
  for (int i = 1; i < _x_size; i++) {
    _map[i] = _map[i - 1] + _y_size;
  }
}

template <typename T>
inline void GridMap<T>::copyMap(T** other_map)
{
  for (int i = 0; i < _x_size; i++) {
    for (int j = 0; j < _y_size; j++) {
      _map[i][j] = other_map[i][j];
    }
  }
}

template <typename T>
inline void GridMap<T>::freeMap()
{
  if (_map) {
    delete[] _map[0];
  }
  delete[] _map;
  _map = nullptr;
}

template <typename T>
inline void GridMap<T>::deassign(T value)
{
  for (int i = 0; i < _x_size; i++) {
    for (int j = 0; j < _y_size; j++) {
      _map[i][j] = value;
    }
  }
}

}  // namespace astar

#endif  // ASTAR_INCLUDE_GRIDMAP_H_