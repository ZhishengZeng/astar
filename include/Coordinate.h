/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-05 21:50:09
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-01-25 19:31:34
 * @FilePath: /astar/include/Coordinate.h
 */

#ifndef ASTAR_INCLUDE_COORDINATE_H_
#define ASTAR_INCLUDE_COORDINATE_H_

namespace astar {

class Coordinate
{
 public:
  Coordinate() {}
  Coordinate(int x, int y) : _x(x), _y(y) {}
  ~Coordinate() {}

  bool operator==(Coordinate& other) { return (_x == other._x && _y == other._y); }
  bool operator!=(Coordinate& other) { return !((*this) == other); }
  // getter
  int get_x() const { return _x; }
  int get_y() const { return _y; }
  // setter
  void set_x(const int x) { _x = x; }
  void set_y(const int y) { _y = y; }

 private:
  int _x = 0;
  int _y = 0;
};

struct cmpCoordinate
{
  bool operator()(Coordinate a, Coordinate b) const
  {
    return a.get_x() != b.get_x() ? a.get_x() < b.get_x() : a.get_y() < b.get_y();
  }
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_COORDINATE_H_