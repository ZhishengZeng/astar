/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 17:54:46
 * @LastEditTime: 2021-06-18 19:08:53
 * @LastEditors: ZengZhisheng
 * @Description: a map
 * @FilePath: /AStar/include/Mat.h
 */
#ifndef ASTAR_INCLUDE_MAT_H_
#define ASTAR_INCLUDE_MAT_H_

namespace astar {
template <typename T>
class Mat
{
 private:
  int _length;
  int _width;
  std::vector<std::vector<T>> _mat;

 public:
  Mat(int length, int width) : _length(length), _width(width) { init(length, width); };
  ~Mat();
  // getter

  // setter

  // function
  init(int length, int width);
};
}  // namespace astar

#endif  // ASTAR_INCLUDE_MAT_H_