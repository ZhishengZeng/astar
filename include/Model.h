/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 16:19:28
 * @LastEditTime: 2021-06-18 19:19:34
 * @LastEditors: ZengZhisheng
 * @Description:
 * @FilePath: /AStar/include/Model.h
 */
#ifndef ASTAR_INCLUDE_MODEL_H_
#define ASTAR_INCLUDE_MODEL_H_

#include <list>
#include <vector>

#include "Mat.h"
#include "Node.h"
namespace astar {

class Model
{
 private:
  Mat<Node> _map;
  std::list<Node*> _open_list;
  std::list<Node*> _close_list;
  Node* _start_node = nullptr;
  Node* _end_node = nullptr;
  Node* _curr_node = nullptr;

 public:
};

}  // namespace astar

#endif  // ASTAR_INCLUDE_MODEL_H_