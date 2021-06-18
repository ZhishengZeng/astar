/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 19:13:23
 * @LastEditTime: 2021-06-18 19:19:11
 * @LastEditors: ZengZhisheng
 * @Description:
 * @FilePath: /AStar/src/Model.cpp
 */
#include "Model.h"
namespace astar {
void Module::initNodeMat(int length, int width)
{
  // clear
  for (size_t i = 0; i < _node_mat.size(); i++) {
    _node_mat[i].clear();
  }
  _node_mat.clear();
  // init
  _node_mat.resize(length);
  for (int i = 0; i < length; i++) {
    _node_mat[i].resize(width);
  }
  for (size_t i = 0; i < _node_mat.size(); i++) {
    for (size_t j = 0; j < _node_mat[i].size(); j++) {
      _node_mat[i][j].set_coord(i, j);
    }
  }
}

bool isNoneType(Node& node)
{
  return node.get_type() == NodeType::kNone;
}

void Module::initStartPoint(int start_x, int start_y)
{
  Node& node = _node_mat[start_x][start_y];
  if (isNoneType(node)) {
    node.set_type(NodeType::kStart);
    _start_node = &node;
  } else {
    std::cout << "[Error] Set start point type isn't kNone!!" << std::endl;
    exit(1);
  }
}
void Module::initEndPoint(int end_x, int end_y)
{
  Node& node = _node_mat[end_x][end_y];
  if (isNoneType(node)) {
    node.set_type(NodeType::kEnd);
    _end_node = &node;
  } else {
    std::cout << "[Error] Set end point type isn't kNone!!" << std::endl;
    exit(1);
  }
}
void Module::appendObsPoint(int obs_x, int obs_y)
{
  Node& node = _node_mat[obs_x][obs_y];
  if (isNoneType(node)) {
    node.set_type(NodeType::kObs);
  } else {
    std::cout << "[Error] Set obs point type isn't kNone!!" << std::endl;
    exit(1);
  }
}
void printNode(Node& node)
{
  double curr_cost = node.get_known_cost();
  double est_cost = node.get_est_cost();

  switch (node.get_type()) {
    case NodeType::kNone:
      switch (node.get_state()) {
        case NodeState::kNone:
          std::cout << "\33[47m[?+?]\033[0m";
          break;
        case NodeState::kOpen:
          std::cout << "\33[41m[" << curr_cost << "+" << est_cost << "]\033[0m";
          break;
        case NodeState::kClose:
          if (node.get_on_path()) {
            std::cout << "\33[45;30m[" << curr_cost << "+" << est_cost << "]\033[0m";
          } else {
            std::cout << "\33[45m[" << curr_cost << "+" << est_cost << "]\033[0m";
          }
          break;
        default:
          std::cout << "default???";
          break;
      }
      break;
    case NodeType::kObs:
      std::cout << "\33[40m[OBS]\033[0m";
      break;
    case NodeType::kStart:
      std::cout << "\33[42m[" << curr_cost << "+" << est_cost << "]\033[0m";
      break;
    case NodeType::kEnd:
      std::cout << "\33[44m[" << curr_cost << "+" << est_cost << "]\033[0m";
      break;
    default:
      std::cout << "default???";
      break;
  }
}
void Module::printNodeMat()
{
  updatePath();
  std::cout << "--------------------------------------------------\n";
  for (int j = (_node_mat[0].size() - 1); j >= 0; j--) {
    for (size_t i = 0; i < _node_mat.size(); i++) {
      printNode(_node_mat[i][j]);
    }
    std::cout << "\n";
  }
  std::cout << "--------------------------------------------------\n";
  clearPath();
}

void Module::updatePath()
{
  if (_curr_node == nullptr) {
    return;
  }

  Node* temp_node = _curr_node;
  while (temp_node->get_coord() != _start_node->get_coord()) {
    temp_node->set_on_path(true);
    temp_node = temp_node->get_parent_node();
  }
  _start_node->set_on_path(true);
}

void Module::clearPath()
{
  if (_curr_node == nullptr) {
    return;
  }
  Node* temp_node = _curr_node;
  while (temp_node->get_coord() != _start_node->get_coord()) {
    temp_node->set_on_path(false);
    temp_node = temp_node->get_parent_node();
  }
  _start_node->set_on_path(false);
}

void Module::runAStar()
{
  printNodeMat();
  setOpenState(_start_node, _start_node);
  while (true) {
    // 搜索最佳点
    findBestNode();
    // 判断最佳点是否为终点
    if (_curr_node->get_coord() == _end_node->get_coord()) {
      break;
    } else {
      findCandidateCoords();
    }
    printNodeMat();
  }
  printNodeMat();
}

void Module::setOpenState(Node* parent_node, Node* node)
{
  _open_list.push_front(node);
  node->set_state(NodeState::kOpen);
  node->set_known_cost(parent_node->get_known_cost() + 1);
  node->set_est_cost(caculateEstCost(node));
  node->set_parent_node(parent_node);
}

void Module::findBestNode()
{
  std::list<Node*>::iterator min_iter;
  double min_cost = 9999;
  for (std::list<Node*>::iterator itor = _open_list.begin(); itor != _open_list.end(); itor++) {
    if ((*itor)->get_total_cost() < min_cost) {
      min_cost = (*itor)->get_total_cost();
      min_iter = itor;
    }
  }
  _curr_node = *min_iter;
  _open_list.erase(min_iter);
  setCloseState(_curr_node);
}

void Module::setCloseState(Node* node)
{
  _close_list.push_back(node);
  node->set_state(NodeState::kClose);
}

void Module::findCandidateCoords()
{
  std::vector<Node*> neighbor_node_list;
  getNeighborNodes(neighbor_node_list);
  for (size_t i = 0; i < neighbor_node_list.size(); i++) {
    Node* neighbor_node = neighbor_node_list[i];
    if (isSkipNode(neighbor_node)) {
      continue;
    } else {
      if (inOpenList(neighbor_node)) {
        if (neighbor_node->get_total_cost() > (_curr_node->get_total_cost() + 1)) {
          double neighbor_know_cost = neighbor_node->get_known_cost() + 1;
          double neighbor_est_cost = caculateEstCost(neighbor_node);
          neighbor_node->set_known_cost(neighbor_know_cost);
          neighbor_node->set_est_cost(neighbor_est_cost);
          neighbor_node->set_parent_node(_curr_node);
        }
      } else {
        setOpenState(_curr_node, neighbor_node);
      }
    }
  }
}

void Module::getNeighborNodes(std::vector<Node*>& neighbor_node_list)
{
  Coordinate& curr_coord = _curr_node->get_coord();
  int curr_coord_x = curr_coord.get_x();
  int curr_coord_y = curr_coord.get_y();
  if (curr_coord_x > 0) {
    neighbor_node_list.push_back(&_node_mat[curr_coord_x - 1][curr_coord_y]);
  }
  if (curr_coord_x < (int) (_node_mat.size() - 1)) {
    neighbor_node_list.push_back(&_node_mat[curr_coord_x + 1][curr_coord_y]);
  }
  if (curr_coord_y > 0) {
    neighbor_node_list.push_back(&_node_mat[curr_coord_x][curr_coord_y - 1]);
  }
  if (curr_coord_y < (int) (_node_mat[0].size() - 1)) {
    neighbor_node_list.push_back(&_node_mat[curr_coord_x][curr_coord_y + 1]);
  }
}

bool Module::isSkipNode(Node* node)
{
  return (node->get_type() == NodeType::kObs || inCloseList(node));
}

bool Module::inOpenList(Node* node)
{
  for (std::list<Node*>::iterator itor = _open_list.begin(); itor != _open_list.end(); itor++) {
    if ((*itor)->get_coord() == node->get_coord()) {
      return true;
    }
  }
  return false;
}

double Module::caculateEstCost(Node* node)
{
  Coordinate& node_coord = node->get_coord();
  Coordinate& end_node_coord = _end_node->get_coord();
  return std::abs(node_coord.get_x() - end_node_coord.get_x()) + std::abs(node_coord.get_y() - end_node_coord.get_y());
}

bool Module::inCloseList(Node* node)
{
  for (std::list<Node*>::iterator itor = _close_list.begin(); itor != _close_list.end(); itor++) {
    if ((*itor)->get_coord() == node->get_coord()) {
      return true;
    }
  }
  return false;
}
}  // namespace astar
