/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-15 19:51:01
 * @FilePath: /AStar/src/Model.cpp
 */
#include "Model.h"

#include "Util.h"
namespace astar {

void Model::buildMap(int x_size, int y_size)
{
  assert(x_size > 0 && y_size > 0);

  _grid_map.init(x_size, y_size);
  for (int i = 0; i < (int) _grid_map.get_x_size(); i++) {
    for (int j = 0; j < (int) _grid_map.get_y_size(); j++) {
      _grid_map[i][j].set_coord(i, j);
    }
  }
}

void Model::addNodeCost(const std::pair<Coordinate, double>& coord_cost)
{
  _coord_cost_list.push_back(coord_cost);
}

void Model::addObstacle(const Coordinate& obs_coord)
{
  _obs_coord_list.push_back(obs_coord);
}

void Model::enableDiagonalRouting()
{
  _routing_diagonal = true;
}

void Model::disableDiagonalRouting()
{
  _routing_diagonal = false;
}

void Model::enableTurningBack()
{
  _turning_back = true;
}

void Model::disableTurningBack()
{
  _turning_back = false;
}

std::vector<Coordinate> Model::findPath(const Coordinate& start_coord, const Coordinate& end_coord)
{
  std::vector<Coordinate> path_coord;

  addCostToMap();
  addObsToMap();
  setStartNode(start_coord);
  setEndNode(end_coord);
  initStartNode();
  initOffsetList();
  while (true) {
    // 获取当前最优结果点
    getMinCostNodeInOpenList();
    // 判断是否抵达终点
    if (_curr_node->isEnd()) {
      break;
    }
    // 搜寻备选下一步节点
    addNeighborNodesToOpenList();
    // 无路可走
    if (_open_list.empty()) {
      break;
    }
  }
#if SHOWRESULT
  showResult();
#endif
  reportResult();
  return getPathCoord();
}

void Model::addCostToMap()
{
  std::map<Coordinate, double, cmpCoordinate> coord_cost_map;
  std::map<Coordinate, double>::iterator iter;
  for (size_t i = 0; i < _coord_cost_list.size(); i++) {
    const Coordinate& coord = _coord_cost_list[i].first;
    double cost = _coord_cost_list[i].second;

    iter = coord_cost_map.find(coord);
    if (iter != coord_cost_map.end()) {
      coord_cost_map[coord] += cost;
    } else {
      coord_cost_map.insert(std::map<Coordinate, double>::value_type(coord, cost));
    }
  }

  for (size_t x = 0; x < _grid_map.get_x_size(); x++) {
    for (size_t y = 0; y < _grid_map.get_y_size(); y++) {
      Coordinate coord(x, y);
      if (coord_cost_map.find(coord) == coord_cost_map.end()) {
        coord_cost_map.insert(std::map<Coordinate, double>::value_type(coord, 0));
      }
    }
  }

  std::vector<double> cost_list;
  for (iter = coord_cost_map.begin(); iter != coord_cost_map.end(); iter++) {
    cost_list.push_back(iter->second);
  }

  std::sort(cost_list.begin(), cost_list.end());

  // Method 1
  _min_node_cost = cost_list.front();
  // Method 2

  // Method 3

  for (iter = coord_cost_map.begin(); iter != coord_cost_map.end(); iter++) {
    const Coordinate& coord = iter->first;
    _grid_map[coord.get_x()][coord.get_y()].set_self_cost(iter->second);
  }
}

void Model::addObsToMap()
{
  for (size_t i = 0; i < _obs_coord_list.size(); i++) {
    setNode(_obs_coord_list[i], NodeType::kObs);
  }
}

Node* Model::setNode(const Coordinate& coord, const NodeType& node_type)
{
  assert(0 <= coord.get_x() && coord.get_x() < (int) _grid_map.get_x_size());
  assert(0 <= coord.get_y() && coord.get_y() < (int) _grid_map.get_y_size());

  Node& node = _grid_map[coord.get_x()][coord.get_y()];
  if (node.get_type() == NodeType::kNone || node.get_type() == NodeType::kObs) {
    node.set_type(node_type);
    return &node;
  } else {
    std::cout << "[AStar Error] [" << coord.get_x() << " , " << coord.get_y()
              << "] type is not null!!" << std::endl;
    exit(1);
  }
}

void Model::setStartNode(const Coordinate& coord)
{
  _start_node = setNode(coord, NodeType::kStart);
}

void Model::setEndNode(const Coordinate& coord)
{
  _end_node = setNode(coord, NodeType::kEnd);
}

void Model::initStartNode()
{
  _curr_node = _start_node;
  updateEstCost(_start_node);
  updateParentByCurr(_start_node);
  addNodeToOpenList(_start_node);
}

void Model::updateEstCost(Node* node)
{
  node->set_est_cost(caculateEstCost(node, _end_node));
}

double Model::caculateEstCost(Node* a, Node* b)
{
  int wire_length = 0;
  int length = std::abs(a->get_coord().get_x() - b->get_coord().get_x());
  int width = std::abs(a->get_coord().get_y() - b->get_coord().get_y());
  if (_routing_diagonal) {
    wire_length = (std::abs(length - width) + (length < width ? length : width) * 1.414);
  } else {
    wire_length = (length + width);
  }
  return (wire_length + wire_length * _min_node_cost);
}

void Model::updateParentByCurr(Node* node)
{
  node->set_known_cost(_curr_node->get_known_cost() + getCurrWalkingCost(node));
  node->set_parent_node(_curr_node);
}

double Model::getCurrWalkingCost(Node* node)
{
  Coordinate& node_coord = node->get_coord();
  Coordinate& curr_coord = _curr_node->get_coord();
  double walking_cost = 1;
  if (node_coord.get_x() != curr_coord.get_x()
      && node_coord.get_y() != curr_coord.get_y()) {
    walking_cost = 1.414;
  }
  walking_cost += node->get_self_cost();
  return walking_cost;
}

void Model::addNodeToOpenList(Node* node)
{
  _open_list.push(node);
  node->set_state(NodeState::kOpen);
}

void Model::initOffsetList()
{
  int x_offset
      = _end_node->get_coord().get_x() - _start_node->get_coord().get_x();
  int y_offset
      = _end_node->get_coord().get_y() - _start_node->get_coord().get_y();
  x_offset = (x_offset != 0 ? x_offset / std::abs(x_offset) : x_offset);
  y_offset = (y_offset != 0 ? y_offset / std::abs(y_offset) : y_offset);

  if (x_offset != 0) {
    _offset_list.emplace_back(x_offset, 0);
    if (_turning_back) {
      _offset_list.emplace_back(-1 * x_offset, 0);
    }
  }

  if (y_offset != 0) {
    _offset_list.emplace_back(0, y_offset);
    if (_turning_back) {
      _offset_list.emplace_back(0, -1 * y_offset);
    }
  }

  if (_routing_diagonal && x_offset != 0 && y_offset != 0) {
    _offset_list.emplace_back(x_offset, y_offset);
    if (_turning_back) {
      _offset_list.emplace_back(-1 * x_offset, y_offset);
      _offset_list.emplace_back(x_offset, -1 * y_offset);
      _offset_list.emplace_back(-1 * x_offset, -1 * y_offset);
    }
  }
  std::sort(_offset_list.begin(), _offset_list.end(), cmpCoordinate());
  std::vector<astar::Coordinate>::iterator erase_iter;
  erase_iter = std::unique(_offset_list.begin(), _offset_list.end());
  _offset_list.erase(erase_iter, _offset_list.end());
}

void Model::getMinCostNodeInOpenList()
{
  _curr_node = _open_list.top();
  _open_list.pop();
  _curr_node->set_state(NodeState::kClose);
}

void Model::addNeighborNodesToOpenList()
{
  std::vector<Node*> neighbor_node_list;
  getNeighborNodesByCurr(neighbor_node_list);
  for (Node* neighbor_node : neighbor_node_list) {
    if (neighbor_node->isClose() || neighbor_node->isObs()) {
      continue;
    }
    if (neighbor_node->isOpen()) {
      if (isCurrBetterParent(neighbor_node)) {
        updateParentByCurr(neighbor_node);
      }
    } else {
      updateEstCost(neighbor_node);
      updateParentByCurr(neighbor_node);
      addNodeToOpenList(neighbor_node);
    }
  }
}

void Model::getNeighborNodesByCurr(std::vector<Node*>& neighbor_node_list)
{
  Coordinate& curr_coord = _curr_node->get_coord();
  int curr_coord_x = curr_coord.get_x();
  int curr_coord_y = curr_coord.get_y();

  for (size_t i = 0; i < _offset_list.size(); i++) {
    int x = curr_coord_x + _offset_list[i].get_x();
    int y = curr_coord_y + _offset_list[i].get_y();
    if (!isLegalNeighbor(x, y)) {
      continue;
    }
    neighbor_node_list.push_back(&_grid_map[x][y]);
  }
}

bool Model::isLegalNeighbor(int x, int y)
{
  Coordinate& curr_coord = _curr_node->get_coord();
  if (x == curr_coord.get_x() && y == curr_coord.get_y()) {
    return false;
  }
  return (0 <= x && x < (int) _grid_map.get_x_size())
         && (0 <= y && y < (int) _grid_map.get_y_size());
}

bool Model::isCurrBetterParent(Node* node)
{
  return (_curr_node->get_known_cost() + getCurrWalkingCost(node))
         < node->get_known_cost();
}

void Model::showResult()
{
  if (_grid_map.get_x_size() > 50 || _grid_map.get_y_size() > 50) {
    std::cout
        << "[AStar Warning] Plot large map(larger than 50x50) , skipping..."
        << std::endl;
    return;
  }
  setOnPath(true);
  printGridMap();
  setOnPath(false);
}

void Model::setOnPath(const bool on_path)
{
  if (_curr_node == nullptr) {
    return;
  }
  Node* temp_node = _curr_node;
  do {
    temp_node->set_on_path(on_path);
    temp_node = temp_node->get_parent_node();
  } while (!temp_node->isStart());
}

void Model::printGridMap()
{
  std::cout << "--------------------------------------------------\n";
  for (int j = _grid_map.get_y_size() - 1; j >= 0; j--) {
    for (int i = 0; i < (int) _grid_map.get_x_size(); i++) {
      printNode(_grid_map[i][j]);
    }
    std::cout << "\n";
  }
  std::cout << "--------------------------------------------------\n";
}

void Model::printNode(Node& node)
{
  double curr_cost = node.get_known_cost();
  double est_cost = node.get_est_cost();

  switch (node.get_type()) {
    case NodeType::kNone:
      switch (node.get_state()) {
        case NodeState::kNone:
          printf("\33[47m[%05.2lf+%05.2lf]\033[0m", 0.0, 0.0);
          break;
        case NodeState::kOpen:
          printf("\33[41m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
          break;
        case NodeState::kClose:
          if (node.get_on_path()) {
            printf("\33[45;30m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
          } else {
            printf("\33[45m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
          }
          break;
        default:
          std::cout << "[AStar Error] Node type not recognized!" << std::endl;
          exit(1);
      }
      break;
    case NodeType::kObs:
      printf("\33[40m[%05.2lf+%05.2lf]\033[0m", 0.0, 0.0);
      break;
    case NodeType::kStart:
      printf("\33[42m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
      break;
    case NodeType::kEnd:
      printf("\33[44m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
      break;
    default:
      std::cout << "[AStar Error] Node type not recognized!" << std::endl;
      exit(1);
  }
}

void Model::reportResult()
{
  if (_curr_node->isEnd()) {
    std::cout << "[AStar Info] Reached the end node!! Path cost:"
              << _curr_node->get_total_cost();
  } else {
    std::cout << "[AStar Info] Can't reach the end node!!";
  }
  std::cout << std::endl;
}

std::vector<Coordinate> Model::getPathCoord()
{
  std::vector<Coordinate> path_coord;

  if (_curr_node == nullptr || !_curr_node->isEnd()) {
    return path_coord;
  }

  path_coord.push_back(_end_node->get_coord());
  Direction curr_direction
      = getDirection(_curr_node, _curr_node->get_parent_node());

  Node* temp_node = _curr_node;
  do {
    Direction direction = getDirection(temp_node, temp_node->get_parent_node());
    if (curr_direction != direction) {
      curr_direction = direction;
      path_coord.push_back(temp_node->get_coord());
    }
    temp_node = temp_node->get_parent_node();

  } while (!temp_node->isStart());

  path_coord.push_back(_start_node->get_coord());

  for (size_t i = 0, j = (path_coord.size() - 1); i < j; i++, j--) {
    std::swap(path_coord[i], path_coord[j]);
  }

  return path_coord;
}

Direction Model::getDirection(Node* start_node, Node* end_node)
{
  Coordinate& start_coord = start_node->get_coord();
  Coordinate& end_coord = end_node->get_coord();
  if (isHorizontal(start_coord, end_coord)) {
    return Direction::kHorizontal;
  } else if (isVertical(start_coord, end_coord)) {
    return Direction::kVertical;
  } else {
    return Direction::kDiagonal;
  }
}

bool Model::isHorizontal(Coordinate& start_coord, Coordinate& end_coord)
{
  return start_coord.get_y() == end_coord.get_y();
}

bool Model::isVertical(Coordinate& start_coord, Coordinate& end_coord)
{
  return start_coord.get_x() == end_coord.get_x();
}

}  // namespace astar
