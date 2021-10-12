/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-10-12 21:51:28
 * @FilePath: /astar/src/Model.cpp
 */
#include "Model.h"

#include "Util.h"
namespace astar {

/**
 * @description: 建立一个x_size*y_size的grid_map
 * @param {int} x_size
 * @param {int} y_size
 * @return {*}
 */
void Model::buildMap(int x_size, int y_size)
{
  _config.set_map_x_size(x_size);
  _config.set_map_y_size(y_size);
}

/**
 * @description: Add node cost. Node cost is 0 by default
 * @param {Coordinate} coord
 * @param {double} cost
 * @return {*}
 */
void Model::addNodeCost(const Coordinate& coord, const double cost)
{
  std::vector<std::pair<Coordinate, double>>& coord_cost_list = _config.get_coord_cost_list();
  coord_cost_list.emplace_back(coord, cost);
}

/**
 * @description: Add obstacle node
 *
 *               Horizontal-OBS        type_flag='H'
 *               Vertical-OBS          type_flag='V'
 *               Omnidirectional-OBS   type_flag='O'
 *
 * @param {Coordinate} coord
 * @param {char} type_flag
 * @return {*}
 */
void Model::addObstacle(const Coordinate& coord, NodeType type)
{
  std::vector<std::pair<Coordinate, NodeType>>& coord_type_list = _config.get_coord_type_list();
  coord_type_list.emplace_back(coord, type);
}

/**
 * @description: log verbose
 *
 *               0    silence(default)
 *               1    report wire length only
 *               2    report wire length & plot routing info
 *
 * @param {int} level
 * @return {*}
 */
void Model::setLogVerbose(int level = 0)
{
  _config.set_log_verbose(level);
}

/**
 * @description: Oblique routing is allowed
 * @param {*}
 * @return {*}
 */
void Model::enableDiagonalRouting()
{
  _config.set_routing_diagonal(true);
}

/**
 * @description: Oblique routing is not allowed
 * @param {*}
 * @return {*}
 */
void Model::disableDiagonalRouting()
{
  _config.set_routing_diagonal(false);
}

/**
 * @description: Turn back line is allowed
 * @param {*}
 * @return {*}
 */
void Model::enableTurningBack()
{
  _config.set_turning_back(true);
}

/**
 * @description: Turn back line is not allowed
 * @param {*}
 * @return {*}
 */
void Model::disableTurningBack()
{
  _config.set_turning_back(false);
}

/**
 * @description: Returns multiple key points on the path
 *
 *                        (end_point)
 *                            e
 *                            │
 *                            │
 *                   x2 ──── x3          return {s x1 x2 x3 e}
 *                   │
 *                   │
 *         s ────── x1
 *   (start_point)
 *
 * @param {Coordinate} start_coord
 * @param {Coordinate} end_coord
 * @return {std::vector<Coordinate>}
 */
std::vector<Coordinate> Model::getPath(const Coordinate& start_coord, const Coordinate& end_coord)
{
  init(start_coord, end_coord);
  while (true) {
    // 获取当前最优结果点
    updateOptimalNode();
    // 抵达终点
    if (_optimal_node->isEndPoint()) {
      break;
    }
    // 搜寻备选下一步节点
    expandSearchQueue();
    // 无路可走
    if (_search_queue.empty()) {
      break;
    }
  }

  int log_verbose = _config.get_log_verbose();
  if (log_verbose > 0) {
    reportResult();
  }
  if (log_verbose > 1) {
    showResult();
  }

  return getPathCoord();
}

void Model::init(const Coordinate& start_coord, const Coordinate& end_coord)
{
  initGridMap();
  addStartNodeToGridMap(start_coord);
  addEndNodeToGridMap(end_coord);
  addCostToGridMap();
  addObsToGridMap();
  initStartNode();
  initOffsetList();
}

void Model::initGridMap()
{
  _grid_map.init(_config.get_map_x_size(), _config.get_map_y_size());
  for (int i = 0; i < (int) _grid_map.get_x_size(); i++) {
    for (int j = 0; j < (int) _grid_map.get_y_size(); j++) {
      _grid_map[i][j].set_coord(i, j);
    }
  }
}

void Model::addStartNodeToGridMap(const Coordinate& coord)
{
  _start_node = setNode(coord, NodeType::kStart);
}

void Model::addEndNodeToGridMap(const Coordinate& coord)
{
  _end_node = setNode(coord, NodeType::kEnd);
}

void Model::addCostToGridMap()
{
  std::vector<std::pair<Coordinate, double>>& coord_cost_list = _config.get_coord_cost_list();
  std::map<Coordinate, double, cmpCoordinate> coord_cost_map;
  std::map<Coordinate, double>::iterator iter;
  for (size_t i = 0; i < coord_cost_list.size(); i++) {
    const Coordinate& coord = coord_cost_list[i].first;
    double cost = coord_cost_list[i].second;

    iter = coord_cost_map.find(coord);
    if (iter != coord_cost_map.end()) {
      coord_cost_map[coord] += cost;
    } else {
      coord_cost_map.insert(std::map<Coordinate, double>::value_type(coord, cost));
    }
  }

  for (int x = 0; x < _grid_map.get_x_size(); x++) {
    for (int y = 0; y < _grid_map.get_y_size(); y++) {
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

void Model::addObsToGridMap()
{
  std::vector<std::pair<Coordinate, NodeType>>& coord_type_list = _config.get_coord_type_list();
  for (size_t i = 0; i < coord_type_list.size(); i++) {
    setNode(coord_type_list[i].first, coord_type_list[i].second);
  }
}

Node* Model::setNode(const Coordinate& coord, const NodeType& node_type)
{
  assert(0 <= coord.get_x() && coord.get_x() < (int) _grid_map.get_x_size());
  assert(0 <= coord.get_y() && coord.get_y() < (int) _grid_map.get_y_size());

  Node& node = _grid_map[coord.get_x()][coord.get_y()];
  if (!node.isEndPoint() && !node.isStartPoint()) {
    node.set_type(node_type);
  }
  return &node;
}

void Model::initStartNode()
{
  _start_node->set_known_cost(_start_node->get_self_cost());
  _start_node->set_parent_node(_start_node);
  updateEstCost(_start_node);
  addNodeToSearchQueue(_start_node);
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
  if (_config.isRoutingDiagonal()) {
    wire_length = (std::abs(length - width) + (length < width ? length : width) * 1.414);
  } else {
    wire_length = (length + width);
  }
  return (wire_length + wire_length * _min_node_cost);
}

void Model::addNodeToSearchQueue(Node* node)
{
  _search_queue.push(node);
  node->set_state(NodeState::kOpen);
}

void Model::initOffsetList()
{
  if (_config.isTurningBack()) {
    _offset_list = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    if (_config.isRoutingDiagonal()) {
      _offset_list.emplace_back(1, 1);
      _offset_list.emplace_back(1, -1);
      _offset_list.emplace_back(-1, 1);
      _offset_list.emplace_back(-1, -1);
    }
  } else {
    int x_offset = _end_node->get_coord().get_x() - _start_node->get_coord().get_x();
    int y_offset = _end_node->get_coord().get_y() - _start_node->get_coord().get_y();
    x_offset = (x_offset != 0 ? x_offset / std::abs(x_offset) : x_offset);
    y_offset = (y_offset != 0 ? y_offset / std::abs(y_offset) : y_offset);
    if (x_offset != 0) {
      _offset_list.emplace_back(x_offset, 0);
    }
    if (y_offset != 0) {
      _offset_list.emplace_back(0, y_offset);
    }
    if (_config.isRoutingDiagonal() && x_offset != 0 && y_offset != 0) {
      _offset_list.emplace_back(x_offset, y_offset);
    }
  }
  std::sort(_offset_list.begin(), _offset_list.end(), cmpCoordinate());
  std::vector<astar::Coordinate>::iterator erase_iter;
  erase_iter = std::unique(_offset_list.begin(), _offset_list.end());
  _offset_list.erase(erase_iter, _offset_list.end());
}

void Model::updateOptimalNode()
{
  _optimal_node = _search_queue.top();
  _search_queue.pop();
  _optimal_node->set_state(NodeState::kClose);
}

void Model::expandSearchQueue()
{
  std::vector<Node*> neighbor_node_list = getNeighborsByOptimalNode();
  for (size_t i = 0; i < neighbor_node_list.size(); i++) {
    Node* neighbor_node = neighbor_node_list[i];
    if (neighbor_node->isOpenState()) {
      if (needReplaceParentNode(neighbor_node)) {
        updateParentByOptimalNode(neighbor_node);
      }
    } else if (neighbor_node->isNoneState()) {
      updateEstCost(neighbor_node);
      updateParentByOptimalNode(neighbor_node);
      addNodeToSearchQueue(neighbor_node);
    } else {
      std::cout << "[Error] Neighbor node is illegal!" << std::endl;
    }
  }
}

std::vector<Node*> Model::getNeighborsByOptimalNode()
{
  std::vector<Node*> neighbor_node_list;

  Coordinate& optimal_coord = _optimal_node->get_coord();
  int optimal_coord_x = optimal_coord.get_x();
  int optimal_coord_y = optimal_coord.get_y();

  for (size_t i = 0; i < _offset_list.size(); i++) {
    int x = optimal_coord_x + _offset_list[i].get_x();
    int y = optimal_coord_y + _offset_list[i].get_y();
    if (checkCoord(x, y) && checkNode(x, y)) {
      neighbor_node_list.emplace_back(&_grid_map[x][y]);
    }
  }
  return neighbor_node_list;
}

bool Model::checkCoord(const int x, const int y)
{
  if (x < 0 || (int) _grid_map.get_x_size() <= x) {
    return false;
  }
  if (y < 0 || (int) _grid_map.get_y_size() <= y) {
    return false;
  }
  Coordinate& optimal_coord = _optimal_node->get_coord();
  if (optimal_coord.get_x() == x && optimal_coord.get_y() == y) {
    return false;
  }
  return true;
}

bool Model::checkNode(const int x, const int y)
{
  Node* node = &_grid_map[x][y];
  if (node->isCloseState() || node->isOmniObs()) {
    return false;
  }
  Direction direction = getDirection(node, _optimal_node);
  if (direction == Direction::kHorizontal) {
    if (_optimal_node->isHObs() || node->isHObs()) {
      return false;
    }
  } else if (direction == Direction::kVertical) {
    if (_optimal_node->isVObs() || node->isVObs()) {
      return false;
    }
  }
  return true;
}

bool Model::needReplaceParentNode(Node* node)
{
  double cost = 0;
  cost += _optimal_node->get_known_cost();
  cost += getWalkingCost(_optimal_node, node);
  cost += node->get_self_cost();

  return cost < node->get_known_cost();
}

double Model::getWalkingCost(Node* node1, Node* node2)
{
  double walking_cost = 1;
  if (getDirection(node1, node2) == Direction::kDiagonal) {
    walking_cost = 1.414;
  }
  return walking_cost;
}

void Model::updateParentByOptimalNode(Node* node)
{
  node->set_known_cost(_optimal_node->get_known_cost() + getWalkingCost(_optimal_node, node) + node->get_self_cost());
  node->set_parent_node(_optimal_node);
}

void Model::showResult()
{
  if (_grid_map.get_x_size() > 50 || _grid_map.get_y_size() > 50) {
    std::cout << "[astar Warning] Plot large map(larger than 50x50) , skipping..." << std::endl;
    return;
  }
  setOnPath(true);
  printGridMap();
  setOnPath(false);
}

void Model::setOnPath(const bool on_path)
{
  if (_optimal_node == nullptr) {
    return;
  }
  Node* temp_node = _optimal_node;
  do {
    temp_node->set_on_path(on_path);
    temp_node = temp_node->get_parent_node();
  } while (!temp_node->isStartPoint());
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
          std::cout << "[astar Error] Node type not recognized!" << std::endl;
          exit(1);
      }
      break;
    case NodeType::kOmniObs:
      printf("\33[40m[O_OBS+O_OBS]\033[0m");
      break;
    case NodeType::kHObs:
      printf("\33[40m[H_OBS+H_OBS]\033[0m");
      break;
    case NodeType::kVObs:
      printf("\33[40m[V_OBS+V_OBS]\033[0m");
      break;
    case NodeType::kStart:
      printf("\33[42m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
      break;
    case NodeType::kEnd:
      printf("\33[44m[%05.2lf+%05.2lf]\033[0m", curr_cost, est_cost);
      break;
    default:
      std::cout << "[astar Error] Node type not recognized!" << std::endl;
      exit(1);
  }
}

void Model::reportResult()
{
  if (_optimal_node->isEndPoint()) {
    std::cout << "[astar Info] Reached the end node!! Path cost:" << _optimal_node->get_total_cost();
  } else {
    std::cout << "[astar Info] Can't reach the end node!!";
  }
  std::cout << std::endl;
}

std::vector<Coordinate> Model::getPathCoord()
{
  std::vector<Coordinate> path_coord;

  if (_optimal_node == nullptr || !_optimal_node->isEndPoint()) {
    return path_coord;
  }

  path_coord.push_back(_end_node->get_coord());
  Direction curr_direction = getDirection(_optimal_node, _optimal_node->get_parent_node());

  Node* temp_node = _optimal_node;
  do {
    Direction direction = getDirection(temp_node, temp_node->get_parent_node());
    if (curr_direction != direction) {
      curr_direction = direction;
      path_coord.push_back(temp_node->get_coord());
    }
    temp_node = temp_node->get_parent_node();

  } while (!temp_node->isStartPoint());

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
