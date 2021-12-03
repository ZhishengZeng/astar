/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-03 21:17:08
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
  std::map<Coordinate, double, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();
  coord_cost_map.insert(std::map<Coordinate, double>::value_type(coord, cost));
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
void Model::addObstacle(const Coordinate& coord, ObsType type)
{
  std::map<Coordinate, std::vector<ObsType>, cmpCoordinate>& coord_obs_map = _config.get_coord_obs_map();

  if (coord_obs_map.find(coord) != coord_obs_map.end()) {
    coord_obs_map[coord].push_back(type);
  } else {
    coord_obs_map.insert(std::map<Coordinate, std::vector<ObsType>>::value_type(coord, {type}));
  }
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
    // 更新路径头节点
    updatePathHead();
    // 路径头节点抵达终点
    if (_path_head_node == _end_node) {
      break;
    }
    // 得到所有下一步节点
    expandSearching();
    // 能走的路为空
    if (_open_queue.empty()) {
      break;
    }
  }

  int log_verbose = _config.get_log_verbose();
  if (log_verbose > 0) {
    reportResult();
  }
  if (log_verbose > 1) {
    // showResult();
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
  _start_node = &_grid_map[coord.get_x()][coord.get_y()];
}

void Model::addEndNodeToGridMap(const Coordinate& coord)
{
  _end_node = &_grid_map[coord.get_x()][coord.get_y()];
}

void Model::addCostToGridMap()
{
  legalizeCost();
  std::map<Coordinate, double, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();

  for (auto [coord, cost] : coord_cost_map) {
    _grid_map[coord.get_x()][coord.get_y()].set_self_cost(cost);
  }
}

void Model::legalizeCost()
{
  std::map<Coordinate, double, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();

  double min_cost = __DBL_MAX__;
  for (auto [coord, cost] : coord_cost_map) {
    min_cost = std::min(min_cost, cost);
  }

  std::map<Coordinate, double>::iterator iter;
  for (iter = coord_cost_map.begin(); iter != coord_cost_map.end(); iter++) {
    iter->second -= min_cost;
  }
}

void Model::addObsToGridMap()
{
  std::map<Coordinate, std::vector<ObsType>, cmpCoordinate> coord_obs_map = _config.get_coord_obs_map();
  for (auto [coord, obs_list] : coord_obs_map) {
    _grid_map[coord.get_x()][coord.get_y()].set_obs_list(obs_list);
  }
}

void Model::initStartNode()
{
  _start_node->set_cert_cost(0);
  updateEstCost(_start_node);
  _open_queue.push(_start_node);
}

void Model::updateEstCost(Node* node)
{
  node->set_est_cost(caculateEstCost(node, _end_node));
}

double Model::caculateEstCost(Node* a, Node* b)
{
  int total_length = std::abs(a->get_coord().get_x() - b->get_coord().get_x())
                     + std::abs(a->get_coord().get_y() - b->get_coord().get_y());
  return total_length + total_length * _min_node_cost;
}

void Model::initOffsetList()
{
  if (_config.isTurningBack()) {
    _offset_list = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  } else {
    int x_offset = _end_node->get_coord().get_x() - _start_node->get_coord().get_x();
    int y_offset = _end_node->get_coord().get_y() - _start_node->get_coord().get_y();

    if (x_offset > 0) {
      _offset_list.emplace_back(1, 0);
    } else if (x_offset < 0) {
      _offset_list.emplace_back(-1, 0);
    }

    if (y_offset > 0) {
      _offset_list.emplace_back(0, 1);
    } else if (y_offset < 0) {
      _offset_list.emplace_back(0, -1);
    }
  }
}

void Model::updatePathHead()
{
  _path_head_node = _open_queue.top();
  _open_queue.pop();
  _path_head_node->setClose();
}

void Model::expandSearching()
{
  std::vector<Node*> neighbor_node_list = getNeighborsByPathHead();
  for (size_t i = 0; i < neighbor_node_list.size(); i++) {
    Node* neighbor_node = neighbor_node_list[i];
    if (neighbor_node->isClose() || isInvaild(neighbor_node)) {
      continue;
    } else if (neighbor_node->isOpen()) {
      if (needReplaceParentNode(neighbor_node)) {
        updateParentByPathHead(neighbor_node);
      }
    } else {
      updateEstCost(neighbor_node);
      updateParentByPathHead(neighbor_node);
      _open_queue.push(neighbor_node);
    }
  }
}

std::vector<Node*> Model::getNeighborsByPathHead()
{
  std::vector<Node*> neighbor_node_list;

  Coordinate& optimal_coord = _path_head_node->get_coord();
  int optimal_coord_x = optimal_coord.get_x();
  int optimal_coord_y = optimal_coord.get_y();

  for (size_t i = 0; i < _offset_list.size(); i++) {
    int x = optimal_coord_x + _offset_list[i].get_x();
    int y = optimal_coord_y + _offset_list[i].get_y();

    if (x < 0 || (int) _grid_map.get_x_size() <= x) {
      continue;
    }
    if (y < 0 || (int) _grid_map.get_y_size() <= y) {
      continue;
    }
    neighbor_node_list.emplace_back(&_grid_map[x][y]);
  }
  return neighbor_node_list;
}

bool Model::isInvaild(Node* node)
{
  if (node->isClose()) {
    return false;
  }
  // _path_head_node->get_parent_node();
  // Direction direction = getDirection(node, _path_head_node);
  // if (direction == Direction::kH) {
  //   if (_path_head_node->isHObs() || node->isHObs()) {
  //     return false;
  //   }
  // } else if (direction == Direction::kV) {
  //   if (_path_head_node->isVObs() || node->isVObs()) {
  //     return false;
  //   }
  // }
  return true;
}

bool Model::needReplaceParentNode(Node* node)
{
  double cost = 0;
  cost += _path_head_node->get_cert_cost();
  cost += _path_head_node->get_self_cost();
  cost += 1;
  return cost < node->get_cert_cost();
}

void Model::updateParentByPathHead(Node* node)
{
  node->set_cert_cost(_path_head_node->get_cert_cost() + _path_head_node->get_self_cost() + 1);
  node->set_parent_node(_path_head_node);
}

void Model::reportResult()
{
  if (_path_head_node == _end_node) {
    std::cout << "[astar Info] Reached the end node!! Path cost:" << _end_node->getTotalCost();
  } else {
    std::cout << "[astar Info] Can't reach the end node!!";
  }
  std::cout << std::endl;
}

std::vector<Coordinate> Model::getPathCoord()
{
  std::vector<Coordinate> path_coord;

  if (_path_head_node == nullptr || _path_head_node != _end_node) {
    return path_coord;
  }

  path_coord.push_back(_end_node->get_coord());
  Direction curr_direction = getDirection(_path_head_node, _path_head_node->get_parent_node());

  Node* temp_node = _path_head_node;
  do {
    Direction direction = getDirection(temp_node, temp_node->get_parent_node());
    if (curr_direction != direction) {
      curr_direction = direction;
      path_coord.push_back(temp_node->get_coord());
    }
    temp_node = temp_node->get_parent_node();

  } while (temp_node != _start_node);

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
    return Direction::kH;
  } else if (isVertical(start_coord, end_coord)) {
    return Direction::kV;
  } else {
    std::cout << "[astar Error] Invaild Direction!" << std::endl;
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
