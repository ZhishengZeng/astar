/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-07 21:51:46
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
  std::map<Coordinate, std::set<ObsType>, cmpCoordinate>& coord_obs_map = _config.get_coord_obs_map();

  if (coord_obs_map.find(coord) != coord_obs_map.end()) {
    coord_obs_map[coord].insert(type);
  } else {
    coord_obs_map.insert(std::map<Coordinate, std::set<ObsType>>::value_type(coord, {type}));
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
    printResult();
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
    printResult();
  }

  return getFinalInflectionPath();
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
      _grid_map[i][j].set_self_cost(COST_UNIT);
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
  std::map<Coordinate, double, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();

  double min_cost = __DBL_MAX__;
  for (auto [coord, cost] : coord_cost_map) {
    min_cost = std::min(min_cost, cost);
  }

  std::map<Coordinate, double>::iterator iter;
  for (iter = coord_cost_map.begin(); iter != coord_cost_map.end(); iter++) {
    iter->second = (iter->second - min_cost + COST_UNIT);
  }

  for (auto [coord, cost] : coord_cost_map) {
    _grid_map[coord.get_x()][coord.get_y()].set_self_cost(cost);
  }
}

void Model::addObsToGridMap()
{
  std::map<Coordinate, std::set<ObsType>, cmpCoordinate> coord_obs_map = _config.get_coord_obs_map();
  for (auto [coord, obs_set] : coord_obs_map) {
    _grid_map[coord.get_x()][coord.get_y()].set_obs_set(obs_set);
  }
}

void Model::initStartNode()
{
  _start_node->set_cert_sum(0);
  updateEstCost(_start_node);
  updateOpen(_start_node);
}

void Model::updateEstCost(Node* node)
{
  node->set_est_sum(caculateEstCost(node, _end_node));
}

void Model::updateOpen(Node* node)
{
  node->setOpen();
  _open_queue.push(node);
}

double Model::caculateEstCost(Node* a, Node* b)
{
  double est_length = std::abs(a->get_coord().get_x() - b->get_coord().get_x())
                      + std::abs(a->get_coord().get_y() - b->get_coord().get_y());
  double est_cost = est_length * COST_UNIT;
  double est_corner = CORNER_UNIT;

  return (est_length + est_cost + est_corner);
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
    if (neighbor_node->isClose() || (!touchByHead(neighbor_node))) {
      continue;
    }
    if (neighbor_node->isOpen()) {
      if (needReplaceParentNode(neighbor_node)) {
        updateParentByPathHead(neighbor_node);
      }
    } else {
      updateEstCost(neighbor_node);
      updateParentByPathHead(neighbor_node);
      updateOpen(neighbor_node);
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

bool Model::touchByHead(Node* node)
{
  Coordinate& head_coord = _path_head_node->get_coord();
  Coordinate& to_coord = node->get_coord();

  int head_x = head_coord.get_x();
  int head_y = head_coord.get_y();
  int to_x = to_coord.get_x();
  int to_y = to_coord.get_y();

  if (head_x < to_x) {
    // left to right
    if (Util::exist(_path_head_node->get_obs_set(), ObsType::kHRightObs)
        || Util::exist(node->get_obs_set(), ObsType::kHLeftObs)) {
      return false;
    }
  } else if (head_x > to_x) {
    if (Util::exist(_path_head_node->get_obs_set(), ObsType::kHLeftObs)
        || Util::exist(node->get_obs_set(), ObsType::kHRightObs)) {
      return false;
    }
  }

  if (head_y < to_y) {
    // down to up
    if (Util::exist(_path_head_node->get_obs_set(), ObsType::kVTopObs)
        || Util::exist(node->get_obs_set(), ObsType::kVBottomObs)) {
      return false;
    }
  } else if (head_y > to_y) {
    // up to down
    if (Util::exist(_path_head_node->get_obs_set(), ObsType::kVBottomObs)
        || Util::exist(node->get_obs_set(), ObsType::kVTopObs)) {
      return false;
    }
  }

  return true;
}

bool Model::needReplaceParentNode(Node* node)
{
  return getCertSumByHead(node) < node->get_cert_sum();
}

double Model::getCertSumByHead(Node* node)
{
  double cost = 0;
  cost += _path_head_node->get_cert_sum();
  cost += _path_head_node->get_self_cost();
  cost += LENGTH_UNIT;

  if (_path_head_node->get_parent_node()) {
    if (getDirection(_path_head_node->get_parent_node(), _path_head_node) != getDirection(_path_head_node, node)) {
      cost += CORNER_UNIT;
    }
  }

  return cost;
}

void Model::updateParentByPathHead(Node* node)
{
  node->set_cert_sum(getCertSumByHead(node));
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

void Model::printResult()
{
  int factor = 10;

  int none_layer = 0;
  int open_layer = 1;
  int close_layer = 2;
  int obs_layer = 3;
  int start_layer = 4;
  int end_layer = 5;
  int path_layer = 6;

  std::ostringstream oss;
  oss << "a_star.gds";
  std::string gds_file_path = oss.str();
  oss.clear();

  std::ofstream gds_file(gds_file_path);
  if (gds_file.is_open()) {
    gds_file << "HEADER 600" << std::endl;
    gds_file << "BGNLIB" << std::endl;
    gds_file << "LIBNAME DensityLib" << std::endl;
    gds_file << "UNITS 0.001 1e-9" << std::endl;

    gds_file << "BGNSTR" << std::endl;
    gds_file << "STRNAME a_star" << std::endl;
    // open close none
    for (int x = 0; x < _grid_map.get_x_size(); x++) {
      for (int y = 0; y < _grid_map.get_y_size(); y++) {
        Node& node = _grid_map[x][y];

        gds_file << "BOUNDARY" << std::endl;
        if (node.isOpen()) {
          gds_file << "LAYER " << open_layer << std::endl;
        } else if (node.isClose()) {
          gds_file << "LAYER " << close_layer << std::endl;
        } else {
          gds_file << "LAYER " << none_layer << std::endl;
        }
        gds_file << "DATATYPE 0" << std::endl;
        gds_file << "XY" << std::endl;
        gds_file << x * factor << " : " << y * factor << std::endl;
        gds_file << (x + 1) * factor << " : " << y * factor << std::endl;
        gds_file << (x + 1) * factor << " : " << (y + 1) * factor << std::endl;
        gds_file << x * factor << " : " << (y + 1) * factor << std::endl;
        gds_file << x * factor << " : " << y * factor << std::endl;
        gds_file << "ENDEL" << std::endl;
        // obs
        std::set<ObsType>& obs_set = node.get_obs_set();
        for (const ObsType& obs_type : obs_set) {
          if (obs_type == ObsType::kNone) {
            continue;
          }
          gds_file << "PATH" << std::endl;
          gds_file << "LAYER " << obs_layer << std::endl;
          gds_file << "DATATYPE 0" << std::endl;
          gds_file << "WIDTH " << 1 << std::endl;
          gds_file << "XY" << std::endl;
          switch (obs_type) {
            case ObsType::kHLeftObs:
              gds_file << x * factor + 1 << " : " << y * factor + 1 << std::endl;
              gds_file << x * factor + 1 << " : " << (y + 1) * factor - 1 << std::endl;
              break;
            case ObsType::kHRightObs:
              gds_file << (x + 1) * factor - 1 << " : " << y * factor + 1 << std::endl;
              gds_file << (x + 1) * factor - 1 << " : " << (y + 1) * factor - 1 << std::endl;
              break;
            case ObsType::kVTopObs:
              gds_file << x * factor + 1 << " : " << (y + 1) * factor - 1 << std::endl;
              gds_file << (x + 1) * factor - 1 << " : " << (y + 1) * factor - 1 << std::endl;
              break;
            case ObsType::kVBottomObs:
              gds_file << x * factor + 1 << " : " << y * factor + 1 << std::endl;
              gds_file << (x + 1) * factor - 1 << " : " << y * factor + 1 << std::endl;
              break;
            default:
              std::cout << "[AStar Error] Invaild OBS type!" << std::endl;
              exit(1);
          }
          gds_file << "ENDEL" << std::endl;
        }
      }
    }
    // start end path
    std::vector<Coordinate> coord_list = getCoordPath();

    for (size_t i = 0; i < coord_list.size(); i++) {
      Coordinate& coord = coord_list[i];
      gds_file << "BOUNDARY" << std::endl;
      if (i == 0) {
        gds_file << "LAYER " << start_layer << std::endl;
      } else if (i == coord_list.size() - 1) {
        gds_file << "LAYER " << end_layer << std::endl;
      } else {
        gds_file << "LAYER " << path_layer << std::endl;
      }
      gds_file << "DATATYPE 0" << std::endl;
      gds_file << "XY" << std::endl;
      gds_file << coord.get_x() * factor << " : " << coord.get_y() * factor << std::endl;
      gds_file << (coord.get_x() + 1) * factor << " : " << coord.get_y() * factor << std::endl;
      gds_file << (coord.get_x() + 1) * factor << " : " << (coord.get_y() + 1) * factor << std::endl;
      gds_file << coord.get_x() * factor << " : " << (coord.get_y() + 1) * factor << std::endl;
      gds_file << coord.get_x() * factor << " : " << coord.get_y() * factor << std::endl;
      gds_file << "ENDEL" << std::endl;
    }
    gds_file << "ENDSTR" << std::endl;
    gds_file << "ENDLIB" << std::endl;
    gds_file.close();
    std::cout << "[AStar Info] Routing result has been written to '" << gds_file_path << "'!" << std::endl;
  } else {
    std::cout << "[AStar Error] Failed to open gds file '" << gds_file_path << "'!" << std::endl;
    exit(1);
  }
}

std::vector<Coordinate> Model::getCoordPath()
{
  std::vector<Coordinate> coord_path;

  coord_path.push_back(_end_node->get_coord());

  Node* temp_node = _path_head_node;

  while (temp_node != _start_node) {
    coord_path.push_back(temp_node->get_coord());
    temp_node = temp_node->get_parent_node();
  }
  coord_path.push_back(_start_node->get_coord());
  coord_path.push_back(_start_node->get_coord());

  for (size_t i = 0, j = (coord_path.size() - 1); i < j; i++, j--) {
    std::swap(coord_path[i], coord_path[j]);
  }

  return coord_path;
}

std::vector<Coordinate> Model::getFinalInflectionPath()
{
  std::vector<Coordinate> inflection_path;

  if (_path_head_node == nullptr || _path_head_node != _end_node) {
    return inflection_path;
  }

  inflection_path.push_back(_end_node->get_coord());
  Direction curr_direction = getDirection(_path_head_node, _path_head_node->get_parent_node());

  Node* temp_node = _path_head_node;
  while (temp_node != _start_node) {
    Direction direction = getDirection(temp_node, temp_node->get_parent_node());
    if (curr_direction != direction) {
      curr_direction = direction;
      inflection_path.push_back(temp_node->get_coord());
    }
    temp_node = temp_node->get_parent_node();
  }
  inflection_path.push_back(_start_node->get_coord());

  for (size_t i = 0, j = (inflection_path.size() - 1); i < j; i++, j--) {
    std::swap(inflection_path[i], inflection_path[j]);
  }

  return inflection_path;
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
    exit(1);
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
