/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-12-09 14:11:52
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
    plotResult();
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

double Model::caculateEstCost(Node* start_node, Node* end_node)
{
  double est_length = std::abs(start_node->get_coord().get_x() - end_node->get_coord().get_x())
                      + std::abs(start_node->get_coord().get_y() - end_node->get_coord().get_y());
  double est_cost = est_length * COST_UNIT;

  double est_corner = getEstCorner(start_node, end_node);

  return (est_length + est_cost + est_corner);
}

double Model::getEstCorner(Node* start_node, Node* end_node)
{
  if (equalCoord(start_node, end_node)) {
    return 0;
  }

  std::vector<std::vector<Node*>> coord_path_list = tryRouting(start_node, end_node);

  int est_corner_num = 0;
  if (coord_path_list.size() > 0) {
    est_corner_num = getMinCornerNum(coord_path_list);
  } else {
    est_corner_num = 2;
  }
  return (est_corner_num * CORNER_UNIT);
}

bool Model::equalCoord(Node* start_node, Node* end_node)
{
  return start_node->get_coord() == end_node->get_coord();
}

std::vector<std::vector<Node*>> Model::tryRouting(Node* start_node, Node* end_node)
{
  std::vector<std::vector<Node*>> coord_path_list;

  if (isStraight(start_node, end_node)) {
    coord_path_list = routingStraight(start_node, end_node);
  } else {
    coord_path_list = routingLShape(start_node, end_node);
  }
  return coord_path_list;
}

std::vector<std::vector<Node*>> Model::routingStraight(Node* start_node, Node* end_node)
{
  std::vector<std::vector<Node*>> coord_path_list;

  if (passCheckingSegment(start_node, end_node)) {
    std::vector<Node*> coord_path;

    if (start_node->get_parent_node()) {
      coord_path.push_back(start_node->get_parent_node());
    }
    coord_path.push_back(start_node);
    coord_path.push_back(end_node);
    coord_path_list.push_back(coord_path);
  }
  return coord_path_list;
}

bool Model::passCheckingSegment(Node* start_node, Node* end_node)
{
  Direction direction = getDirection(start_node, end_node);

  Coordinate& start_coord = start_node->get_coord();
  Coordinate& end_coord = end_node->get_coord();

  if (direction == Direction::kH) {
    int start_x = start_coord.get_x();
    int end_x = end_coord.get_x();
    int y = start_coord.get_y();

    int offset = 0;
    if (start_x < end_x) {
      offset = 1;
    } else if (start_x > end_x) {
      offset = -1;
    } else {
      std::cout << "[AStar Error] The abs of offset is not 1!" << std::endl;
      exit(1);
    }

    for (int x = start_x; x != end_x; x += offset) {
      Node* pre_node = &_grid_map[x][y];
      Node* curr_node = &_grid_map[x + offset][y];

      if (offset == 1) {
        // left to right
        if (Util::exist(pre_node->get_obs_set(), ObsType::kEastObs)
            || Util::exist(curr_node->get_obs_set(), ObsType::kWestObs)) {
          return false;
        }
      } else if (offset == -1) {
        // right to left
        if (Util::exist(pre_node->get_obs_set(), ObsType::kWestObs)
            || Util::exist(curr_node->get_obs_set(), ObsType::kEastObs)) {
          return false;
        }
      }
    }
  } else if (direction == Direction::kV) {
    int start_y = start_coord.get_y();
    int end_y = end_coord.get_y();
    int x = start_coord.get_x();

    int offset = 0;
    if (start_y < end_y) {
      offset = 1;
    } else if (start_y > end_y) {
      offset = -1;
    } else {
      std::cout << "[AStar Error] The abs of offset is not 1!" << std::endl;
      exit(1);
    }

    for (int y = start_y; y != end_y; y += offset) {
      Node* pre_node = &_grid_map[x][y];
      Node* curr_node = &_grid_map[x][y + offset];

      if (offset == 1) {
        // down to up
        if (Util::exist(pre_node->get_obs_set(), ObsType::kNorthObs)
            || Util::exist(curr_node->get_obs_set(), ObsType::kSouthObs)) {
          return false;
        }
      } else if (offset == -1) {
        // up to down
        if (Util::exist(pre_node->get_obs_set(), ObsType::kSouthObs)
            || Util::exist(curr_node->get_obs_set(), ObsType::kNorthObs)) {
          return false;
        }
      }
    }
  } else {
    std::cout << "[AStar Error] Segment is not straight!" << std::endl;
    exit(1);
  }
  return true;
}

std::vector<std::vector<Node*>> Model::routingLShape(Node* start_node, Node* end_node)
{
  Coordinate& start_coord = start_node->get_coord();
  Coordinate& end_coord = end_node->get_coord();

  std::vector<std::vector<Node*>> coord_path_list;

  Node* Inflection_node1 = &_grid_map[start_coord.get_x()][end_coord.get_y()];
  if (passCheckingSegment(start_node, Inflection_node1) && passCheckingSegment(Inflection_node1, end_node)) {
    std::vector<Node*> coord_path;

    if (start_node->get_parent_node()) {
      coord_path.push_back(start_node->get_parent_node());
    }
    coord_path.push_back(start_node);
    coord_path.push_back(Inflection_node1);
    coord_path.push_back(end_node);
    coord_path_list.push_back(coord_path);
  }

  Node* Inflection_node2 = &_grid_map[end_coord.get_x()][start_coord.get_y()];
  if (passCheckingSegment(start_node, Inflection_node2) && passCheckingSegment(Inflection_node2, end_node)) {
    std::vector<Node*> coord_path;

    if (start_node->get_parent_node()) {
      coord_path.push_back(start_node->get_parent_node());
    }
    coord_path.push_back(start_node);
    coord_path.push_back(Inflection_node2);
    coord_path.push_back(end_node);
    coord_path_list.push_back(coord_path);
  }

  return coord_path_list;
}

int Model::getMinCornerNum(std::vector<std::vector<Node*>>& coord_path_list)
{
  int min_corner_num = __INT_MAX__;

  if (coord_path_list.size() == 0) {
    min_corner_num = 0;
  }

  for (size_t i = 0; i < coord_path_list.size(); i++) {
    int corner_num = 0;
    std::vector<Node*>& coord_path = coord_path_list[i];
    for (size_t j = 2; j < coord_path.size(); j++) {
      if (getDirection(coord_path[j - 2], coord_path[j - 1]) != getDirection(coord_path[j - 1], coord_path[j])) {
        corner_num++;
      }
    }
    min_corner_num = std::min(min_corner_num, corner_num);
  }

  if (min_corner_num == __INT_MAX__) {
    std::cout << "[AStar Error] Corner num cannot update!" << std::endl;
    exit(1);
  }

  return min_corner_num;
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
    if (neighbor_node->isClose() || (!passCheckingSegment(_path_head_node, neighbor_node))) {
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

void Model::plotResult()
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
            case ObsType::kWestObs:
              gds_file << x * factor + 1 << " : " << y * factor + 1 << std::endl;
              gds_file << x * factor + 1 << " : " << (y + 1) * factor - 1 << std::endl;
              break;
            case ObsType::kEastObs:
              gds_file << (x + 1) * factor - 1 << " : " << y * factor + 1 << std::endl;
              gds_file << (x + 1) * factor - 1 << " : " << (y + 1) * factor - 1 << std::endl;
              break;
            case ObsType::kNorthObs:
              gds_file << x * factor + 1 << " : " << (y + 1) * factor - 1 << std::endl;
              gds_file << (x + 1) * factor - 1 << " : " << (y + 1) * factor - 1 << std::endl;
              break;
            case ObsType::kSouthObs:
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
    return Direction::kOblique;
  }
}

bool Model::isStraight(Node* start_node, Node* end_node)
{
  Coordinate& start_coord = start_node->get_coord();
  Coordinate& end_coord = end_node->get_coord();
  return isHorizontal(start_coord, end_coord) || isVertical(start_coord, end_coord);
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
