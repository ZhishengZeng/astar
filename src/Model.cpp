/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-11 11:49:07
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2022-04-28 11:54:55
 * @FilePath: /astar/src/Model.cpp
 */
#include "AS_Model.hpp"

#include "AS_Util.hpp"

namespace astar {

/**
 * @description: 建立一个x_size*y_size的grid_map
 * @param {int} x_size
 * @param {int} y_size
 * @return {*}
 */
void Model::buildMap(const int x_size, const int y_size)
{
  _config.set_map_x_size(x_size);
  _config.set_map_y_size(y_size);
}

/**
 * @description: Add obstacle node
 *
 * @param {Coordinate} coord
 * @param {Orientation} orientation
 * @return {*}
 */
void Model::addOBS(const Coordinate& coord, const Orientation& orientation)
{
  std::map<Coordinate, std::map<Orientation, bool>, cmpCoordinate>& coord_obs_map = _config.get_coord_obs_map();
  coord_obs_map[coord][orientation] = true;
}

/**
 * @description: Add node cost. Default is 0.01
 * @param {Coordinate} coord
 * @param {Orientation} orientation
 * @param {double} cost
 * @return {*}
 */
void Model::addCost(const Coordinate& coord, const Orientation& orientation, const double cost)
{
  std::map<Coordinate, std::map<Orientation, double>, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();
  coord_cost_map[coord][orientation] = cost;
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
void Model::setLogVerbose(const int level)
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
    _path_head = _open_queue.top();
    _open_queue.pop();
    _path_head->set_search_state(SearchState::kClose);
    // 路径头节点抵达终点
    if (_path_head == _end) {
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

// private

void Model::init(const Coordinate& start_coord, const Coordinate& end_coord)
{
  initGridMap();
  addStartNodeToGridMap(start_coord);
  addEndNodeToGridMap(end_coord);
  addObsToGridMap();
  legalizeCostMap();
  addCostToGridMap();
  initStartNode();
  initOffsetList();
}

void Model::initGridMap()
{
  std::map<Orientation, bool> obs_map = {{Orientation::kEast, false},
                                         {Orientation::kSouth, false},
                                         {Orientation::kWest, false},
                                         {Orientation::kNorth, false}};

  std::map<Orientation, double> cost_map = {{Orientation::kEast, COST_UNIT},
                                            {Orientation::kSouth, COST_UNIT},
                                            {Orientation::kWest, COST_UNIT},
                                            {Orientation::kNorth, COST_UNIT}};

  _grid_map.init(_config.get_map_x_size(), _config.get_map_y_size());
  for (int i = 0; i < (int) _grid_map.get_x_size(); i++) {
    for (int j = 0; j < (int) _grid_map.get_y_size(); j++) {
      _grid_map[i][j].set_coord(i, j);
      _grid_map[i][j].set_obs_map(obs_map);
      _grid_map[i][j].set_cost_map(cost_map);
    }
  }
}

void Model::addStartNodeToGridMap(const Coordinate& coord)
{
  _start = &_grid_map[coord.get_x()][coord.get_y()];
}

void Model::addEndNodeToGridMap(const Coordinate& coord)
{
  _end = &_grid_map[coord.get_x()][coord.get_y()];
}

void Model::addObsToGridMap()
{
  std::map<Coordinate, std::map<Orientation, bool>, cmpCoordinate>& coord_obs_map = _config.get_coord_obs_map();

  for (auto [coord, obs_map] : coord_obs_map) {
    std::map<Orientation, bool>& node_obs_map = _grid_map[coord.get_x()][coord.get_y()].get_obs_map();

    std::map<Orientation, bool>::iterator iter;
    for (iter = obs_map.begin(); iter != obs_map.end(); iter++) {
      node_obs_map[iter->first] = iter->second;
    }
  }
}

void Model::legalizeCostMap()
{
  std::map<Coordinate, std::map<Orientation, double>, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();

  double min_cost = __DBL_MAX__;
  for (auto [coord, cost_map] : coord_cost_map) {
    std::map<Orientation, double>::iterator iter;
    for (iter = cost_map.begin(); iter != cost_map.end(); iter++) {
      min_cost = std::min(min_cost, iter->second);
    }
  }

  if (min_cost < COST_UNIT) {
    for (auto [coord, cost_map] : coord_cost_map) {
      std::map<Orientation, double>::iterator iter;
      for (iter = cost_map.begin(); iter != cost_map.end(); iter++) {
        iter->second = (iter->second - min_cost + COST_UNIT);
      }
    }
  }
  _min_cost = std::max(min_cost, COST_UNIT);
}

void Model::addCostToGridMap()
{
  std::map<Coordinate, std::map<Orientation, double>, cmpCoordinate>& coord_cost_map = _config.get_coord_cost_map();
  for (auto& [coord, cost_map] : coord_cost_map) {
    std::map<Orientation, double>& node_cost_map = _grid_map[coord.get_x()][coord.get_y()].get_cost_map();

    std::map<Orientation, double>::iterator iter;
    for (iter = cost_map.begin(); iter != cost_map.end(); iter++) {
      node_cost_map[iter->first] = iter->second;
    }
  }
}

void Model::initStartNode()
{
  _start->set_known_cost(0);
  _start->set_estimated_cost(getEstimateCost(_start, _end));
  _start->set_search_state(SearchState::kOpen);
  _open_queue.push(_start);
}

void Model::initOffsetList()
{
  if (_config.isTurningBack()) {
    _offset_list = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  } else {
    int x_offset = _end->get_coord().get_x() - _start->get_coord().get_x();
    int y_offset = _end->get_coord().get_y() - _start->get_coord().get_y();

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

void Model::expandSearching()
{
  std::vector<Node*> neighbor_node_list = getNeighborsByPathHead();
  for (size_t i = 0; i < neighbor_node_list.size(); i++) {
    Node* neighbor_node = neighbor_node_list[i];
    if (neighbor_node->isClose() || (!passCheckingSegment(_path_head, neighbor_node))) {
      continue;
    }
    if (neighbor_node->isOpen()) {
      if (replaceParentNode(_path_head, neighbor_node)) {
        neighbor_node->set_known_cost(getKnowCost(_path_head, neighbor_node));
        neighbor_node->set_parent_node(_path_head);
      }
    } else {
      neighbor_node->set_known_cost(getKnowCost(_path_head, neighbor_node));
      neighbor_node->set_parent_node(_path_head);
      neighbor_node->set_estimated_cost(getEstimateCost(neighbor_node, _end));
      neighbor_node->set_search_state(SearchState::kOpen);
      _open_queue.push(neighbor_node);
    }
  }
}

std::vector<Node*> Model::getNeighborsByPathHead()
{
  std::vector<Node*> neighbor_node_list;

  Coordinate& optimal_coord = _path_head->get_coord();
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

bool Model::replaceParentNode(Node* parent, Node* child)
{
  return getKnowCost(parent, child) < child->get_known_cost();
}

void Model::reportResult()
{
  if (_path_head == _end) {
    std::cout << "[astar Info] Reached the end node!! Path cost:" << _end->getTotalCost();
  } else {
    std::cout << "[astar Info] Can't reach the end node!!";
  }
  std::cout << std::endl;
}

std::vector<Coordinate> Model::getFinalInflectionPath()
{
  std::vector<Coordinate> inflection_path;

  if (_path_head == nullptr) {
    return inflection_path;
  }

  if (_path_head != _end) {
    return inflection_path;
  }

  inflection_path.push_back(_end->get_coord());

  if (_path_head->get_parent_node() != nullptr) {
    Direction curr_direction = getDirection(_path_head, _path_head->get_parent_node());
    Node* temp_node = _path_head;
    while (temp_node != _start) {
      Direction direction = getDirection(temp_node, temp_node->get_parent_node());
      if (curr_direction != direction) {
        curr_direction = direction;
        inflection_path.push_back(temp_node->get_coord());
      }
      temp_node = temp_node->get_parent_node();
    }
  }

  inflection_path.push_back(_start->get_coord());

  for (size_t i = 0, j = (inflection_path.size() - 1); i < j; i++, j--) {
    std::swap(inflection_path[i], inflection_path[j]);
  }

  if (inflection_path.size() >= 2 || inflection_path.empty()) {
    return inflection_path;
  } else {
    std::cout << "[astar Info] Inflection path error!!" << std::endl;
    assert(false);
  }

  return inflection_path;
}

// Plot

std::string convertToString(double value)
{
  std::stringstream oss;
  oss << std::setiosflags(std::ios::fixed) << std::setprecision(3) << value;
  std::string string = oss.str();
  oss.clear();
  return string;
}

void Model::plotResult()
{
  int factor = 50;
  int obs_indent = factor * 0.1;
  int text_indent = factor * 0.5;

  int none_layer = 0;
  int open_layer = 1;
  int close_layer = 2;
  int obs_and_cost_layer = 3;
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

    for (int x = 0; x < _grid_map.get_x_size(); x++) {
      for (int y = 0; y < _grid_map.get_y_size(); y++) {
        Node& node = _grid_map[x][y];

        int real_lb_x = x * factor;
        int real_lb_y = y * factor;
        int real_rt_x = (x + 1) * factor;
        int real_rt_y = (y + 1) * factor;

        // open close none
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
        gds_file << real_lb_x << " : " << real_lb_y << std::endl;
        gds_file << real_rt_x << " : " << real_lb_y << std::endl;
        gds_file << real_rt_x << " : " << real_rt_y << std::endl;
        gds_file << real_lb_x << " : " << real_rt_y << std::endl;
        gds_file << real_lb_x << " : " << real_lb_y << std::endl;
        gds_file << "ENDEL" << std::endl;
        // obs
        std::map<Orientation, bool>& obs_map = node.get_obs_map();
        for (auto& [orientation, is_obs] : obs_map) {
          if (!is_obs) {
            continue;
          }
          gds_file << "PATH" << std::endl;
          gds_file << "LAYER " << obs_and_cost_layer << std::endl;
          gds_file << "DATATYPE 0" << std::endl;
          gds_file << "WIDTH " << 1 << std::endl;
          gds_file << "XY" << std::endl;
          switch (orientation) {
            case Orientation::kWest:
              gds_file << real_lb_x + obs_indent << " : " << real_lb_y + obs_indent << std::endl;
              gds_file << real_lb_x + obs_indent << " : " << real_rt_y - obs_indent << std::endl;
              break;
            case Orientation::kEast:
              gds_file << real_rt_x - obs_indent << " : " << real_lb_y + obs_indent << std::endl;
              gds_file << real_rt_x - obs_indent << " : " << real_rt_y - obs_indent << std::endl;
              break;
            case Orientation::kNorth:
              gds_file << real_lb_x + obs_indent << " : " << real_rt_y - obs_indent << std::endl;
              gds_file << real_rt_x - obs_indent << " : " << real_rt_y - obs_indent << std::endl;
              break;
            case Orientation::kSouth:
              gds_file << real_lb_x + obs_indent << " : " << real_lb_y + obs_indent << std::endl;
              gds_file << real_rt_x - obs_indent << " : " << real_lb_y + obs_indent << std::endl;
              break;
            default:
              std::cout << "[AStar Error] Invaild Direction 2d!" << std::endl;
              exit(1);
          }
          gds_file << "ENDEL" << std::endl;
        }
        // cost
        std::map<Orientation, double>& cost_map = node.get_cost_map();
        for (auto& [orientation, cost] : cost_map) {
          if (cost == COST_UNIT) {
            continue;
          }
          gds_file << "TEXT" << std::endl;
          gds_file << "LAYER " << obs_and_cost_layer << std::endl;
          gds_file << "TEXTTYPE 0" << std::endl;
          switch (orientation) {
            case Orientation::kWest:
              // 0000000000000100
              gds_file << "PRESENTATION 4" << std::endl;
              gds_file << "XY" << std::endl;
              gds_file << real_lb_x + obs_indent << " : " << real_lb_y + text_indent << std::endl;
              break;
            case Orientation::kEast:
              // 0000000000000110
              gds_file << "PRESENTATION 6" << std::endl;
              gds_file << "XY" << std::endl;
              gds_file << real_rt_x - obs_indent << " : " << real_lb_y + text_indent << std::endl;
              break;
            case Orientation::kNorth:
              // 0000000000000001
              gds_file << "PRESENTATION 1" << std::endl;
              gds_file << "XY" << std::endl;
              gds_file << real_lb_x + text_indent << " : " << real_rt_y - obs_indent << std::endl;
              break;
            case Orientation::kSouth:
              // 0000000000001001
              gds_file << "PRESENTATION 9" << std::endl;
              gds_file << "XY" << std::endl;
              gds_file << real_lb_x + text_indent << " : " << real_lb_y + obs_indent << std::endl;
              break;
            default:
              std::cout << "[AStar Error] Invaild Direction 2d!" << std::endl;
              exit(1);
          }
          gds_file << "STRING " << convertToString(cost) << std::endl;
          gds_file << "ENDEL" << std::endl;
        }
      }
    }
    // start end path
    std::vector<Coordinate> coord_list = getCoordPath();

    for (size_t i = 0; i < coord_list.size(); i++) {
      Coordinate& coord = coord_list[i];

      int real_lb_x = coord.get_x() * factor;
      int real_lb_y = coord.get_y() * factor;
      int real_rt_x = (coord.get_x() + 1) * factor;
      int real_rt_y = (coord.get_y() + 1) * factor;

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
      gds_file << real_lb_x << " : " << real_lb_y << std::endl;
      gds_file << real_rt_x << " : " << real_lb_y << std::endl;
      gds_file << real_rt_x << " : " << real_rt_y << std::endl;
      gds_file << real_lb_x << " : " << real_rt_y << std::endl;
      gds_file << real_lb_x << " : " << real_lb_y << std::endl;
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

  coord_path.push_back(_end->get_coord());

  Node* temp_node = _path_head;

  while (temp_node != _start) {
    coord_path.push_back(temp_node->get_coord());
    temp_node = temp_node->get_parent_node();
  }
  coord_path.push_back(_start->get_coord());
  coord_path.push_back(_start->get_coord());

  for (size_t i = 0, j = (coord_path.size() - 1); i < j; i++, j--) {
    std::swap(coord_path[i], coord_path[j]);
  }

  return coord_path;
}

// Calculate known cost

double Model::getKnowCost(Node* start, Node* end)
{
  double cost = 0;
  cost += start->get_known_cost();
  cost += start->getCost(getOrientation(start, end));
  cost += end->getCost(getOrientation(end, start));
  cost += getManhattanDistance(start, end);
  cost += getKnowCornerCost(start, end);
  return cost;
}

double Model::getKnowCornerCost(Node* start, Node* end)
{
  if (start->get_parent_node()) {
    if (getDirection(start->get_parent_node(), start) != getDirection(start, end)) {
      return CORNER_UNIT;
    }
  }
  return 0;
}

// Calculate estimate cost

double Model::getEstimateCost(Node* start, Node* end)
{
  double est_length = getManhattanDistance(start, end);
  double est_cost = est_length * (2 * _min_cost);
  double est_corner_cost = getEstimateCornerCost(start, end);

  return (est_length + est_cost + est_corner_cost);
}

double Model::getEstimateCornerCost(Node* start, Node* end)
{
  if (start->get_coord() == end->get_coord()) {
    return 0;
  }

  std::vector<std::vector<Node*>> coord_path_list = tryRouting(start, end);

  int est_corner_num = 0;
  if (coord_path_list.size() > 0) {
    est_corner_num = getMinCornerNum(coord_path_list);
  } else {
    est_corner_num = 2;
  }
  return (est_corner_num * CORNER_UNIT);
}

std::vector<std::vector<Node*>> Model::tryRouting(Node* start, Node* end)
{
  std::vector<std::vector<Node*>> coord_path_list;

  if (isStraight(start, end)) {
    coord_path_list = routingStraight(start, end);
  } else {
    coord_path_list = routingLShape(start, end);
  }
  return coord_path_list;
}

std::vector<std::vector<Node*>> Model::routingStraight(Node* start, Node* end)
{
  std::vector<std::vector<Node*>> coord_path_list;

  if (passCheckingSegment(start, end)) {
    std::vector<Node*> coord_path;

    if (start->get_parent_node()) {
      coord_path.push_back(start->get_parent_node());
    }
    coord_path.push_back(start);
    coord_path.push_back(end);
    coord_path_list.push_back(coord_path);
  }
  return coord_path_list;
}

bool Model::passCheckingSegment(Node* start, Node* end)
{
  Direction direction = getDirection(start, end);

  Coordinate& start_coord = start->get_coord();
  Coordinate& end_coord = end->get_coord();

  if (direction == Direction::kHorizontal) {
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
      if (pre_node->isOBS(getOrientation(pre_node, curr_node))
          || curr_node->isOBS(getOrientation(curr_node, pre_node))) {
        return false;
      }
    }
  } else if (direction == Direction::kVertical) {
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

      if (pre_node->isOBS(getOrientation(pre_node, curr_node))
          || curr_node->isOBS(getOrientation(curr_node, pre_node))) {
        return false;
      }
    }
  } else {
    std::cout << "[AStar Error] Segment is not straight!" << std::endl;
    exit(1);
  }
  return true;
}

std::vector<std::vector<Node*>> Model::routingLShape(Node* start, Node* end)
{
  Coordinate& start_coord = start->get_coord();
  Coordinate& end_coord = end->get_coord();

  std::vector<std::vector<Node*>> coord_path_list;

  Node* Inflection_node1 = &_grid_map[start_coord.get_x()][end_coord.get_y()];
  if (passCheckingSegment(start, Inflection_node1) && passCheckingSegment(Inflection_node1, end)) {
    std::vector<Node*> coord_path;

    if (start->get_parent_node()) {
      coord_path.push_back(start->get_parent_node());
    }
    coord_path.push_back(start);
    coord_path.push_back(Inflection_node1);
    coord_path.push_back(end);
    coord_path_list.push_back(coord_path);
  }

  Node* Inflection_node2 = &_grid_map[end_coord.get_x()][start_coord.get_y()];
  if (passCheckingSegment(start, Inflection_node2) && passCheckingSegment(Inflection_node2, end)) {
    std::vector<Node*> coord_path;

    if (start->get_parent_node()) {
      coord_path.push_back(start->get_parent_node());
    }
    coord_path.push_back(start);
    coord_path.push_back(Inflection_node2);
    coord_path.push_back(end);
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
      if (getDirection(coord_path[j - 2], coord_path[j - 1])
          != getDirection(coord_path[j - 1], coord_path[j])) {
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

// base

Orientation Model::getOrientation(Node* start, Node* end)
{
  if (isPoint(start, end)) {
    return Orientation::kNone;
  }
  Coordinate& start_coord = start->get_coord();
  Coordinate& end_coord = end->get_coord();
  if (isHorizontal(start, end)) {
    return (start_coord.get_x() - end_coord.get_x()) > 0 ? Orientation::kWest : Orientation::kEast;
  } else if (isVertical(start, end)) {
    return (start_coord.get_y() - end_coord.get_y()) > 0 ? Orientation::kSouth : Orientation::kNorth;
  } else {
    std::cout << "[AStar Error] Segment is not straight!" << std::endl;
    exit(1);
  }
}

Direction Model::getDirection(Node* start, Node* end)
{
  if (isHorizontal(start, end)) {
    return Direction::kHorizontal;
  } else if (isVertical(start, end)) {
    return Direction::kVertical;
  } else {
    return Direction::kOblique;
  }
}

bool Model::isStraight(Node* start, Node* end)
{
  return isHorizontal(start, end) || isVertical(start, end);
}

bool Model::isHorizontal(Node* start, Node* end)
{
  Coordinate& start_coord = start->get_coord();
  Coordinate& end_coord = end->get_coord();
  return start_coord.get_y() == end_coord.get_y();
}

bool Model::isVertical(Node* start, Node* end)
{
  Coordinate& start_coord = start->get_coord();
  Coordinate& end_coord = end->get_coord();
  return start_coord.get_x() == end_coord.get_x();
}

bool Model::isPoint(Node* start, Node* end)
{
  Coordinate& start_coord = start->get_coord();
  Coordinate& end_coord = end->get_coord();
  return start_coord == end_coord;
}

int Model::getManhattanDistance(Node* start, Node* end)
{
  return std::abs(start->get_coord().get_x() - end->get_coord().get_x())
         + std::abs(start->get_coord().get_y() - end->get_coord().get_y());
}

}  // namespace astar
