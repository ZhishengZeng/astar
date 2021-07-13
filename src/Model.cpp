/*
 * @Author: ZengZhisheng
 * @Date: 2021-06-18 19:13:23
 * @Description:
 * @FilePath: /AStar/src/Model.cpp
 */

#include "Model.h"

#include "Util.h"
namespace astar {

void Model::setMapSize(int x_grid_num, int y_grid_num)
{
  assert(x_grid_num > 0 && y_grid_num > 0);

  _grid_map.init(x_grid_num, y_grid_num);
  for (int i = 0; i < (int) _grid_map.get_x_grid_num(); i++) {
    for (int j = 0; j < (int) _grid_map.get_y_grid_num(); j++) {
      _grid_map[i][j].set_coord(i, j);
    }
  }
}

Node* Model::setNode(const Coordinate& coord, const NodeType& node_type)
{
  assert(0 <= coord.get_x()
         && coord.get_x() < (int) _grid_map.get_x_grid_num());
  assert(0 <= coord.get_y()
         && coord.get_y() < (int) _grid_map.get_y_grid_num());

  Node& node = _grid_map[coord.get_x()][coord.get_y()];
  if (node.get_type() == NodeType::kNone) {
    node.set_type(node_type);
    return &node;
  } else {
    std::cout << "[AStar Error] [" << coord.get_x() << " , " << coord.get_y()
              << "] type is not null!!" << std::endl;
    exit(1);
  }
}

void Model::setObstacle(const std::vector<Coordinate>& obs_coord_list)
{
  for (size_t i = 0; i < obs_coord_list.size(); i++) {
    setNode(obs_coord_list[i], NodeType::kObs);
  }
}

void Model::showResult()
{
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

void printNode(Node& node)
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
          std::cout << "default???";
          break;
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
      std::cout << "default???";
      break;
  }
}

void Model::printGridMap()
{
  std::cout << "--------------------------------------------------\n";
  for (int j = _grid_map.get_y_grid_num() - 1; j >= 0; j--) {
    for (int i = 0; i < (int) _grid_map.get_x_grid_num(); i++) {
      printNode(_grid_map[i][j]);
    }
    std::cout << "\n";
  }
  std::cout << "--------------------------------------------------\n";
}

std::vector<Coordinate> Model::findPath(const Coordinate& start_coord,
                                        const Coordinate& end_coord)
{
  std::vector<Coordinate> path_coord;

  setStartNode(start_coord);
  setEndNode(end_coord);
  initStartNode();
  while (true) {
#if SHOWSTEPBYSTEP
    showResult();
#endif
    // 搜索最小代价点
    getMinCostNodeInOpenList();
#if SHOWSTEPBYSTEP
    showResult();
#endif
    // 是否抵达终点
    if (_curr_node->isEnd()) {
      break;
    }
    addNeighborNodesToOpenList();
    // 无路可走
    if (_open_list.empty()) {
      break;
    }
  }
#if SHOWRESULT
  showResult();
#endif
  path_coord = getPathCoord();

  if (_curr_node->isEnd()) {
    std::cout << "[AStar Info] Reached the end node!!";
    std::cout << " Path cost:" << _curr_node->get_total_cost();
  } else {
    std::cout << "[AStar Info] No Where!!";
  }
  std::cout << std::endl;
  return path_coord;
}

bool isHorizontal(Coordinate& start_coord, Coordinate& end_coord)
{
  return start_coord.get_y() == end_coord.get_y();
}

bool isVertical(Coordinate& start_coord, Coordinate& end_coord)
{
  return start_coord.get_x() == end_coord.get_x();
}

Direction getDirection(Node* start_node, Node* end_node)
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

double caculateEstCost(Node* a, Node* b)
{
  int length = std::abs(a->get_coord().get_x() - b->get_coord().get_x());
  int width = std::abs(a->get_coord().get_y() - b->get_coord().get_y());
#if DIAGONAL
  return std::abs(length - width) + (length < width ? length : width) * 1.414;
#else
  return length + width;
#endif
}

void Model::updateEstCost(Node* node)
{
  node->set_est_cost(caculateEstCost(node, _end_node));
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
  return walking_cost;
}

void Model::addNodeToOpenList(Node* node)
{
  _open_list.push(node);
  node->set_state(NodeState::kOpen);
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

  std::vector<Coordinate> offset_list;
#if DIAGONAL
  offset_list
      = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {-1, 1}, {1, -1}, {-1, -1}, {1, 1}};
#else
  offset_list = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
#endif

  for (size_t i = 0; i < offset_list.size(); i++) {
    int x = curr_coord_x + offset_list[i].get_x();
    int y = curr_coord_y + offset_list[i].get_y();
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
  return (0 <= x && x < (int) _grid_map.get_x_grid_num())
         && (0 <= y && y < (int) _grid_map.get_y_grid_num());
}

bool Model::isCurrBetterParent(Node* node)
{
  return (_curr_node->get_known_cost() + getCurrWalkingCost(node))
         < node->get_known_cost();
}

void Model::freeModel()
{
  _start_node = nullptr;
  _end_node = nullptr;
  _curr_node = nullptr;
}

}  // namespace astar
