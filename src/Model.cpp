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
    std::cout << "[Error] [" << coord.get_x() << " , " << coord.get_y()
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
  while (temp_node->get_coord() != _start_node->get_coord()) {
    temp_node->set_on_path(on_path);
    temp_node = temp_node->get_parent_node();
  }
  _start_node->set_on_path(on_path);
}

void printNode(Node& node)
{
  int curr_cost = node.get_known_cost();
  int est_cost = node.get_est_cost();

  switch (node.get_type()) {
    case NodeType::kNone:
      switch (node.get_state()) {
        case NodeState::kNone:
          printf("\33[47m[%02d+%02d]\033[0m", 0, 0);
          break;
        case NodeState::kOpen:
          printf("\33[41m[%02d+%02d]\033[0m", curr_cost, est_cost);
          break;
        case NodeState::kClose:
          if (node.get_on_path()) {
            printf("\33[45;30m[%02d+%02d]\033[0m", curr_cost, est_cost);
          } else {
            printf("\33[45m[%02d+%02d]\033[0m", curr_cost, est_cost);
          }
          break;
        default:
          std::cout << "default???";
          break;
      }
      break;
    case NodeType::kObs:
      printf("\33[40m[%02d+%02d]\033[0m", 0, 0);
      break;
    case NodeType::kStart:
      printf("\33[42m[%02d+%02d]\033[0m", curr_cost, est_cost);
      break;
    case NodeType::kEnd:
      printf("\33[44m[%02d+%02d]\033[0m", curr_cost, est_cost);
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

void Model::findPath(const Coordinate& start_coord, const Coordinate& end_coord)
{
  double start_time, end_time;
  start_time = Util::microtime();

  setStartNode(start_coord);
  setEndNode(end_coord);
  initStartNode();
  while (true) {
#if SHOWSTEPBYSTEP
    showResult();
#endif
    // 搜索最小代价点
    getMinCostNodeInOpenList();
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

  end_time = Util::microtime();
  showResult();
  std::cout << (_curr_node->isEnd() ? "[Info] Reached the end node!!"
                                    : "[Info] No Where!!")
            << " time:" << (end_time - start_time) << std::endl;
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

int caculateEstCost(Node* a, Node* b)
{
  return std::abs(a->get_coord().get_x() - b->get_coord().get_x())
         + std::abs(a->get_coord().get_y() - b->get_coord().get_y());
}

void Model::updateEstCost(Node* node)
{
  node->set_est_cost(caculateEstCost(node, _end_node));
}

void Model::updateParentByCurr(Node* node)
{
  node->set_known_cost(_curr_node->get_known_cost() + 1);
  node->set_parent_node(_curr_node);
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
      if (currIsBetterThan(neighbor_node->get_parent_node())) {
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
  if (curr_coord_x > 0) {
    neighbor_node_list.push_back(&_grid_map[curr_coord_x - 1][curr_coord_y]);
  }
  if (curr_coord_x < (int) (_grid_map.get_x_grid_num() - 1)) {
    neighbor_node_list.push_back(&_grid_map[curr_coord_x + 1][curr_coord_y]);
  }
  if (curr_coord_y > 0) {
    neighbor_node_list.push_back(&_grid_map[curr_coord_x][curr_coord_y - 1]);
  }
  if (curr_coord_y < (int) (_grid_map.get_y_grid_num() - 1)) {
    neighbor_node_list.push_back(&_grid_map[curr_coord_x][curr_coord_y + 1]);
  }
}

bool Model::currIsBetterThan(Node* node)
{
  return _curr_node->get_total_cost() < node->get_total_cost();
}

}  // namespace astar
