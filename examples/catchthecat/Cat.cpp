#include "Cat.h"
#include "World.h"

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <queue>
#include <unordered_map>

bool IsVisited(std::unordered_map<int,std::unordered_map<int, bool>>& visited, Point2D& position) {
  if (visited.contains(position.y) && visited[position.y].contains(position.x)) {
    return true;
  }
  return false;
}

void SetVisited(std::unordered_map<int,std::unordered_map<int, bool>>& visited, Point2D& position) {
  visited[position.y][position.x] = true;
}

bool HasParent(std::unordered_map<int, std::unordered_map<int, Point2D>>& parent, Point2D& position) {
  if (parent.contains(position.y) && parent[position.y].contains(position.x)) {
    return true;
  }
  return false;
}

void SetParent(std::unordered_map<int, std::unordered_map<int, Point2D>>& parent, Point2D& child, Point2D& p) {
  parent[child.y][child.x] = p;
}

Point2D GetParent(std::unordered_map<int, std::unordered_map<int, Point2D>>& parent, Point2D& position) {
  return parent[position.y][position.x];
}

Point2D Cat::Move(World* world) {
  int sideLength = world->getWorldSideSize();
  Point2D pos = world->getCat();
  int dr_even[6] = {-1, -1, 0, 0, 1, 1};
  int dc_even[6] = {0, 1, -1, 1, 0, 1};
  int dr_odd[6] = {-1, -1, 0, 0, 1, 1};
  int dc_odd[6] = {-1, 0, -1, 1, -1, 0};

  std::unordered_map<int,std::unordered_map<int, bool>> visited;
  std::unordered_map<int, std::unordered_map<int, Point2D>> parent;

  std::queue<Point2D> q;

  q.push(pos); //Add starting position
  SetVisited(visited, pos);

  while (!q.empty()) {
    Point2D current = q.front();
    q.pop();

    if (!world->isValidPosition(current)) { //Checking Boundaries
      std::vector<Point2D> path;
      for (Point2D v = current; HasParent(parent, v); v = GetParent(parent, v)) {
        path.push_back(v);
      }
      path.push_back(pos);
      reverse(path.begin(), path.end());

      if (path.size() > 1) {
        return path[1]; //Take the first step
      }
      return pos;
    }

    int *dr = (current.y % 2 == 0 ? dr_even : dr_odd);
    int *dc = (current.y % 2 == 0 ? dc_even : dc_odd);

    for (int k = 0; k < 6; k++) {
      int nx = current.x + dc[k];
      int ny = current.y + dr[k];
      Point2D temp (nx,ny);
      if (world->isValidPosition(temp) && !IsVisited(visited, temp) && world->catCanMoveToPosition(temp)) {
        SetVisited(visited, temp);
        SetParent(parent, temp, current);
        q.push(temp);
      }
    }
  }
  return pos;
}
