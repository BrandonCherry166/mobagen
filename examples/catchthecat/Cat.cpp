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
  Point2D start = world -> getCat();
  int sideLength = world->getWorldSideSize();
  int halfLength = sideLength / 2;
  int minBound = -halfLength;
  int maxBound = (sideLength % 2 == 0) ? halfLength - 1 : halfLength;
  std::unordered_map<int,std::unordered_map<int, bool>> visited;
  std::unordered_map<int, std::unordered_map<int, Point2D>> parent;

  std::queue<Point2D> q;

  q.push(start); //Add starting position
  SetVisited(visited, start);

  while (!q.empty()) {
    Point2D current = q.front();
    q.pop();

    std::cout << current.x << ", " << current.y << std::endl;
    if (current.x == minBound || current.y == minBound || current.x == maxBound || current.y == maxBound) { //Checking Boundaries
      std::vector<Point2D> path;
      for (Point2D v = current; HasParent(parent, v); v = GetParent(parent, v)) {
        path.push_back(v);
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      std::cout << "Path to Boundary: " << std::endl;
      for (auto &p: path) {
        std::cout << p.x << ", " << p.y << std::endl;
      }

      if (path.size() >= 2) {
        return path[1]; //Take the first step
      }
      else {
        return path[0];
      }
    }
    std::vector<Point2D> neighbors =
      {
      world->E(current),
      world->W(current),
      world->NE(current),
      world->NW(current),
      world->SE(current),
      world->SW(current)
      };
    for (auto& next : neighbors) {
      if (!IsVisited(visited, next) && world->catCanMoveToPosition(next)) {
        SetVisited(visited, next);
        SetParent(parent, next, current);
        q.push(next);
      }
    }
  }
  return start;
}
