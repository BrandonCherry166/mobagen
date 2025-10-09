#include "Cat.h"
#include "World.h"

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <queue>
#include <unordered_map>

//Helper Functions
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

//Heuristic
int HeuristicToBoundary(Point2D p, int minBound, int maxBound) {
  int dx = std::min(std::abs(p.x - minBound), std::abs(p.x - maxBound));
  int dy = std::min(std::abs(p.y - minBound), std::abs(p.y - maxBound));
  return (dx + dy + std::abs(dx - dy)) / 2; //Hex Distance
}

//Nodes!
struct Node {
  Point2D pos;
  int g; // Cost
  int f; //g + h

  bool operator>(const Node& other) const {
    return f > other.f;
  }
};


Point2D Cat::Move(World* world) {
  Point2D start = world -> getCat();
  int sideLength = world->getWorldSideSize();
  int halfLength = sideLength / 2;
  int minBound = -halfLength;
  int maxBound = (sideLength % 2 == 0) ? halfLength - 1 : halfLength;
  std::unordered_map<int,std::unordered_map<int, bool>> closed;
  std::unordered_map<int, std::unordered_map<int, Point2D>> parent;

  //Experimental
  std::unordered_map<int, std::unordered_map<int, int>> gScore;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  gScore[start.y][start.x] = 0;
  open.push({start, 0, HeuristicToBoundary(start, minBound, maxBound)}); //Add starting position

  while (!open.empty()) {
    Node current = open.top();
    open.pop();

    if (closed[current.pos.y][current.pos.x]) {
      continue; //Already Processed
    }
    closed[current.pos.y][current.pos.x] = true;

    if (current.pos.x == minBound || current.pos.y == minBound || current.pos.x == maxBound || current.pos.y == maxBound) { //Checking Boundaries
      std::vector<Point2D> path;
      for (Point2D v = current.pos; HasParent(parent, v); v = GetParent(parent, v)) {
        path.push_back(v);
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());

      if (path.size() > 1) {
        if (world->isValidPosition(path[1]) && !world->getContent(path[1]) && world->getCat() != path[1])
          return path[1]; //Take the first step
      }
      else {
        return path[0];
      }
    }
    std::vector<Point2D> neighbors =
      {
      world->E(current.pos),
      world->W(current.pos),
      world->NE(current.pos),
      world->NW(current.pos),
      world->SE(current.pos),
      world->SW(current.pos)
      };
    for (auto& next : neighbors) {
      if (!world->isValidPosition(next) || world->getContent(next)) {
        continue; //Skip walls and invalid spots
      }
      int tentG = current.g + 1;

      if (!gScore[next.y].count(next.x) || tentG < gScore[next.y][next.x]) {
        gScore[next.y][next.x] = tentG;
        SetParent(parent, next, current.pos);

        int f = tentG + HeuristicToBoundary(next, minBound, maxBound);
        open.push({next, tentG, f});
      }
    }
  }
  return start;
}

