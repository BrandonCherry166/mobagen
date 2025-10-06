#include "Catcher.h"
#include "World.h"
#include <unordered_map>
#include <queue>

struct PointHash {
  std::size_t operator()(const Point2D& p) const noexcept {
    return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
  }
};

bool IsBoundary(Point2D p, int side) {
  if (p.x == -side || p.x == side || p.y == -side || p.y == side) {
    return true;
  }
  return false;
}
bool CanCatEscape(World* world, Point2D start, const std::unordered_map <Point2D, bool, PointHash>& tempWalls, int side) {
  std::queue <Point2D> queue;
  std::unordered_map <Point2D, bool, PointHash> visited;

  queue.push(start);
  visited[start] = true;

  while (!queue.empty()) {
    Point2D current = queue.front();
    queue.pop();

    if (IsBoundary(current, side)) {
      return true; //Escape Found
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

    for (int i = 0; i < neighbors.size(); i++) {
      Point2D next = neighbors[i];

      if (!world->isValidPosition(next)) {
        continue;
      }
      if (world->getContent(next)) {
        continue;
      }
      if (tempWalls.contains(next)) {
        continue;
      }
      if (visited[next]) {
        continue;
      }

      visited[next] = true;
      queue.push(next);
    }
  }
  return false; //No Escape!
}

Point2D Catcher::Move(World* world) {
  auto side = world->getWorldSideSize() / 2;
  Point2D catPos = world->getCat();
  std::vector<Point2D> openTiles;

  //Get Open Tiles
  for (int i = -side; i < side; i++) {
    for (int j = -side; j < side; j++) {
      Point2D temp(i, j);
      if (!world->getContent(temp)) {
        openTiles.push_back(temp);
      }
    }
  }

  Point2D bestTile = {-1, -1};
  int bestScore = -1;

  for (auto& tile : openTiles) {
    std::unordered_map<Point2D, bool, PointHash> temp;
    temp[tile] = true;

    bool escapes = CanCatEscape(world, catPos, temp, side);
    if (!escapes) {
      return tile; //GG
    }

    //Not trapped, so measure distance
    std::queue<Point2D> q;
    std::unordered_map<Point2D, int, PointHash> dist;

    q.push(catPos);
    dist[catPos] = 0;

    int shortest = std::numeric_limits<int>::max();

    while (!q.empty()) {
      Point2D current = q.front();
      q.pop();

      if (IsBoundary(current, side)) {
        shortest = dist[current];
        break;
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

      for (int i = 0; i < neighbors.size(); i++) {
        Point2D next = neighbors[i];
        if (!world->isValidPosition(next)) {
          continue;
        }
        if (world->getContent(next)) {
          continue;
        }

        if (temp.contains(next)) {
          continue;
        }
        if (dist.contains(next)) {
          continue;
        }

        dist[next] = dist[current] + 1;
        q.push(next);
      }
    }
    if (shortest < bestScore) {
      continue;
    }
    bestScore = shortest;
    bestTile = tile;
  }
  return bestTile;
}
