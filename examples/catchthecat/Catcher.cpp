#include "Catcher.h"
#include "World.h"

#include <algorithm>
#include <unordered_map>
#include <queue>

struct PointHash {
  std::size_t operator()(const Point2D& p) const noexcept {
    return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
  }
};

struct Node
{
  Point2D pos;
  int g, f;
  bool operator>(const Node& other) const {
    return f > other.f;
  }
};

static std::vector<Point2D> GetNeighbors(World* world, Point2D p) {
  return {
    world->E(p),
    world->W(p),
    world->NE(p),
    world->NW(p),
    world->SE(p),
    world->SW(p)
  };
}

int CountReachableTiles(World* world, Point2D start, const std::unordered_map<Point2D, bool, PointHash>& tempWalls) {
  std::queue<Point2D> q;
  std::unordered_map<Point2D, bool, PointHash> visited; q.push(start);
  visited[start] = true;
  int count = 0;
  while (!q.empty()) {
    Point2D p = q.front();
    q.pop();
    count++;
    for (auto& n : GetNeighbors(world, p)) {
      if (!world->isValidPosition(n)) {
        continue;
      }
      if (world->getContent(n)) {
        continue;
      }
      if (tempWalls.contains(n)) {
        continue;
      }
      if (visited[n]) {
        continue;
      }
      visited[n] = true;
      q.push(n);
    }
  }
  return count;
}
bool IsBoundary(Point2D p, int side) {
  if (p.x == -side || p.x == side || p.y == -side || p.y == side) {
    return true;
  }
  return false;
}

int Heuristic(Point2D a, Point2D b) {
  int dx = a.x - b.x;
  int dy = a.y - b.y;
  int dz = (a.x + a.y) - (b.x + b.y);
  return (std::abs(dx) + std::abs(dy) + std::abs(dz)) / 2;
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

static std::vector<Point2D> AStar(World* world, Point2D start, int side) {
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  std::unordered_map<Point2D, int, PointHash> gScore;
  std::unordered_map<Point2D, Point2D, PointHash> cameFrom;

  gScore[start] = 0;
  open.push({start, 0,0});

  Point2D goal = {-999, -999};
  bool found = false;

  while (!open.empty()) {
    Node current = open.top();
    open.pop();

    if (IsBoundary(current.pos, side)) {
      goal = current.pos;
      found = true;
      break;
    }

    for (auto& next : GetNeighbors(world, current.pos)){
      if (!world->isValidPosition(next)) {
        continue;
      }
      if (world->getContent(next)) {
        continue;
      }

      int tentative = gScore[current.pos] + 1;

      if (!gScore.contains(next) || tentative < gScore[next]) {
        gScore[next] = tentative;
        int h = std::min({
       Heuristic(next, { side, next.y }),
       Heuristic(next, { -side, next.y }),
       Heuristic(next, { next.x, side }),
       Heuristic(next, { next.x, -side })
     });
        open.push({next, tentative, tentative + h});
        cameFrom[next] = current.pos;
      }
    }
  }
  std::vector<Point2D> path;
  if (!found) {
    return path;
  }
  for (Point2D p = goal; cameFrom.contains(p); p=cameFrom[p]) {
    path.push_back(p);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

Point2D Catcher::Move(World* world) {
  int side = world->getWorldSideSize() / 2;
  Point2D catPos = world->getCat();
  auto path = AStar(world, catPos, side);
  if (path.empty()) {
   return {-1, -1};
  }

  std::vector<Point2D> candidates;
  for (int i = 0; i < (int)path.size() && i < 5; i++) {
    if (!world->getContent(path[i])) {
      candidates.push_back(path[i]);
    }
  }

  //Sort to prioritize edges
  std::sort(candidates.begin(), candidates.end(), [side](const Point2D& a, const Point2D& b) {
    auto distA = std::min({
    side - std::abs(a.x),
    side - std::abs(a.y),
    side - std::abs(a.x + a.y)});

    auto distB = std::min({
    side - std::abs(b.x),
    side - std::abs(b.y),
    side - std::abs(b.x + b.y)});

    return distA < distB;
  });

  Point2D best = {-1, -1};
  int bestScore = std::numeric_limits<int>::max();

  for (auto& tile: candidates) {
    std::unordered_map<Point2D, bool, PointHash> tempWalls;
    tempWalls[tile] = true;

    if (!CanCatEscape(world, catPos, tempWalls, side)) {
      return tile; //GG
    }

    //auto futurePath = AStar(world, catPos, side);
    int score = CountReachableTiles(world, catPos, tempWalls);

    int worstAfterCat = std::numeric_limits<int>::min();
    for (auto& n : GetNeighbors(world, catPos)) {
      if (!world->isValidPosition(n))
        continue;
      if (world->getContent(n))
        continue;
      if (tempWalls.contains(n))
        continue;

      int region = CountReachableTiles(world, n, tempWalls);
      worstAfterCat = std::max(worstAfterCat, region);
    }
    score = (worstAfterCat == std::numeric_limits<int>::min()) ? CountReachableTiles(world, catPos, tempWalls) : worstAfterCat;
    if (score < bestScore) {
      bestScore = score;
      best = tile;
    }
  }
  if (best.x != -1 && best.y != -1 && world->isValidPosition(best) && !world->getContent(best) && catPos != best) {
    return best;
  }
  //All Else Fails, pick a direction to throw a wall in that is valid
  for (auto n : GetNeighbors(world, catPos)) {
    if (world->isValidPosition(n) && !world->getContent(n)) {
      return n;
    }
  }
}

