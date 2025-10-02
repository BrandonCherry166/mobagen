#include "Cat.h"
#include "World.h"
#include <stdexcept>
#include <vector>
#include <queue>

Point2D Cat::Move(World* world) {
  int sideLength = world->getWorldSideSize();
  Point2D pos = world->getCat();
  int dr_even[6] = {-1, -1, 0, 0, 1, 1};
  int dc_even[6] = {0, 1, -1, 1, 0, 1};
  int dr_odd[6] = {-1, -1, 0, 0, 1, 1};
  int dc_odd[6] = {-1, 0, -1, 1, -1, 0};

  std::vector<std::vector<bool>> visited (sideLength, std::vector<bool>(sideLength, false)); //2D Vector of Visited Cells
  std::vector<std::vector<Point2D>> parent (sideLength, std::vector<Point2D>(sideLength, Point2D(-1, -1)));

  std::queue<Point2D> q;

  q.push(pos); //Add starting position
  visited[pos.y][pos.x] = true;


}
