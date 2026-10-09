#include "../include/multi_robot_explorer/frontier_cells.h"
#include <cassert>
#include <limits>

int main()
{
    assert(!frontier_cells::hasFreeCell({}));
    assert(!frontier_cells::hasFreeCell({-1, -1, 100}));
    assert(frontier_cells::hasFreeCell({-1, 0, 100}));
    assert(!frontier_cells::validGrid(0, 0, 0));
    assert(!frontier_cells::validGrid(3, 3, 8));
    assert(!frontier_cells::validGrid(3, 3, 10));
    assert(!frontier_cells::validGrid(std::numeric_limits<std::size_t>::max(), 2, 0));
    assert(frontier_cells::find({}, 0, 0).empty());
    assert(frontier_cells::find({0}, 1, 1).empty());
    assert(frontier_cells::find({0, -1, 0, -1}, 2, 2).empty());
    assert(frontier_cells::find({0}, 3, 3).empty());
    std::vector<std::int8_t> grid(25, 100);
    grid[12] = 0;
    assert(frontier_cells::find(grid, 5, 5).empty());
    grid[6] = -1;
    auto cells = frontier_cells::find(grid, 5, 5);
    assert(cells.size() == 1 && cells[0] == 12);
    grid[12] = 100;
    assert(frontier_cells::find(grid, 5, 5).empty());
    grid.assign(25, 0);
    grid[0] = -1;
    cells = frontier_cells::find(grid, 5, 5);
    assert(cells.size() == 1 && cells[0] == 6);
    return 0;
}
