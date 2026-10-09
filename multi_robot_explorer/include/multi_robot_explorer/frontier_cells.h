#ifndef MULTI_ROBOT_EXPLORER_FRONTIER_CELLS_H
#define MULTI_ROBOT_EXPLORER_FRONTIER_CELLS_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <algorithm>

namespace frontier_cells
{
inline bool hasFreeCell(const std::vector<std::int8_t> &data)
{
    return std::find(data.begin(), data.end(), 0) != data.end();
}

inline bool validGrid(std::size_t width, std::size_t height, std::size_t size)
{
    return width > 0 && height > 0 && size / height == width && size % height == 0;
}

inline std::vector<std::size_t> find(const std::vector<std::int8_t> &data,
                                    std::size_t width, std::size_t height)
{
    std::vector<std::size_t> cells;
    if (!validGrid(width, height, data.size()) || width < 3 || height < 3)
        return cells;
    for (std::size_t y = 1; y < height - 1; ++y)
    {
        for (std::size_t x = 1; x < width - 1; ++x)
        {
            const std::size_t i = y * width + x;
            if (data[i] != 0)
                continue;
            bool unknown = false;
            for (std::size_t ny = y - 1; ny <= y + 1 && !unknown; ++ny)
                for (std::size_t nx = x - 1; nx <= x + 1; ++nx)
                    if (data[ny * width + nx] == -1)
                    {
                        unknown = true;
                        break;
                    }
            if (unknown)
                cells.push_back(i);
        }
    }
    return cells;
}
}

#endif
