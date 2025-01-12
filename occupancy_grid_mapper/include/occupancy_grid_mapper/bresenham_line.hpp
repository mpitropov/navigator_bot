#ifndef BRESENHAM_LINE_HPP
#define BRESENHAM_LINE_HPP

#include "occupancy_grid_mapper_node.hpp"

struct GridPoint
{
    int x;
    int y;
};

struct GridLine
{
    GridPoint start;
    GridPoint end;
};

void bresenhamLine(nav_msgs::msg::OccupancyGrid &map, const GridLine &line)
{
    int x0 = line.start.x;
    int y0 = line.start.y;
    int x1 = line.end.x;
    int y1 = line.end.y;

    const bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }

    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    const int dx = x1 - x0;
    const int dy = std::abs(y1 - y0);
    const int ystep = (y0 < y1) ? 1 : -1;

    int y = y0;
    int error = dx / 2;

    for (int x = x0; x <= x1; ++x)
    {
        const int idx = steep ? (x * map.info.width + y) : (y * map.info.width + x);

        // Ensure we are within map bounds
        if (idx >= 0 && idx < static_cast<int>(map.info.width * map.info.height))
        {
            // Only clear cells that are not already marked as obstacles
            if (map.data[idx] != OCCUPIED)
            {
                map.data[idx] = FREE;
            }
        }

        error -= dy;
        if (error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}

#endif // BRESENHAM_LINE_HPP
