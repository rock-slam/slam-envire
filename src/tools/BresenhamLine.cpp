#include "BresenhamLine.hpp"

envire::Bresenham::Bresenham(const Eigen::Vector2i& start, const Eigen::Vector2i& end)
{
    init(start.x(), start.y(), end.x(), end.y());
}

envire::Bresenham::Bresenham(const envire::GridBase::Position& start, const envire::GridBase::Position& end)
{
    init(start.x, start.y, end.x, end.y);
}

void envire::Bresenham::init(const envire::GridBase::Position& start, const envire::GridBase::Position& end)
{
    init(start.x, start.y, end.x, end.y);
}

void envire::Bresenham::init(int startX, int startY, int endX, int endY)
{
    x0 = startX;
    y0 = startY;
    x1 = endX;
    y1 = endY;
    dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
    dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1; 
    err = dx+dy; /* error value e_xy */
    hasP = true;
}

bool envire::Bresenham::getNextPoint(Eigen::Vector2i& p)
{
    return getNextPoint(p.x(), p.y());
}

bool envire::Bresenham::getNextPoint(envire::GridBase::Position& p)
{
    int x,y;
    bool ret = getNextPoint(x, y);
    p.x = x;
    p.y = y;
    return ret;
}

bool envire::Bresenham::getNextPoint(int& x, int& y)
{
    if(!hasP)
        return false;
    
    x = x0;
    y = y0;
    
    if (x0==x1 && y0==y1) 
    {
        hasP = false;
        return true;
    }
    
    e2 = 2*err;
    if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
    
    return true;
}
        
void envire::Bresenham::getLine(std::vector< envire::GridBase::Position >& positions
)
{
    GridBase::Position p;
    while(getNextPoint(p))
        positions.push_back(p);
}


void envire::lineBresenham(const envire::GridBase::Position& p1, const envire::GridBase::Position& p2, std::vector< envire::GridBase::Position >& positions)
{
    Bresenham bresenham(p1, p2);
    
    bresenham.getLine(positions);
}
