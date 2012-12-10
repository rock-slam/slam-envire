#ifndef ENVIRE_BRESENHAM_LINE__
#define ENVIRE_BRESENHAM_LINE__

#include <envire/maps/GridBase.hpp>

namespace envire
{

/**
 * Draws a line between two points p1(p1x,p1y) and p2(p2x,p2y).
 * This function is based on the Bresenham's line algorithm and is highly 
 * optimized to be able to draw lines very quickly. There is no floating point 
 * arithmetic nor multiplications and divisions involved. Only addition, 
 * subtraction and bit shifting are used. 
 *
 * Note that you have to define your own customized setPixel(x,y) function, 
 * which essentially lights a pixel on the screen.
 *
 * Author: Woon Khang Tang
 * url: http://www.codekeep.net/snippets/e39b2d9e-0843-4405-8e31-44e212ca1c45.aspx
 * (c) All public snippets in CodeKeep may be freely used, copied, and distributed. 
 *
 * @todo update to iterator like accessor
 */
void lineBresenham(const envire::GridBase::Position& p1, const envire::GridBase::Position& p2, std::vector<envire::GridBase::Position>& positions )
{
    int p1x = p1.x, p1y = p1.y;
    int p2x = p2.x, p2y = p2.y;
    int F, x, y;

    if (p1x > p2x)  // std::swap points if p1 is on the right of p2
    {
        std::swap(p1x, p2x);
        std::swap(p1y, p2y);
    }

    // Handle trivial cases separately for algorithm speed up.
    // Trivial case 1: m = +/-INF (Vertical line)
    if (p1x == p2x)
    {
        if (p1y > p2y)  // std::swap y-coordinates if p1 is above p2
        {
            std::swap(p1y, p2y);
        }

        x = p1x;
        y = p1y;
        while (y <= p2y)
        {
	    positions.push_back( envire::GridBase::Position( x, y ) );
            y++;
        }
        return;
    }
    // Trivial case 2: m = 0 (Horizontal line)
    else if (p1y == p2y)
    {
        x = p1x;
        y = p1y;

        while (x <= p2x)
        {
	    positions.push_back( envire::GridBase::Position( x, y ) );
            x++;
        }
        return;
    }


    int dy            = p2y - p1y;  // y-increment from p1 to p2
    int dx            = p2x - p1x;  // x-increment from p1 to p2
    int dy2           = (dy << 1);  // dy << 1 == 2*dy
    int dx2           = (dx << 1);
    int dy2_minus_dx2 = dy2 - dx2;  // precompute constant for speed up
    int dy2_plus_dx2  = dy2 + dx2;


    if (dy >= 0)    // m >= 0
    {
        // Case 1: 0 <= m <= 1 (Original case)
        if (dy <= dx)   
        {
            F = dy2 - dx;    // initial F

            x = p1x;
            y = p1y;
            while (x <= p2x)
            {
		positions.push_back( envire::GridBase::Position( x, y ) );
                if (F <= 0)
                {
                    F += dy2;
                }
                else
                {
                    y++;
                    F += dy2_minus_dx2;
                }
                x++;
            }
        }
        // Case 2: 1 < m < INF (Mirror about y=x line
        // replace all dy by dx and dx by dy)
        else
        {
            F = dx2 - dy;    // initial F

            y = p1y;
            x = p1x;
            while (y <= p2y)
            {
		positions.push_back( envire::GridBase::Position( x, y ) );
                if (F <= 0)
                {
                    F += dx2;
                }
                else
                {
                    x++;
                    F -= dy2_minus_dx2;
                }
                y++;
            }
        }
    }
    else    // m < 0
    {
        // Case 3: -1 <= m < 0 (Mirror about x-axis, replace all dy by -dy)
        if (dx >= -dy)
        {
            F = -dy2 - dx;    // initial F

            x = p1x;
            y = p1y;
            while (x <= p2x)
            {
		positions.push_back( envire::GridBase::Position( x, y ) );
                if (F <= 0)
                {
                    F -= dy2;
                }
                else
                {
                    y--;
                    F -= dy2_plus_dx2;
                }
                x++;
            }
        }
        // Case 4: -INF < m < -1 (Mirror about x-axis and mirror 
        // about y=x line, replace all dx by -dy and dy by dx)
        else    
        {
            F = dx2 + dy;    // initial F

            y = p1y;
            x = p1x;
            while (y >= p2y)
            {
		positions.push_back( envire::GridBase::Position( x, y ) );
                if (F <= 0)
                {
                    F += dx2;
                }
                else
                {
                    x++;
                    F += dy2_plus_dx2;
                }
                y--;
            }
        }
    }
}

}

#endif
