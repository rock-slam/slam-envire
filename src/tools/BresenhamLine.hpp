#ifndef ENVIRE_BRESENHAM_LINE__
#define ENVIRE_BRESENHAM_LINE__

#include <envire/maps/GridBase.hpp>

namespace envire
{

/**
* Convenience method that returns all points on a line between p1 and p2.
* */
void lineBresenham(const envire::GridBase::Position& p1, const envire::GridBase::Position& p2, std::vector<envire::GridBase::Position>& positions );

/**
 * This class use usefull for calculation
 * straight lines in grid. The Bresenham Algrorithm 
 * is a fast but inaccurate way to do this. 
 * Inaccurate means, that the result will have
 * aliasing artifacts.
 * 
 * The implementation of the core algorithm was taken from an wikipedia article.
 * */
class Bresenham {
    public:
        Bresenham(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
        Bresenham(const GridBase::Position& start, const GridBase::Position& end);
        
        /**
         * Inits the algorithm.
         * The method may be used to 'reinit' the class
         * after a line was allready interpolated. 
         * */
        void init(const GridBase::Position& start, const GridBase::Position& end);
        void init(int startX, int startY, int endX, int endY);

        /**
         * Calculates the next point in the line 
         * and returns it over the given paramters.
         * 
         * returns false if the end point is reached an
         *              no further point can be calculated.
         * 
         * */
        bool getNextPoint(int &x, int &y);
        bool getNextPoint(Eigen::Vector2i &p);
        bool getNextPoint(GridBase::Position &p);
        
        /**
         * Convenience method that returns all points on a line.
         * */
        void getLine(std::vector<envire::GridBase::Position>& positions);
    private:
        bool hasP;
        int x0,y0, x1, y1, dx, sx, dy, sy, err, e2;

};


}

#endif
