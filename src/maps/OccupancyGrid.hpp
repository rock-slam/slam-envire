#ifndef _OCCUPANCYGRID_HPP_
#define _OCCUPANCYGRID_HPP_

#include <envire/maps/Grids.hpp>

namespace envire
{  

class OccupancyGrid : public Grid<float>
{
    ENVIRONMENT_ITEM( OccupancyGrid )
  public:
    static const std::string OCCUPANCY;
  private:
    const static std::vector<std::string> &bands; 

    //Initial probabiliity in odd-log form
    float l_0;
  public:
    OccupancyGrid() : Grid<float>() {};
    OccupancyGrid(size_t width, size_t height, double scalex, double scaley);
    ~OccupancyGrid(){};

    float getProbability(double posX, double posY);
    virtual const std::vector<std::string>& getBands() const {return bands;};
    virtual void clear(float initial_prob = 0.5);

    virtual bool updateProbability(double posX, double posY, double propability);
};

};

#endif
