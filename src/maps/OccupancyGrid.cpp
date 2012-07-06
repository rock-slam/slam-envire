#include "OccupancyGrid.hpp"

using namespace envire;

ENVIRONMENT_ITEM_DEF( OccupancyGrid )
const std::string OccupancyGrid::OCCUPANCY = "occupancy";
static const std::vector<std::string> &initOccupancyBands()
{
  static std::vector<std::string> bands;
  if(bands.empty())
  {
    bands.push_back(OccupancyGrid::OCCUPANCY);
  }
  return bands;
}
const std::vector<std::string> &OccupancyGrid::bands = initOccupancyBands();
    
    
OccupancyGrid::OccupancyGrid(size_t width, size_t height, double scalex, double scaley):
    Grid<float>::Grid(width,height,scalex,scaley){
        clear();
}

bool OccupancyGrid::updateProbability(int x, int y, float propability){
    if(!inGrid(x,y))
        return false;
    
    //See Probailistic robotics Page 226 (Chapter 9 Algorithm occupancy grid mapping)
    float &value = getGridData()[x][y];
    value = value + log10f(propability/std::max(1e-6,1.0-propability)) - l_0;
    return true; 
};

void OccupancyGrid::clear(float initial_prob)
{
    l_0 = log10f(initial_prob/(1.0-initial_prob));
    for(size_t xi=0;xi<cellSizeX;xi++)
    {
	for(size_t yi=0;yi<cellSizeY;yi++)
	{
	    getGridData()[xi][yi] = l_0;
	}
    }
}

bool OccupancyGrid::getProbability(int x, int y,float &probability) const
{
    if(!inGrid(x,y))
        return false;

    float value = powf(10,getGridData()[x][y]);
    //convert from odd-log to probability
    probability =  value/(1.0+value);
    return true;
}
