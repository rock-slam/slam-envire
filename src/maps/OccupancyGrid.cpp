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

bool OccupancyGrid::updateProbability(double posX, double posY, double propability){
    size_t x,y;
    if(!toGrid(posX,posY,x,y)){
        return false;
    }
    
    //See Probailistic robotics Page 226 (Chapter 9 Algorithm occupancy grid mapping)
    float &value = getGridData()[x][y];
    value = value + log10f(propability/(1.0-propability)) - l_0;
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

float OccupancyGrid::getProbability(double posX, double posY)
{
    size_t x,y;
    if(!toGrid(posX,posY,x,y)){
        return std::numeric_limits<float>::signaling_NaN();
    }
    float value = powf(10,getGridData()[x][y]);
    //convert from odd-log to probability
    float result =  value/(1.0+value);
    return result;
}



