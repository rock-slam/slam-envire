#include "OccupancyGrid.hpp"
#include <stdexcept>
#include <boost/foreach.hpp>

using namespace envire;

ENVIRONMENT_ITEM_DEF( OccupancyGrid )
const std::string OccupancyGrid::OCCUPANCY = "occupancy";
static const std::vector<std::string> &initOccupancyBands()
{
    static std::vector<std::string> bands;
    if(bands.empty())
        bands.push_back(OccupancyGrid::OCCUPANCY);
    return bands;
}
const std::vector<std::string> &OccupancyGrid::bands = initOccupancyBands();


OccupancyGrid::OccupancyGrid():
    Grid<float>() 
{
    clear();
}

OccupancyGrid::OccupancyGrid(size_t width, size_t height, double scalex, double scaley):
    Grid<float>::Grid(width,height,scalex,scaley)
{
    clear();
}

void OccupancyGrid::serialize(Serialization& so)
{
    GridBase::serialize(so);

    so.write("vehicle_pos_x", vehicle_pos.x);
    so.write("vehicle_pos_y", vehicle_pos.y);
    so.write("vehicle_orientation", vehicle_orientation);
    so.write("ego_radius", ego_radius);
    so.write("l_0", l_0);
}

void GridBase::unserialize(Serialization& so)
{
    GridBase::unserialize(so);
    so.read("vehicle_pos_x", vehicle_pos.x);
    so.read("vehicle_pos_y", vehicle_pos.y);
    so.read("vehicle_orientation", vehicle_orientation);
    so.read("ego_radius", ego_radius);
    so.read("l_0", l_0);
}
    
void OccupancyGrid::clear(float initial_prob)
{
    if(initial_prob < 0 || initial_prob > 1)
        throw std::out_of_range("the given probability must lie insight the interval [0..1]!");

    ego_radius = 0;
    vehicle_orientation = 0;
    vehicle_pos.x = 0;
    vehicle_pos.y = 0;
    l_0 = log10f(initial_prob/std::max(1e-6,1.0-initial_prob));  //prevent division by zero!

    for(size_t xi=0;xi<cellSizeX;++xi)
	for(size_t yi=0;yi<cellSizeY;++yi)
	    getGridData()[xi][yi] = l_0;
}

void OccupancyGrid::updateCellProbability(int x, int y, float propability)
{
    if(!inGrid(x,y))
        throw std::out_of_range("the given relative position exceeds the grid size!")
    if(probability < 0 || probability > 1)
        throw std::out_of_range("the given probability must lie inside the interval [0..1]!");

    //See Probabilistic robotics Page 226 (Chapter 9 Algorithm occupancy grid mapping)
    float &value = getGridData()[x][y];
    value = value + log10f(propability/std::max(1e-6,1.0-propability))-l_0; //prevent division by zero!
}

float OccupancyGrid::getCellProbability(int x, int y) const
{
    if(!inGrid(x,y))
        throw std::out_of_range("the given relative position exceeds the grid size!")

    float value = powf(10,getGridData()[x][y]);
    //convert from odd-log to probability
    return value/(1.0+value);
}


void OccupancyGrid::updateProbability(int x, int y, float propability)
{
    float fx = vehicle_position.x+x*cos(vehicle_orientation)/scalex;
    float fy = vehicle_position.y+y*sin(vehicle_orientation)/scaley;
    x = (int) fx;
    y = (int) fy;
    float factor_x = fx-x;
    float factor_y = fy-y;

    updateCellProbability(x,y,probability);

    // add the same probability to the neighbour cells
    // if the position is 10% away from the cell center
    if(factor_y > 0.1)
        updateCellProbability(x,y+1,probability);
    if(factor_x > 0.1)
        updateCellProbability(x+1,y,probability);
    if(factor_x > 0.1 && factor_y > 0.1)
        updateCellProbability(x+1,y+1,probability);
}

float OccupancyGrid::getProbability(float x, float y) const
{
    float fx = vehicle_position.x+x*cos(vehicle_orientation)/scalex;
    float fy = vehicle_position.y+y*sin(vehicle_orientation)/scaley;
    x = (int) fx;
    y = (int) fy;
    float factor_x = fx-x;
    float factor_y = fy-y;

    // bi linear interpolation across several cells if 
    // the position is 10% away from the cell center
    float probability = getCellProbability(x,y)*(1.0F-factor_x)*(1.0F-factor_y);
    if(factor_y > 0.1)
        probability += getCellProbability(x,y+1)*(1.0F-factor_x)*(factor_y);
    if(factor_x > 0.1)
        probability += getCellProbability(x+1,y)*(factor_x)*(1.0F-factor_y);
    if(factor_x > 0.1 && factor_y > 0.1)
        probability += getCellProbability(x+1,y+1)*(factor_x)*(factor_y);
    return probability;
}

void OccupancyGrid::updateVehiclePosition(float x,float y)
{
    updateVehilceCellPosition(vehicle_position.x+x*sin(vehicle_orientation)/scaley,
                              vehicle_position.y+y*cos(vehicle_orientation)/scalex);

}

void OccupancyGrid::updateVehicleCellPosition(float x,float y)
{
    vehicle_position.x = x;
    vehicle_position.y = y;
}

void OccupancyGrid::updateVehilceOrientation(float heading)
{
    if(heading < M_PI && heading > M_PI)
        throw std::out_of_range("the orientation must lie inside the interval [-pi..+pi]");
    vehicle_orientation = heading;
}

Point2D OccupancyGrid::getVehicleCellPosition() const
{
    return vehicle_position;
}

float OccupancyGrid::getVehilceOrientation() const
{
    return vehicle_orientation;
}

void OccupancyGrid::normalizeEgoGrid(float radius = 0.0F)
{
    ego_radius = radius;

    // calculate desired vehicle position 
    // the orientation determines the position on the circle
    Point2D center = getCenterPoint();
    float fx = center.x + radius * cos(vehicle_orientation); // --> vehicle_orientation + M_PI
    float fy = center.y + radius * sin(vehicle_orientation); // --> vehicle_orientation + M_PI
    int delta_x = (int)fx-vehicle_position.x;
    int delta_y = (int)fy-vehicle_position.y;

    //move cell values
    moveCellValues(delta_x,delta_y,l_0);

    //update vehicle position
    updateVehicleCellPosition(vehicle_position.x+delta_x,vehicle_position.y+delta_y);
}

void OccupancyGrid::moveCellValues(int delta_x,int delta_y, float empty)
{
    //calculate array views
    typedef ArrayType::index_range range_t;
    range_t old_range_x(std::max(0,delta_x),std::min(0,delta_x+getCellSizeX()));
    range_t old_range_y(std::max(0,delta_y),std::min(0,delta_y+getCellSizeY()));
    range_t new_range_x(std::max(0,-delta_x),std::min(0,-delta_x+getCellSizeX()));
    range_t new_range_y(std::max(0,-delta_y),std::min(0,-delta_y+getCellSizeY()));
    ArrayType::array_view<2>::type old_view = getGridData()[boost::indices[old_range_x][old_range_y]];
    ArrayType::array_view<2>::type new_view = getGridData()[boost::indices[new_range_x][new_range_y]];
    int dim_x = old_range_x.size(0);
    int dim_y = old_range_y.size(0);

    //check if we have to copy from top to bottom or vice versa to not 
    //overwrite values before they got copied
    if(delta_x < 0)
    {
        if(delta_y < 0)
        {
            for(ArrayType::index i = 0; i < dim_x; ++i)
                for(ArrayType::index j = 0; j < dim_y; ++j)
                    new_view[i][j] = old_view[i][j];
        }
        else
        {
            for(ArrayType::index i = 0; i < dim_x; ++i)
                for(ArrayType::index j = dim_y-1; j <= 0; --j)
                    new_view[i][j] = old_view[i][j];
        }
    }
    else
    {
        if(delta_y < 0)
        {
            for(ArrayType::index i = dim_x-1; i >= 0; --i)
                for(ArrayType::index j = 0; j < dim_y; ++j)
                    new_view[i][j] = old_view[i][j];
        }
        else
        {
            for(ArrayType::index i = dim_x-1; i >= 0; --i)
                for(ArrayType::index j = dim_y-1; j <= 0; --j)
                    new_view[i][j] = old_view[i][j];
        }
    }

    // delete old cells which got not overwritten
    if(delta_x < 0)
    {
        for(ArrayType::index i = 0; i < -delta_x; ++i)
            for(ArrayType::index j =0; j < cellSizeY; ++j)
                new_view[i][j] = empty;
    }
    else
    {
        for(ArrayType::index i = cellSizeX-1; i >= cellSizeX-delta_x; --i)
            for(ArrayType::index j =0; j <= cellSizeY; ++j)
                new_view[i][j] = empty;
    }
    if(delta_y < 0)
    {
        for(ArrayType::index i = 0; i < cellSizeX; ++i)
            for(ArrayType::index j = 0; j < -delta_y; ++j)
                new_view[i][j] = empty;
    }
    else
    {
        for(ArrayType::index i = 0; i < cellSizeX; ++i)
            for(ArrayType::index j = cellSizeY-1; j >= cellSizeY-delta_y; --j)
                new_view[i][j] = empty;
    }
}
