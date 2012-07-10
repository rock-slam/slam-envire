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

OccupancyGrid::~OccupancyGrid()
{
}

OccupancyGrid::OccupancyGrid(size_t width, size_t height, double scalex, double scaley):
    Grid<float>::Grid(width,height,scalex,scaley)
{
    clear();
}

void OccupancyGrid::serialize(Serialization& so)
{
    GridBase::serialize(so);

    so.write("vehicle_position_x", vehicle_position.x());
    so.write("vehicle_position_y", vehicle_position.y());
    so.write("vehicle_orientation", vehicle_orientation);
    so.write("ego_radius", ego_radius);
    so.write("l_0", l_0);
}

void OccupancyGrid::unserialize(Serialization& so)
{
    GridBase::unserialize(so);
    so.read("vehicle_position_x", vehicle_position.x());
    so.read("vehicle_position_y", vehicle_position.y());
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
    Point2D center = getCenterPoint();
    updateVehicleCellPosition(center.x()/scalex, center.y()/scaley);
    l_0 = log10f(initial_prob/std::max(1e-6,1.0-initial_prob));  //prevent division by zero!

    ArrayType array = getGridData();
    for(size_t xi=0;xi<cellSizeX;++xi)
	for(size_t yi=0;yi<cellSizeY;++yi)
	    array[xi][yi] = l_0;
}

void OccupancyGrid::updateCellProbability(int x, int y, float probability)
{
    if(!inGrid(x,y))
        throw std::out_of_range("the given relative position exceeds the grid size!");
    if(probability < 0 || probability > 1)
        throw std::out_of_range("the given probability must lie inside the interval [0..1]!");

    //See Probabilistic robotics Page 226 (Chapter 9 Algorithm occupancy grid mapping)
    float &value = getGridData()[x][y];
    value = value + log10f(probability/std::max(1e-6,1.0-probability))-l_0; //prevent division by zero!
    std::cout << "update cell " << x << "/" << y << " = " << value << std::endl;
}

float OccupancyGrid::getCellProbability(int x, int y) const
{
    if(!inGrid(x,y))
        throw std::out_of_range("the given relative position exceeds the grid size!");

    float value = powf(10,getGridData()[x][y]);
    //convert from odd-log to probability
    return value/(1.0+value);
}


void OccupancyGrid::updateProbability(float x, float y, float probability)
{
    float fx = vehicle_position.x()+x*cos(-vehicle_orientation)/scalex-y*sin(-vehicle_orientation)/scaley;
    float fy = vehicle_position.y()+x*sin(-vehicle_orientation)/scalex+y*cos(-vehicle_orientation)/scaley;
    int ix = (int) fx;
    int iy = (int) fy;
    float factor_x = fx-ix;
    float factor_y = fy-iy;

    updateCellProbability(ix,iy,probability);

    // add the same probability to the neighbour cells
    // if the position is 10% away from the cell center
    if(factor_y > 0.1)
        updateCellProbability(ix,iy+1,probability);
    if(factor_x > 0.1)
        updateCellProbability(ix+1,iy,probability);
    if(factor_x > 0.1 && factor_y > 0.1)
        updateCellProbability(ix+1,iy+1,probability);
}

float OccupancyGrid::getProbability(float x, float y) const
{
    float fx = vehicle_position.x()+x*cos(-vehicle_orientation)/scalex-y*sin(-vehicle_orientation)/scaley;
    float fy = vehicle_position.y()+x*sin(-vehicle_orientation)/scalex+y*cos(-vehicle_orientation)/scaley;
    int ix = (int) fx;
    int iy = (int) fy;
    float factor_x = fx-ix;
    float factor_y = fy-iy;

    // bi linear interpolation across several cells if 
    // the position is 10% away from the cell center
    float probability = getCellProbability(ix,iy)*(1.0F-factor_x)*(1.0F-factor_y);
    if(factor_y > 0.1)
        probability += getCellProbability(ix,iy+1)*(1.0F-factor_x)*(factor_y);
    if(factor_x > 0.1)
        probability += getCellProbability(ix+1,iy)*(factor_x)*(1.0F-factor_y);
    if(factor_x > 0.1 && factor_y > 0.1)
        probability += getCellProbability(ix+1,iy+1)*(factor_x)*(factor_y);
    return probability;
}

void OccupancyGrid::updateVehiclePosition(float x,float y)
{
    //TODO use to grid from grid methods ???
    float fx = vehicle_position.x()+x*cos(-vehicle_orientation)/scalex-y*sin(-vehicle_orientation)/scaley;
    float fy = vehicle_position.y()+x*sin(-vehicle_orientation)/scalex+y*cos(-vehicle_orientation)/scaley;
    updateVehicleCellPosition(fx,fy);
}

void OccupancyGrid::updateVehicleCellPosition(float x,float y)
{
    std::cout << "update " << x << " " << y << std::endl;
    vehicle_position.x() = x;
    vehicle_position.y() = y;
}

void OccupancyGrid::updateVehicleOrientation(float heading)
{
    if(heading < M_PI && heading > M_PI)
        throw std::out_of_range("the orientation must lie inside the interval [-pi..+pi]");
    vehicle_orientation = heading;
}

GridBase::Point2D OccupancyGrid::getVehicleCellPosition() const
{
    return vehicle_position;
}

float OccupancyGrid::getVehicleOrientation() const
{
    return vehicle_orientation;
}

void OccupancyGrid::normalizeEgoGrid(float radius)
{
    ego_radius = radius/scalex;         //TODO what happend if scalex differes from scaley

    // calculate desired vehicle position 
    // the orientation determines the position on the circle
    Point2D center = getCenterPoint();
    float fx = center.x()/scalex - ego_radius * cos(vehicle_orientation); // --> vehicle_orientation + M_PI
    float fy = center.y()/scaley - ego_radius * sin(vehicle_orientation); // --> vehicle_orientation + M_PI
    int delta_x = fx-vehicle_position.x();
    int delta_y = fy-vehicle_position.y();

    //move cell values
    moveCellValues(delta_x,delta_y,l_0);

    //update vehicle position
    updateVehicleCellPosition(vehicle_position.x()+delta_x,vehicle_position.y()+delta_y);
}

void OccupancyGrid::moveCellValues(int delta_x,int delta_y, float empty)
{
    //TODO check if the delta is to big and just clear all fields in this case
    //calculate array views
    typedef ArrayType::index_range range_t;
    ArrayType &array = getGridData();
    range_t new_range_x(std::max(0,delta_x),std::min(getCellSizeX(),delta_x+getCellSizeX()));
    range_t new_range_y(std::max(0,delta_y),std::min(getCellSizeY(),delta_y+getCellSizeY()));
    range_t old_range_x(std::max(0,-delta_x),std::min(getCellSizeX(),-delta_x+getCellSizeX()));
    range_t old_range_y(std::max(0,-delta_y),std::min(getCellSizeY(),-delta_y+getCellSizeY()));
    ArrayType::array_view<2>::type old_view = array[boost::indices[old_range_x][old_range_y]];
    ArrayType::array_view<2>::type new_view = array[boost::indices[new_range_x][new_range_y]];
    int dim_x = new_view.shape()[0]; 
    int dim_y = new_view.shape()[1]; 

    std::cout << "move cell values " << delta_x << " / " << delta_y << " dim: " << dim_x << "/" << dim_y << std::endl;

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
                for(ArrayType::index j = dim_y-1; j >= 0; --j)
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
                for(ArrayType::index j = dim_y-1; j >= 0; --j)
                    new_view[i][j] = old_view[i][j];
        }
    }

    // delete old cells which got not overwritten
    if(delta_x < 0)
    {
        for(ArrayType::index i = 0; i < -delta_x; ++i)
            for(ArrayType::index j =0; j < (int) cellSizeY; ++j)
                array[i][j] = empty;
    }
    else
    {
        for(ArrayType::index i = cellSizeX-1; i >= (int) cellSizeX-delta_x; --i)
            for(ArrayType::index j =0; j < (int) cellSizeY; ++j)
                array[i][j] = empty;
    }
    if(delta_y < 0)
    {
        for(ArrayType::index i = 0; i < (int) cellSizeX; ++i)
            for(ArrayType::index j = 0; j < -delta_y; ++j)
                array[i][j] = empty;
    }
    else
    {
        for(ArrayType::index i = 0; i < (int)cellSizeX; ++i)
            for(ArrayType::index j = cellSizeY-1; j >= (int)cellSizeY-delta_y; --j)
                array[i][j] = empty;
    }
}
