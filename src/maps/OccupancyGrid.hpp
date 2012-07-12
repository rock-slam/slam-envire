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

    Point2D vehicle_position;     // position [in cells] of the vehicle inside the grid
    float vehicle_orientation;    // orientation/heading [in rad] of the vehicle in
                                  // respect to the grid
    float ego_radius;             // current radius of the ego centered occupancy grid
                                  // this is only used for display
    float l_0;                    // Initial probability in odd-log form

  public:
    OccupancyGrid();
    OccupancyGrid(size_t width, size_t height, double scalex, double scaley);
    ~OccupancyGrid();

    virtual const std::vector<std::string>& getBands() const {return bands;};
    void serialize(Serialization& so);
    void unserialize(Serialization& so);
    virtual void clear(float initial_prob = 0.5);
    virtual void clearCellValues(float initial_prob = 0.5);

    //updates the probability of the given cell
    virtual void updateCellProbability(int x,int y,float probability);

    //returns the probability of the given cell [cells]
    float getCellProbability(int x, int y) const;

    // updates the probability relative to the current vehicle position
    // x is always in the direction of the vehicle [m]
    // TODO add way to set footprint 
    void updateProbability(float x,float y,float propability);

    // gets the probability relative to the current vehicle position
    // x is always in the direction of the vehicle [m]
    float getProbability(float x, float y) const;

    // updates the vehicle position relative to the current vehicle
    // position and orientation [m]
    // x is always in the direction of the vehicle
    void updateVehiclePosition(float x,float y);

    // updates the vehicle position relative to the grid frame [cells]
    void updateVehicleCellPosition(float x,float y);

    // updates the orientation of the vehicle relative to the grid frame [rad]
    void updateVehicleOrientation(float heading);

    // getter for the vehicle position relative to the grid frame [cells]
    Point2D getVehicleCellPosition() const;

    // getter for the orientation of the vehicle relative to the grid frame [rad]
    float getVehicleOrientation() const;
    
    // moves the content of the grid according to the current vehicle position
    // so that the vehicle is located on a virtual circle (radius [m]) around
    // the center of the grid and is pointing to the center
    //
    // This operation can be used after updateVehicleOrientation or updateVehiclePosition
    // was called to normalize the grid to an ego centered occupancy grid
    //
    // Note: scalex and scaley must have the same value otherwise an domain error is thrown
    void normalizeVehiclePosition(float radius = 0.0F);

    // convertes x,y expressed relative to the vehicle to grid coordinates
    Point2D fromVehicle(double x, double y)const;

    float getEgoRadius();

  private:
    //moves each cell value to the new cell given by delta_x and delta_y
    //all cells which were not overwritten are set to the value of empty 
    void moveCellValues(int delta_x,int delta_y,float empty = 0);
};

};

#endif
