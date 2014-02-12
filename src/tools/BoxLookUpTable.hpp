#ifndef ENVIRE_BOXLOOKUPTABLE_HPP
#define ENVIRE_BOXLOOKUPTABLE_HPP
#include <base/pose.h>

namespace envire
{

class BoxLookUpTable
{

public:
    BoxLookUpTable();
    
    const double& getDistanceToBox(double x, double y) const;
    
    void recompute(double scale, double width, double height, double maxDistFromBox);
    
    void printDebug();
    
private:
    void computeDistances();
    
    double scale;
    double width;
    double height;
    double maxDistFromBox;
    double *distanceTable;
    
    int sizeX;
    int sizeY;
    double widthHalf;
    double heightHalf;
    double offsetX;
    double offsetY;
    double centerX;
    double centerY;
};

}
#endif // RADIALLOOKUPTABLE_HPP
