#ifndef ENVIRE_BOXLOOKUPTABLE_HPP
#define ENVIRE_BOXLOOKUPTABLE_HPP

namespace envire
{

class BoxLookUpTable
{

public:
    BoxLookUpTable();
    
    const double& getDistanceToBox(double x, double y) const;
    
    void recompute(double scale, double sizeX, double sizeY, double maxDistFromBox);
    
    void printDebug();
    
private:
    void computeDistances();
    
    double scale;
    double sizeX;
    double sizeY;
    double maxDistFromBox;
    double *distanceTable;
    
    int cellSizeX;
    int cellSizeY;
    double sizeXHalf;
    double sizeYHalf;
    double offsetX;
    double offsetY;
    double centerX;
    double centerY;
};

}
#endif // RADIALLOOKUPTABLE_HPP
