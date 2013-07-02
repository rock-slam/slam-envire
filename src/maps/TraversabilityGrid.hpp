#ifndef ENVIRE_TRAVERSABILITYGRID_H
#define ENVIRE_TRAVERSABILITYGRID_H

#include <envire/maps/Grid.hpp>
#include <base/samples/distance_image.h>
#include <boost/function.hpp>

namespace envire
{  
class TraversabilityGrid : public Grid<uint8_t>
{
    ENVIRONMENT_ITEM( TraversabilityGrid )
public:
    static const std::string TRAVERSABILITY;
private:
    const static std::vector<std::string> &bands;
public:
    TraversabilityGrid() : Grid<uint8_t>() {};
    TraversabilityGrid(size_t width, size_t height, 
                        double scalex, double scaley, 
                        double offsetx = 0.0, double offsety = 0.0,
                        std::string const& id = Environment::ITEM_NOT_ATTACHED):Grid<uint8_t>::Grid(width,height,scalex,scaley,offsetx, offsety, id){};
    ~TraversabilityGrid(){};
    
    void computeStatistic(base::Pose2D pose, double width, double height, double borderWidth);
    
    virtual const std::vector<std::string>& getBands() const {return bands;};
};

}
#endif // TRAVERSABILITYGRID_H
