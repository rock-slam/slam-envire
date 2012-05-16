#ifndef POINTCLOUDMANIPULATOR_H
#define POINTCLOUDMANIPULATOR_H

#include <envire/maps/Pointcloud.hpp>
#include <osg/Group>

namespace envire {
    
class TabBoxManipulator;
class CutPointcloud;

class PointcloudManipulator
{
public:
    PointcloudManipulator( envire::EnvironmentItem* item, osg::Group* parentNode );
    ~PointcloudManipulator();
    void addManipulationBox();
    void removeManipulationBox();
    
protected:
    Pointcloud* pointcloud;
    osg::Group *parentNode;
    
    std::list< TabBoxManipulator* > manipulators;
    std::list< CutPointcloud* > operators;
};

}
#endif // POINTCLOUDMANIPULATOR_H
