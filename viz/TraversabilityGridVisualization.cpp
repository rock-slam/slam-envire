#include "TraversabilityGridVisualization.hpp"
#include "envire/maps/Grids.hpp"
#include <osg/Image>
#include <osg/Geometry>
#include <osg/Geode>
#include <boost/bind.hpp>

TraversabilityGridVisualization::TraversabilityGridVisualization()
{

}

bool TraversabilityGridVisualization::handlesItem(envire::EnvironmentItem* item) const
{
    if(dynamic_cast<envire::Grid<uint8_t> *>(item))
    {
	return true; 
    }
    return false;
}

osg::Group* TraversabilityGridVisualization::getNodeForItem ( envire::EnvironmentItem* item ) const
  {
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    group->addChild(geode.get());
    updateNode ( item, group);
    return group.release();
  }

bool colorForCoordinate(int x, int y, envire::GridVisualizationBase::Color &ret, const envire::TraversabilityGrid::ArrayType &trGridData)
{
    assert(x >= 0);
    assert(x < 1600);
    assert(y >= 0);
    assert(y < 1600);
    
    //sec color arcording to value of the field
    uint8_t value = trGridData[y][x];
    switch(value)
    {
	case 0:
	    //unkown
	    ret.r = 0;
	    ret.g = 0;
	    ret.b = 255;
	    break;
	case 1:
	    //obstacle
	    ret.r = 255;
	    ret.g = 0;
	    ret.b = 0;
	    break;
	default:
	    //this is hard coded for now we assume there are vaules from 0 to 12
	    assert(value < 13);
	    //give a red to black color gradient
	    ret.r = 255 - 240 * (value / 12.0 + 1);
	    ret.g = 255 * (value / 12.0);
	    ret.b = 0;
	    break;
    }

    return true;
}
  
void TraversabilityGridVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* node) const
{
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
    envire::TraversabilityGrid *trGrid = dynamic_cast<envire::TraversabilityGrid *>(item);
    
    assert(trGrid);
    
    const std::string bandName(envire::TraversabilityGrid::TRAVERSABILITY);

    const envire::TraversabilityGrid::ArrayType &trGridData = trGrid->getGridData(bandName);

    showGridAsImage(geode, trGrid, boost::bind(colorForCoordinate, _1, _2, _3, trGridData));
}
