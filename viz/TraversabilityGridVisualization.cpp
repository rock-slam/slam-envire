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
    std::cout << "TRViz handles " << item->getClassName();
    if(dynamic_cast<envire::Grid<uint8_t> *>(item))
    {
	std::cout << " True " << std::endl;
	return true; 
    }
    std::cout << " False " << std::endl;
    return false;
}

osg::Group* TraversabilityGridVisualization::getNodeForItem ( envire::EnvironmentItem* item ) const
  {
    std::cout << "ElevationGridVisualization: getNodeForItem" << std::endl; 
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
    uint8_t value = trGridData[x][y];
    switch(value)
    {
	case 0:
	    //unkown
	    ret.r = 0;
	    ret.g = 0;
	    ret.b = 255;
	    break;
	case 1:
	    //traversable
	    ret.r = 0;
	    ret.g = 255;
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
    envire::Grid<uint8_t> *trGrid = dynamic_cast<envire::Grid<uint8_t> *>(item);
    
    assert(trGrid);
    
    const std::string bandName("grid_data");

    const envire::TraversabilityGrid::ArrayType &trGridData = trGrid->getGridData(bandName);

    showGridAsImage(geode, trGrid, boost::bind(colorForCoordinate, _1, _2, _3, trGridData));
}
  
void TraversabilityGridVisualization::updateNode2(envire::EnvironmentItem* item, osg::Group* node) const
{
    std::cout << "TraversabilityGridVisualization: Update" << std::endl;
    
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
//     envire::TraversabilityGrid *trGrid = dynamic_cast<envire::TraversabilityGrid *>(item);
    envire::Grid<uint8_t> *trGrid = dynamic_cast<envire::Grid<uint8_t> *>(item);
    assert(trGrid->getGridDepth() == 1);
    assert(trGrid);

    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom;

    // Create an array of four vertices.
    osg::ref_ptr<osg::Vec3Array> v;

    //sorrage for color per vertex
    osg::ref_ptr<osg::Vec4Array> colorArray;
    osg::ref_ptr<osg::Vec3Array> normals;

    if(!geode->getNumDrawables())
    {
	std::cout << "creating array" << std::endl;
	geom = new osg::Geometry();
	// Create an array of four vertices.
	v = new osg::Vec3Array;
	geom->setVertexArray( v.get() );

	//sorrage for color per vertex
	colorArray = new osg::Vec4Array;
	normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0,0,1));
    } else {
	geom = geode->getDrawable(0)->asGeometry();
	v = dynamic_cast<osg::Vec3Array *>(geom->getVertexArray());
	colorArray = dynamic_cast<osg::Vec4Array *>(geom->getColorArray());
	normals = dynamic_cast<osg::Vec3Array *>(geom->getNormalArray());
    }
    
    assert(geom.valid());
    assert(v.valid());
    assert(colorArray.valid());
    assert(normals.valid());

    const int size = trGrid->getWidth()*trGrid->getHeight();

    bool reAddDrawable = v->size() != size * 4;
    std::cout<< "readding drawable : " << reAddDrawable << std::endl;
    
    v->resize(size * 4);
    colorArray->resize(size * 4);
    
    const double xScale = trGrid->getScaleX();
    const double yScale = trGrid->getScaleY();
    
    const size_t height = trGrid->getHeight();
    const size_t width = trGrid->getWidth();
    
    const std::string bandName("grid_data");

    std::cout<< "Width : " << trGrid->getWidth() << " height " << trGrid->getHeight() << " xScale " << xScale << " yScale " << yScale << std::endl;
    
    const envire::TraversabilityGrid::ArrayType &trGridData = trGrid->getGridData(bandName);
    
    for(size_t y = 0; y < height; y++)
    {
	for(size_t x = 0; x < width; x++)
	{
	    const int curCell = y * height + x;
	    osg::Vec4 color(1,1,1,1);
	 
	    //sec color arcording to value of the field
	    uint8_t value = trGridData[x][y];
	    switch(value)
	    {
		case 0:
		    //unkown
		    color = osg::Vec4(0,1,1,1);
		    break;
		case 1:
		    //traversable
		    color = osg::Vec4(0,1,0,1);
		    break;
		default:
		    //this is hard coded for now we assume there are vaules from 0 to 12
		    assert(value < 13);
		    //give a red to black color gradient
		    color = osg::Vec4(1.0 - 0.9 * (value / 12.0), 0, 0, 1);
		    break;
	    }
	    
	    
	    //up left
	    (*v)[curCell] = osg::Vec3(x * xScale, y* yScale, 0);
	    (*colorArray)[curCell] = (color);
	    //down left
	    (*v)[curCell] = osg::Vec3(x * xScale, (y+1)* yScale, 0);
	    (*colorArray)[curCell] = (color);
	    //down right
	    (*v)[curCell] = osg::Vec3((x+1) * xScale, (y+1)* yScale, 0);
	    (*colorArray)[curCell] = (color);
	    //up right
	    (*v)[curCell] = osg::Vec3((x+1) * xScale, (y)* yScale, 0);
	    (*colorArray)[curCell] = (color);
	}	
    }
    
    std::cout << "copy done" << std::endl;

    geom->setColorArray( colorArray.get() );
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    if(reAddDrawable)    
    {
	if(geom->getNumPrimitiveSets())
	    geom->removePrimitiveSet(0);
	
	// Draw a four-vertex quad from the stored data.
	geom->addPrimitiveSet(
		new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, v->size() ) );
    }
    
    geom->setNormalArray(normals);
    geom->setNormalBinding( osg::Geometry::BIND_OVERALL );
    
    geode->addDrawable(geom);
}
