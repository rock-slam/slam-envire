#include "LaserScanVisualization.hpp"

#include <envire/maps/LaserScan.hpp>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>

using namespace envire;

bool LaserScanVisualization::handlesItem(envire::EnvironmentItem *item) const
{
    if(dynamic_cast<envire::LaserScan *>(item))
	return true;
    
    return false;
}    


osg::Group* LaserScanVisualization::getNodeForItem ( envire::EnvironmentItem* item ) const
{
    envire::LaserScan *ls = dynamic_cast<envire::LaserScan *>(item);
    assert(ls);
    
    osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform = new osg::PositionAttitudeTransform();
    
    for(std::vector<envire::LaserScan::scanline_t>::const_iterator it = ls->lines.begin(); it != ls->lines.end(); it++) 
    {
	osg::Node *lineNode = getNodeForLine(*it, ls->delta_psi, ls->origin_psi);
	osg::ref_ptr<osg::PositionAttitudeTransform> lineTransform = new osg::PositionAttitudeTransform();
	osg::ref_ptr<osg::PositionAttitudeTransform> offsetTransform = new osg::PositionAttitudeTransform();
	offsetTransform->setPosition(osg::Vec3f(ls->center_offset.x(), ls->center_offset.y(), ls->center_offset.z()));
	offsetTransform->addChild(lineNode);
	lineTransform->setAttitude(osg::Quat(ls->origin_phi + it->delta_phi, osg::Vec3f(1,0,0)));
	lineTransform->addChild(offsetTransform);
	mainTransform->addChild(lineTransform);
	//std::cout << "LaserScanVisualization: Adding Lines" << std::endl; 
    }
    
    return mainTransform.release();
}


osg::Node* LaserScanVisualization::getNodeForLine ( const envire::LaserScan::scanline_t& line, double stepSize, double startingAngle ) const
{
    osg::ref_ptr<osg::Vec4Array> color2 = new osg::Vec4Array;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> pointsOSG = new osg::Vec3Array;

    color2->clear();

    bool has_rem = line.ranges.size() == line.remissions.size();
    double remissionScaleFactor = 5000;

    for(size_t i=0;i<line.ranges.size();i++) 
    {
	osg::Vec3 point;
	float range = line.ranges[i] / 1000.0;
	point.x() = -sin(stepSize * i + startingAngle) * range;
	point.y() = cos(stepSize * i + startingAngle) * range;
	pointsOSG->push_back(point);

	if( has_rem )
	{
	    float cval = std::min( 1.0, std::max( 0.0, line.remissions[i] / remissionScaleFactor ) );
	    color2->push_back( osg::Vec4( cval, cval, cval, 1.0 ) );
	}
    }
    

    geom->setVertexArray(pointsOSG.get());
    osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, pointsOSG->size() );
    // Draw a four-vertex quad from the stored data.
    geom->addPrimitiveSet(drawArrays.get());
    geom->getOrCreateStateSet()->setAttribute( new osg::Point( 2.0f ), osg::StateAttribute::ON );

    if( has_rem )
    {
	geom->setColorArray( color2 );
	geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }
    // Add the Geometry (Drawable) to a Geode and
    //   return the Geode.
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom.get() );
    
    //std::cout << "LaserScanVisualization: Adding Points" << std::endl; 
    
    return geode.release();

    
}

void LaserScanVisualization::highlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{    
    osg::Transform *mainTransform = group->asTransform();
    for(unsigned int i= 0; i < mainTransform->getNumChildren(); i++) {
	osg::Geode *geode = mainTransform->getChild(i)->asTransform()->getChild(0)->asTransform()->getChild(0)->asGeode();
	osg::Geometry *drawable = geode->getDrawable(0)->asGeometry();
	osg::Vec4Array *color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1,0,0,1));
	drawable->setColorArray(color);
	drawable->setColorBinding( osg::Geometry::BIND_OVERALL );
    }
}

void LaserScanVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::Transform *mainTransform = group->asTransform();
    for(unsigned int i= 0; i < mainTransform->getNumChildren(); i++) {
	osg::Geode *geode = mainTransform->getChild(i)->asTransform()->getChild(0)->asTransform()->getChild(0)->asGeode();
	osg::Geometry *drawable = geode->getDrawable(0)->asGeometry();
	osg::Vec4Array *color = new osg::Vec4Array;
	//TODO remission values go here
	color->push_back(osg::Vec4(1,1,1,1));
	drawable->setColorArray(color);
	drawable->setColorBinding( osg::Geometry::BIND_OVERALL );
    }
}

void LaserScanVisualization::updateNode ( envire::EnvironmentItem* item, osg::Group* node ) const
{

}

