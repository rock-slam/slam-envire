#include "MapSegmentVisualization.hpp"
#include <envire/maps/MapSegment.hpp>
#include <osg/Geode>
#include <vizkit3d/Uncertainty.hpp>

using namespace envire;

MapSegmentVisualization::MapSegmentVisualization()
{
}

bool MapSegmentVisualization::handlesItem(envire::EnvironmentItem* item) const
{
    return dynamic_cast<envire::MapSegment *>(item);
}

osg::Group* MapSegmentVisualization::getNodeForItem(envire::EnvironmentItem* item) const
{
    osg::ref_ptr<osg::Group> group = new osg::Group();
    group->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    return group.release();
}

void MapSegmentVisualization::highlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    // TODO
}

void MapSegmentVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    // TODO
}

void MapSegmentVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    // remove all children first
    group->removeChildren( 0, group->getNumChildren() );

    // add a geode
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    group->addChild( geode );

    envire::MapSegment *mapSegment = dynamic_cast<envire::MapSegment *>( item );
    assert( mapSegment );

    // add trajectories
    for( size_t i=0; i<mapSegment->trajectories.size(); i++ )
    {
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1,1,1,1));
	geom->setColorArray(color.get());
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

	for(std::vector<base::Vector3d>::const_iterator it = mapSegment->trajectories[i].begin(); it != mapSegment->trajectories[i].end(); it++) 
	    vertices->push_back(osg::Vec3(it->x(),it->y(), it->z()));

	geom->setVertexArray(vertices);
	osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_STRIP, 0, vertices->size() );
	geom->addPrimitiveSet(drawArrays.get());

	geode->addDrawable( geom.get() );
    }

    // add uncertainties
    for( size_t i=0; i<mapSegment->gmm.params.size(); i++ )
    {
	vizkit3d::Uncertainty *ellipse = new vizkit3d::Uncertainty();
	ellipse->setMean( Eigen::Vector3d(mapSegment->gmm.params[i].dist.mean.tail<3>()) );
	ellipse->setCovariance( Eigen::Matrix3d( mapSegment->gmm.params[i].dist.cov.bottomRightCorner<3,3>()) );

	group->addChild( ellipse );
    }
}
