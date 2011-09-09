#include "PointcloudVisualization.hpp"
#include <osg/Group>
#include <osg/Geode>
#include <osg/Point>
#include <osg/Geometry>
#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Featurecloud.hpp>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>

PointcloudVisualization::PointcloudVisualization()
    : vertexColor(osg::Vec4(0.1,0.9,0.1,.5)), 
      normalColor(osg::Vec4(0.9,0.1,0.1,.5)), 
      normalScaling(0.2),
      showNormals(false)
{
}

osg::Group* PointcloudVisualization::getNodeForItem(envire::EnvironmentItem* item) const
{
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    
    group->addChild(geode.get());

    // switch off lighting for this node
    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    
    updateNode(item, group);
    
    return group.release();
}

bool PointcloudVisualization::handlesItem(envire::EnvironmentItem* item) const
{
    return dynamic_cast<envire::Pointcloud *>(item) || dynamic_cast<envire::Featurecloud *>(item);
}

void PointcloudVisualization::highlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1,0,0,1));
    osg::Geometry *geom = group->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}

void PointcloudVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(vertexColor);
    osg::Geometry *geom = group->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}

void PointcloudVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Geode> geode = group->getChild(0)->asGeode();
    //remove old drawables
    while(geode->removeDrawables(0));
    
    envire::Pointcloud *pointcloud = dynamic_cast<envire::Pointcloud *>(item);
    assert(pointcloud);

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;

    // TODO this is a big hack!
    // cycle through colors
    const unsigned long col = reinterpret_cast<unsigned long>(item);
    osg::Vec4 pointColor = osg::Vec4( ((col*88734)%256)/255.0, ((col*398482)%256)/255.0, ((col*36784787)%256)/255.0, 1.0 ); 
    color->push_back( pointColor );

    //color->push_back(vertexColor);

    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    
    for(std::vector<Eigen::Vector3d>::const_iterator it = pointcloud->vertices.begin(); it != pointcloud->vertices.end(); it++) {
	vertices->push_back(osg::Vec3(it->x(),it->y(), it->z()));
    }

    
    //attach vertivces to geometry
    geom->setVertexArray(vertices);
    
    osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, vertices->size() );
    geom->addPrimitiveSet(drawArrays.get());
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(10.0);
    point->setDistanceAttenuation( osg::Vec3(0.5, 0.5, 0.5 ) );
    point->setMinSize( 0.2 );
    point->setMaxSize( 5.0 );
    geom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );

    if( pointcloud->hasData(envire::Pointcloud::VERTEX_NORMAL) && showNormals )
    {
	osg::ref_ptr<osg::Geometry> ngeom = new osg::Geometry;
	osg::ref_ptr<osg::Vec4Array> ncolor = new osg::Vec4Array;
	ncolor->push_back(normalColor);
	ngeom->setColorArray(ncolor.get());
	ngeom->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::ref_ptr<osg::Vec3Array> nvertices = new osg::Vec3Array;

	std::vector<Eigen::Vector3d> &normals(pointcloud->getVertexData<Eigen::Vector3d>(envire::Pointcloud::VERTEX_NORMAL));

	for(size_t n=0;n<pointcloud->vertices.size();n++) {
	    Eigen::Vector3d &point( pointcloud->vertices[n] );
	    Eigen::Vector3d normal( normals[n] * normalScaling );
	    nvertices->push_back(osg::Vec3(point.x(),point.y(), point.z()));
	    nvertices->push_back(osg::Vec3(point.x()+normal.x(),point.y()+normal.y(), point.z()+normal.z()));
	}

	//attach vertivces to geometry
	ngeom->setVertexArray(nvertices);

	osg::ref_ptr<osg::DrawArrays> ndrawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, nvertices->size() );
	ngeom->addPrimitiveSet(ndrawArrays.get());

	geode->addDrawable(ngeom.get());    
    }

    // additional visualisation for features
    envire::Featurecloud *featurecloud = dynamic_cast<envire::Featurecloud *>(item);
    if( featurecloud )
    {
	osg::ref_ptr<osg::Geometry> ngeom = new osg::Geometry;
	osg::ref_ptr<osg::Vec4Array> ncolor = new osg::Vec4Array;
	ncolor->push_back(pointColor);
	ngeom->setColorArray(ncolor.get());
	ngeom->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::ref_ptr<osg::Vec3Array> nvertices = new osg::Vec3Array;

	for(size_t n=0;n<featurecloud->keypoints.size();n++) {
	    // asume the origin as the origin of the original acquisition
	    envire::KeyPoint &keypoint( featurecloud->keypoints[n] );
	    Eigen::Vector3d point( pointcloud->vertices[n] );
	    Eigen::Vector3d nview( -pointcloud->vertices[n].normalized() );

	    const size_t circle_segments = 12 + keypoint.size * 12;
	    Eigen::Vector3d s = nview.cross( Eigen::Vector3d::UnitX() ).normalized() * keypoint.size;
	    Eigen::AngleAxisd rot( M_PI*2.0/circle_segments, nview.normalized() );
	    for( size_t i=0; i<circle_segments; i++ )
	    {
		nvertices->push_back(osg::Vec3(point.x()+s.x(),point.y()+s.y(),point.z()+s.z()));
		s = rot * s;
		nvertices->push_back(osg::Vec3(point.x()+s.x(),point.y()+s.y(),point.z()+s.z()));
	    }

	    Eigen::Vector3d normal( nview * normalScaling );
	    nvertices->push_back(osg::Vec3(point.x(),point.y(), point.z()));
	    nvertices->push_back(osg::Vec3(point.x()+normal.x(),point.y()+normal.y(), point.z()+normal.z()));
	}

	ngeom->setVertexArray(nvertices);

	osg::ref_ptr<osg::DrawArrays> ndrawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, nvertices->size() );
	ngeom->addPrimitiveSet(ndrawArrays.get());
	ngeom->getOrCreateStateSet()->setAttribute( new osg::LineWidth( 2.0 ), osg::StateAttribute::ON );

	geode->addDrawable(ngeom.get());    
    }

    geode->addDrawable(geom.get());    
}
