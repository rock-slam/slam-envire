#include "TriMeshVisualization.hpp"
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <envire/Core.hpp>
#include <envire/maps/TriMesh.hpp>
#include <osg/Drawable>
#include <osg/ShapeDrawable>


osg::Group* TriMeshVisualization::getNodeForItem(envire::EnvironmentItem* item) const
{
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    
    group->addChild(geode.get());
    
    updateNode(item, group);
    
    return group.release();
}

bool TriMeshVisualization::handlesItem(envire::EnvironmentItem* item) const
{
    return dynamic_cast<envire::TriMesh *>(item);
}

void TriMeshVisualization::highlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1,0,0,1));
    osg::Geometry *geom = group->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}

void TriMeshVisualization::unHighlightNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1,1,1,1));
    osg::Geometry *geom = group->getChild(0)->asGeode()->getDrawable(0)->asGeometry();
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}

void TriMeshVisualization::updateNode(envire::EnvironmentItem* item, osg::Group* group) const
{
    std::cout << "TrimeshViz: UpdateNode called" <<std::endl;
    osg::ref_ptr<osg::Geode> geode = group->getChild(0)->asGeode();
    
    envire::TriMesh *triMesh = dynamic_cast<envire::TriMesh *>(item);
    assert(triMesh);

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1,1,1,1));
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

    
    for(std::vector<Eigen::Vector3d>::const_iterator it = triMesh->vertices.begin(); it != triMesh->vertices.end(); it++) {
	vertices->push_back(osg::Vec3(it->x(),it->y(), it->z()));
    }
    
    //attach vertivces to geometry
    geom->setVertexArray(vertices);
    
    if( triMesh->hasData( envire::Pointcloud::VERTEX_NORMAL ) )
    {
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	std::vector<Eigen::Vector3d> &normals_(triMesh->getVertexData<Eigen::Vector3d>( envire::Pointcloud::VERTEX_NORMAL ) );

	for(std::vector<Eigen::Vector3d>::const_iterator it = normals_.begin(); it != normals_.end(); it++) {
	    normals->push_back(osg::Vec3(it->x(),it->y(), it->z()));
	}
	geom->setNormalArray(normals);
	geom->setNormalIndices( geom->getVertexIndices() );
	geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    }
    
    osg::ref_ptr<osg::DrawElementsUInt> drawable = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    for(std::vector<envire::TriMesh::triangle_t>::const_iterator it = triMesh->faces.begin(); it != triMesh->faces.end(); it++) {
	drawable->push_back(it->get<0>());
	drawable->push_back(it->get<1>());
	drawable->push_back(it->get<2>());
	//std::cout << "TrimeMeshViz: Adding Triangle " << it->get<0>() << " " << it->get<1>() << " " << it->get<2>() << std::endl;
    }
    geom->addPrimitiveSet(drawable.get());    

    //remove old drawables
    while(geode->removeDrawables(0));
    
    //add a mew one
    geode->addDrawable(geom.get());    
}
