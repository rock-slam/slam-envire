#include "GridVisualizationBase.hpp"

#include <osg/Point>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <envire/maps/Grids.hpp>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/ReadFile>
#include <iostream>
#include <osg/ShapeDrawable>

namespace envire 
{
  osg::Group* GridVisualizationBase::getNodeForItem ( envire::EnvironmentItem* item ) const
  {
    std::cout << "ElevationGridVisualization: getNodeForItem" << std::endl; 
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    group->addChild(geode.get());
    updateNode ( item, group);
    return group.release();
  }
  
  
  void GridVisualizationBase::loadImageAsRectangle(osg::ref_ptr<osg::Geode> geode,
						   osg::ref_ptr<osg::Image> image,
						   float pos_x1,float pos_y1,float pos_x2,float pos_y2)const
  {
    if(! (image.valid()))
    {
      std::cout << "Error Loading Texture: Image is not valid " << std::endl;
      exit(0);
    }
    // Create a Geometry object.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    
    //full color display
    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    geom->setColorArray( c.get() );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    c->push_back( osg::Vec4( 1.0f, 1.0f, 1.0f, 1.0f ) );
    
    //set texture coordinates
    osg::ref_ptr<osg::Vec2dArray>  texture_coordinates = new osg::Vec2dArray;
    texture_coordinates->push_back(osg::Vec2d(0,0));
    texture_coordinates->push_back(osg::Vec2d(1,0));
    texture_coordinates->push_back(osg::Vec2d(1,1));
    texture_coordinates->push_back(osg::Vec2d(0,1));
    geom->setTexCoordArray(0, texture_coordinates.get());
    
    // Specify the vertices:
    osg::ref_ptr<osg::Vec3dArray>  vertice_array = new osg::Vec3dArray;
    vertice_array->push_back(osg::Vec3d(pos_x1,pos_y1,0));
    vertice_array->push_back(osg::Vec3d(pos_x2,pos_y1,0));
    vertice_array->push_back(osg::Vec3d(pos_x2,pos_y2,0));
    vertice_array->push_back(osg::Vec3d(pos_x1,pos_y2,0));
    // Associate this set of vertices with the geometry associated with the geode 
    geom->setVertexArray(vertice_array);
   
     // Create a QUAD primitive by specifying the 
     // vertices from our vertex list that make up this QUAD:

    // Add primitive to the geometry
    geom->addPrimitiveSet(new osg::DrawArrays( osg::PrimitiveSet::POLYGON, 0, vertice_array->size() ) );
    
    // Attach the image in a Texture2D object
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    texture->setImage( image.get() );

    osg::StateSet *state = geom->getOrCreateStateSet();
    
    //apply texture to geom
    state->setTextureAttributeAndModes(0, texture.get());
    
    //remove old drawables
    while(geode->removeDrawables(0));
    //add a mew one
    geode->addDrawable(geom.get());  
  }
}
