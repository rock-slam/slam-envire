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
    osg::ref_ptr<osg::Vec3Array>  vertice_array = new osg::Vec3Array;
    vertice_array->push_back(osg::Vec3(pos_x1,pos_y1,0));
    vertice_array->push_back(osg::Vec3(pos_x2,pos_y1,0));
    vertice_array->push_back(osg::Vec3(pos_x2,pos_y2,0));
    vertice_array->push_back(osg::Vec3(pos_x1,pos_y2,0));
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
  
void GridVisualizationBase::showGridAsImage(osg::ref_ptr< osg::Geode > geode, GridBase* grid, boost::function<bool (int x, int y, Color &ret)> colorForGridCoordinate) const
{
    std::cout << "showGridAsImage: Update" << std::endl;
    
    //Load the texture image
    osg::ref_ptr<osg::Image> image = new osg::Image();
    
    int image_width = grid->getCellSizeX();
    int image_height = grid->getCellSizeY();
    
    int size = image_width*image_height*3;
    unsigned char* mydata = new unsigned char[size]; 
    Color c;
    
    for(size_t y = 0; y < image_height; y++)
    {
	for(size_t x = 0; x < image_width; x++)
	{
	    unsigned char* pos = mydata + (y * image_width * 3 + x * 3);
	    if(!colorForGridCoordinate(x, y, c))
		throw std::runtime_error("GridVisualizationBase::showGridAsImage: No Color given for cooridnate");
	    *pos = c.r;
	    pos++;
	    *pos = c.g;
	    pos++;
	    *pos = c.b;
	}
    }

    image->setImage(
	    image_width,
	    image_height,
	    1, // datadepth per channel
	    GL_RGB, //GLint internalTextureformat, (GL_LINE_STRIP, 0x0003)
	    GL_RGB, // GLenum pixelFormat, (GL_RGB, 0x1907)
	    GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
	    (unsigned char*)(mydata), // unsigned char* data
	    osg::Image::USE_NEW_DELETE, //osg::Image::NO_DELETE // AllocationMode mode (shallow copy)
	    1);
    
    loadImageAsRectangle(geode,image,grid->getOffsetX(),grid->getOffsetY(),
			image_width*grid->getScaleX() + grid->getOffsetX(),
			image_height*grid->getScaleY() + grid->getOffsetY());
  }   

}
