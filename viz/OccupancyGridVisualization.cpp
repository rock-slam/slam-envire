#include "OccupancyGridVisualization.hpp"

#include <osg/Point>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <envire/maps/Grids.hpp>
#include <envire/maps/OccupancyGrid.hpp>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/ReadFile>
#include <iostream>
#include <osg/ShapeDrawable>

namespace envire 
{
  bool OccupancyGridVisualization::handlesItem(envire::EnvironmentItem *item) const
  {
      if(dynamic_cast<envire::OccupancyGrid *>(item))
	  return true;
      return false;
  }
 
  void OccupancyGridVisualization::updateNode ( envire::EnvironmentItem* item, osg::Group* node ) const
  {
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
    envire::OccupancyGrid *grid = dynamic_cast<envire::OccupancyGrid *>(item);
    assert(grid);

    //Load the texture image
    osg::ref_ptr<osg::Image> image = new osg::Image();
    
    int image_width = grid->getCellSizeX();
    int image_height = grid->getCellSizeY();
    
    int size = image_width*image_height*3;
    unsigned char* mydata = new unsigned char[size]; 
    float val;
    for(size_t y = 0; y < (unsigned int) image_height; y++)
    {
	for(size_t x = 0; x < (unsigned int) image_width; x++)
	{
	    unsigned char* pos = mydata + (y * image_width * 3 + x * 3);
            grid->getProbability(x,y,val);
            val = 255*val;
	    *pos = val;
	    pos++;
	    *pos = val;
	    pos++;
	    *pos = val;
            std::cout << x << "/" << y << " " << val*255 << " ";
	}
    }
    std::cout << std::endl;

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
