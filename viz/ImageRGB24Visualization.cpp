#include "ImageRGB24Visualization.hpp"

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
    
  bool ImageRGB24Visualization::handlesItem(envire::EnvironmentItem *item) const
  {
      if(dynamic_cast<envire::ImageRGB24 *>(item))
	  return true;
      
      return false;
  }    
 
  void ImageRGB24Visualization::updateNode ( envire::EnvironmentItem* item, osg::Group* node ) const
  {
    std::cout << "ImageRGB24Visualization: Update" << std::endl;
    
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
    envire::ImageRGB24 *image_rgb24 = dynamic_cast<envire::ImageRGB24 *>(item);
    assert(image_rgb24->getGridDepth() == 1);
    assert(image_rgb24);
    
    //Load the texture image
    osg::ref_ptr<osg::Image> image = new osg::Image();
    envire::ImageRGB24::ArrayType &data_r = image_rgb24->getGridData(envire::ImageRGB24::R);
    envire::ImageRGB24::ArrayType &data_g = image_rgb24->getGridData(envire::ImageRGB24::G);
    envire::ImageRGB24::ArrayType &data_b = image_rgb24->getGridData(envire::ImageRGB24::B);
    
    int image_width = image_rgb24->getWidth();
    int image_height = image_rgb24->getHeight();
    
    int size = image_width*image_height*3;
    unsigned char* mydata = new unsigned char[size]; 
    unsigned char* pos = mydata;
    unsigned char* pos_r = data_r.data();
    unsigned char* pos_g = data_g.data();
    unsigned char* pos_b = data_b.data();
    unsigned char* end_pos = &mydata[size];
    //fast data copy
    while(pos != end_pos)
    {
	*(pos++) = *(pos_r++);
	*(pos++) = *(pos_g++);
	*(pos++) = *(pos_b++);
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
    
    loadImageAsRectangle(geode,image,image_rgb24->getOffsetX(),image_rgb24->getOffsetY(),
			image_width*image_rgb24->getScaleX() + image_rgb24->getOffsetX(),
			image_height*image_rgb24->getScaleY() + image_rgb24->getOffsetY());
  }   
}
