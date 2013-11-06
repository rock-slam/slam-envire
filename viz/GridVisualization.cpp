#include "GridVisualization.hpp"

#include <osg/Geode>
#include <envire/maps/Grids.hpp>
#include <osg/Texture2D>
#include <osg/Image>
#include <iostream>

namespace envire 
{
    GridVisualization::GridVisualization() : GridVisualizationBase(), showEmptyCells(false), cycleGridColor(false)
    {
        // set available colors
        for(unsigned i = 0; i < 7; i++)
            for(unsigned j = 0; j < 3; j++)
                colors[i][j] = 0.0;
        // white
        colors[0][0] = 1.0;
        colors[0][1] = 1.0;
        colors[0][2] = 1.0;
        // rgb
        colors[1][0] = 1.0;
        colors[2][1] = 1.0;
        colors[3][2] = 1.0;
        // cym
        colors[4][1] = 1.0;
        colors[4][2] = 1.0;
        colors[5][0] = 1.0;
        colors[5][1] = 1.0;
        colors[6][0] = 1.0;
        colors[6][2] = 1.0;
    }

  bool GridVisualization::handlesItem(envire::EnvironmentItem *item) const
  {
      if( dynamic_cast<envire::Grid<float> *>(item) )
	  return true;
      if( dynamic_cast<envire::Grid<double> *>(item) )
	  return true;
      if( dynamic_cast<envire::Grid<unsigned char> *>(item) )
	  return true;
      
      return false;
  }    

  template <class T>
  void copyGridData( unsigned char *pos, unsigned char *end, T *data, float scale, const float *color, bool empty_grid_invisible )
  {
      while( pos != end )
      {
          unsigned char v = *data++ * scale;
          *(pos++) = color[0] * v; 
          *(pos++) = color[1] * v; 
          *(pos++) = color[2] * v; 
          *(pos++) = empty_grid_invisible ? (*data == 0 ? 0 : 128 + 0.5*v) : 255;
      }
  }
 
  void GridVisualization::updateNode( envire::EnvironmentItem* item, osg::Group* node ) const
  {
    
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
    envire::GridBase *grid_base = dynamic_cast<envire::GridBase *>(item);
    long item_id = item->getUniqueIdNumericalSuffix();
    const float *color = &colors[White][0];
    if(cycleGridColor)
        color = &colors[(item_id % 6)+1][0];

    //Load the texture image
    osg::ref_ptr<osg::Image> image = new osg::Image();
    
    int image_width = grid_base->getWidth();
    int image_height = grid_base->getHeight();
    
    int size = image_width*image_height*4;
    unsigned char* mydata = new unsigned char[size]; 
    unsigned char* pos = mydata;
    unsigned char* end_pos = &mydata[size];

    {
        envire::Grid<float> *grid = dynamic_cast<envire::Grid<float>*>(item);
        if( grid )
        {
            float min, max;
            grid->getMinMaxValues( grid->getBands().front(), min, max );
            copyGridData( pos, end_pos, grid->getGridData().data(), 255.0 / max, color, !showEmptyCells );
        }
    }

    {
        envire::Grid<double> *grid = dynamic_cast<envire::Grid<double>*>(item);
        if( grid )
        {
            double min, max;
            grid->getMinMaxValues( grid->getBands().front(), min, max );
            copyGridData( pos, end_pos, grid->getGridData().data(), 255.0 / max, color, !showEmptyCells );
        }
    }

    {
        envire::Grid<unsigned char> *grid = dynamic_cast<envire::Grid<unsigned char>*>(item);
        if( grid )
            copyGridData( pos, end_pos, grid->getGridData().data(), 1, color, !showEmptyCells );
    }
    
    image->setImage(
	    image_width,
	    image_height,
	    1, // datadepth per channel
	    GL_RGBA, //GLint internalTextureformat, (GL_LINE_STRIP, 0x0003)
	    GL_RGBA, // GLenum pixelFormat, (GL_RGB, 0x1907)
	    GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
	    (unsigned char*)(mydata), // unsigned char* data
	    osg::Image::USE_NEW_DELETE, //osg::Image::NO_DELETE // AllocationMode mode (shallow copy)
	    1);
    
    loadImageAsRectangle(geode,image,grid_base->getOffsetX(),grid_base->getOffsetY(),
			image_width*grid_base->getScaleX() + grid_base->getOffsetX(),
			image_height*grid_base->getScaleY() + grid_base->getOffsetY());
  }   
}

