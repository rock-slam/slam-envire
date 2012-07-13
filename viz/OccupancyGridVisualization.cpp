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
#include <osg/LineWidth>

namespace envire 
{
  bool OccupancyGridVisualization::handlesItem(envire::EnvironmentItem *item) const
  {
      if(dynamic_cast<envire::OccupancyGrid *>(item))
	  return true;
      return false;
  }


  void OccupancyGridVisualization::setColor(const osg::Vec4d& color, osg::Geode* geode)
  {
//      osg::Material *material = new osg::Material();
//      material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
//      material->setSpecular(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 1.0));
//      material->setAmbient(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
//      material->setEmission(osg::Material::FRONT, color);
//      material->setShininess(osg::Material::FRONT, 10.0);
//      geode->getOrCreateStateSet()->setAttribute(material);    
  }

  void OccupancyGridVisualization::updateNode ( envire::EnvironmentItem* item, osg::Group* node ) const
  {
      osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
      envire::OccupancyGrid *grid = dynamic_cast<envire::OccupancyGrid *>(item);
      assert(grid);

      //remove old drawables
      while(geode->removeDrawables(0));

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
              val = 255*grid->getCellProbability(x,y);
              *pos = val;
              pos++;
              *pos = val;
              pos++;
              *pos = val;
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

      // switch off lighting for this node
      osg::StateSet* stategeode = geode->getOrCreateStateSet();
      stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

      // draw circle 
      osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
      osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
      color->push_back(osg::Vec4(0,1,1,1));
      geom->setColorArray(color.get());
      geom->setColorBinding( osg::Geometry::BIND_OVERALL );
      osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

      static const double delta = 2.0 * M_PI/1000;
      float radius = grid->getEgoRadius();
      GridBase::Point2D center = grid->getCenterPoint();
      for(double theta = 0; theta<=2.0* M_PI;theta+=delta)
          vertices->push_back(osg::Vec3(radius*cos(theta)+center.x(),radius*sin(theta)+center.y(),0.1));
      geom->setVertexArray(vertices);
      osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_LOOP, 0, vertices->size() );
      geom->addPrimitiveSet(drawArrays.get());
      geom->getOrCreateStateSet()->setAttribute( new osg::LineWidth( 2.0 ), osg::StateAttribute::ON );
      geode->addDrawable(geom.get());    

      // draw vehicle 
      osg::ref_ptr<osg::Geode> c2g = new osg::Geode();
      GridBase::Point2D position = grid->getVehicleCellPosition();
      osg::ref_ptr<osg::Cone> c2 = new osg::Cone(osg::Vec3f(position.x()*grid->getScaleX(),position.y()*grid->getScaleY(),0.0), 0.30, 1.0);
      c2->setRotation(osg::Quat(0.5*M_PI, osg::Vec3d(0,1,0))*
                      osg::Quat(grid->getVehicleOrientation(), osg::Vec3d(0,0,1)));
      osg::ref_ptr<osg::ShapeDrawable> c2d = new osg::ShapeDrawable(c2);
      geode->addDrawable(c2d.get());
  }
}
