#include "ElevationGridVisualization.hpp"

#include <osg/Point>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Material>
#include <osg/PositionAttitudeTransform>

#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/ReadFile>
#include <iostream>
#include <osg/ShapeDrawable>

static const double SCALING_MIN_VALUE = 0;	//in meters
static const double SCALING_MAX_VALUE = 5;	//in meters

namespace vizkit 
{
    
  bool ElevationGridVisualization::handlesItem(envire::EnvironmentItem *item) const
  {
      if(dynamic_cast<envire::ElevationGrid *>(item))
	  return true;  
      return false;
  }    

  void ElevationGridVisualization::updateNode ( envire::EnvironmentItem* item, osg::Group* node ) const
  {
    std::cout << "ElevationGridVisualization: Update" << std::endl; 
     
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
    envire::ElevationGrid *eg = dynamic_cast<envire::ElevationGrid *>(item);
    assert(eg);
    assert(eg->getGridDepth()==8);
    
    //Load the texture image
    osg::ref_ptr<osg::Image> image = new osg::Image();
    envire::ElevationGrid::ArrayType &data = eg->getGridData(envire::ElevationGrid::ELEVATION);
   
    //convert double to uint16 
    int size = eg->getWidth()*eg->getHeight()*3;
    unsigned char* data2 = new unsigned char[size];
    unsigned char* pos = data2;
    unsigned char* end_pos = &data2[size];
    double* pos2 = data.data();
    
   //scaling between SCALING_MIN_VALUE and SCALING_MAX_VALUE meters 
   double maxValue = (1<<24)-1.0;
   double scaling = maxValue/(SCALING_MAX_VALUE-SCALING_MIN_VALUE);  
   union
   {
      uint32_t ivalue;
      unsigned char cvalue[4];
   };
   while(pos!= end_pos)
   {
      ivalue = (uint32_t)(std::max(0.0,std::min(((*pos2++)-SCALING_MIN_VALUE)*scaling,maxValue)));
      *pos++ = cvalue[0];
      *pos++ = cvalue[1];
      *pos++ = cvalue[2];
   }
   image->setImage(
	    eg->getWidth(),
	    eg->getHeight(),
	    1, // datadepth per channel
	    GL_RGB, //GL_RGB,//GL_LUMINANCE16, 
	    GL_RGB, 
	    GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
	    (unsigned char*)(data2), // unsigned char* data
	    osg::Image::USE_NEW_DELETE, // USE_NEW_DELETE, //osg::Image::NO_DELETE,// AllocationMode mode (shallow copy)
	    1);
	    
    std::cout << image->getTotalSizeInBytes() << "\n\n";
    loadElevationGridAsHeightMap(geode,*eg,image);
			
	//		loadElevationGridAsHightMap(geode,image,eg,0,0,
	//		eg->getWidth()*eg->getScaleX(),
	//		eg->getHeight()*eg->getScaleY());
  }   
  
   void ElevationGridVisualization::loadElevationGridAsHeightMap(const osg::ref_ptr<osg::Geode> geode,
							        envire::ElevationGrid &elevation_grid,
								const osg::ref_ptr<osg::Image> texture)const
  {
    envire::ElevationGrid::ArrayType &data = elevation_grid.getGridData(envire::ElevationGrid::ELEVATION);
    osg::HeightField* heightField = new osg::HeightField();
    heightField->allocate(elevation_grid.getWidth(), elevation_grid.getHeight());
    heightField->setXInterval(elevation_grid.getScaleX());
    heightField->setYInterval(elevation_grid.getScaleX());
    heightField->setSkirtHeight(0.0f);
    for (unsigned int r = 0; r < heightField->getNumRows(); r++) {
        for (unsigned int c = 0; c < heightField->getNumColumns(); c++) 
	{
	    heightField->setHeight(c, r,std::min( std::max(SCALING_MIN_VALUE,data[r][c]),SCALING_MAX_VALUE));
        }
    }
    
    // remove old drawables
    while(geode->removeDrawables(0));
    osg::ShapeDrawable *drawable = new osg::ShapeDrawable(heightField);
    geode->addDrawable(drawable);
 
    // set material properties
    osg::StateSet* state = geode->getOrCreateStateSet();
    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );

    mat->setAmbient( osg::Material::FRONT_AND_BACK,
	        osg::Vec4( .5f, .5f, .3f, 1.0f ) );
    mat->setDiffuse( osg::Material::FRONT_AND_BACK,
	        osg::Vec4( .5f, .5f, .3f, 1.0f ) );
    //mat->setSpecular( osg::Material::FRONT,
//	        osg::Vec4( 1.f, 1.f, 1.f, 1.0f ) );

    state->setAttribute( mat.get() );

    /*
    osg::Texture2D* tex = new osg::Texture2D(texture);
    tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
    */
  }
  
}
