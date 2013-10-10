#include "ElevationGridVisualization.hpp"
#include <vizkit3d/ColorConversionHelper.hpp>

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

namespace envire 
{
    
ElevationGridVisualization::ElevationGridVisualization() : cycleHeightColor(true)
{
}
    
  bool ElevationGridVisualization::handlesItem(envire::EnvironmentItem *item) const
  {
      if(dynamic_cast<envire::ElevationGrid *>(item))
	  return true;  
      return false;
  }    

  void ElevationGridVisualization::updateNode ( envire::EnvironmentItem* item, osg::Group* node ) const
  {
    osg::ref_ptr<osg::Geode> geode = node->getChild(0)->asGeode();
    envire::ElevationGrid *eg = dynamic_cast<envire::ElevationGrid *>(item);
    
    //Load the texture image
    osg::ref_ptr<osg::Image> image; 
    
    envire::ElevationGrid::ArrayType &data = eg->getGridData(envire::ElevationGrid::ELEVATION);

    double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::min();
    for (size_t yi = 0; yi < eg->getCellSizeY(); ++yi)
    {
        for (size_t xi = 0; xi < eg->getCellSizeX(); ++xi)
        {
            if(fabs(data[yi][xi]) != std::numeric_limits<double>::infinity())
            {
                min_z = std::min(min_z, data[yi][xi]);
                max_z = std::max(max_z, data[yi][xi]);
            }
        }
    }

    // see if we want to visualize illumination and or visibility
    double *illumination = NULL, *visibility = NULL;
    if( eg->hasData( envire::ElevationGrid::ILLUMINATION ) )
	illumination = eg->getGridData(envire::ElevationGrid::ILLUMINATION).data();
    if( eg->hasData( envire::ElevationGrid::VISIBILITY ) )
	visibility = eg->getGridData(envire::ElevationGrid::VISIBILITY).data();
   
   // setup height color image
   if(cycleHeightColor || illumination || visibility)
   {
	image = new osg::Image();

        //convert double to uint16 
        int size = eg->getWidth()*eg->getHeight()*4;
        unsigned char* image_raw_data = new unsigned char[size];
        unsigned char* pos = image_raw_data;
        unsigned char* end_pos = &image_raw_data[size];
        double* pos2 = data.data();

        //scaling between SCALING_MIN_VALUE and SCALING_MAX_VALUE meters 
        double scaling = std::abs(max_z - min_z);
        if(scaling == 0)
            scaling = 1.0;

        // fill image with color
        while(pos!= end_pos)
        {
            double hue = (*pos2 - std::floor(*pos2)) / scaling;
            pos2++;
            osg::Vec4f col(1.0,1.0,0.6,1.0);
	    double luminance = 0.6;
	    if( illumination )
		luminance *= *(illumination++);
	    if( visibility )
		col.a() = *(visibility++);
	    if( cycleHeightColor )
		vizkit3d::hslToRgb( hue - std::floor(hue), 1.0, luminance, col.r(), col.g(), col.b());
	    else
		col = osg::Vec4f( col.r() * luminance, col.g() * luminance, col.b() * luminance, col.a() );
	    *pos++ = (unsigned char)(col.r() * 255.0);
	    *pos++ = (unsigned char)(col.g() * 255.0);
	    *pos++ = (unsigned char)(col.b() * 255.0);
	    *pos++ = (unsigned char)(col.a() * 255.0);
        }
        
        image->setImage(
                eg->getWidth(),
                eg->getHeight(),
                1, // datadepth per channel
                GL_RGBA, 
                GL_RGBA, 
                GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
                (unsigned char*)(image_raw_data), // unsigned char* data
                osg::Image::USE_NEW_DELETE, // USE_NEW_DELETE, //osg::Image::NO_DELETE,// AllocationMode mode (shallow copy)
                1);
   }
    
    loadElevationGridAsHeightMap(geode,*eg,image, min_z > 0.0 ? 0.0 : min_z);
  }   
  
   void ElevationGridVisualization::loadElevationGridAsHeightMap(const osg::ref_ptr<osg::Geode> geode,
							        envire::ElevationGrid &elevation_grid,
								const osg::ref_ptr<osg::Image> texture,
                                                                double z_offset
                                                                )const
  {
    envire::ElevationGrid::ArrayType &data = elevation_grid.getGridData(envire::ElevationGrid::ELEVATION);
    osg::HeightField* heightField = new osg::HeightField();
    heightField->allocate(elevation_grid.getCellSizeX(), elevation_grid.getCellSizeY());
    heightField->setXInterval(elevation_grid.getScaleX());
    heightField->setYInterval(elevation_grid.getScaleY());
    heightField->setOrigin(osg::Vec3d(elevation_grid.getOffsetX(), elevation_grid.getOffsetY(), 0.0));
    heightField->setSkirtHeight(0.0f);
    for (unsigned int r = 0; r < heightField->getNumRows(); r++) {
        for (unsigned int c = 0; c < heightField->getNumColumns(); c++) 
	{
            if( fabs( data[r][c] ) != std::numeric_limits<double>::infinity() )
                heightField->setHeight(c, r, data[r][c]);
            else
                heightField->setHeight(c, r, z_offset);
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
    
    // apply texture
    if(texture)
    {
        osg::Texture2D* tex = new osg::Texture2D(texture);
        tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        state->setTextureAttributeAndModes(0, tex);
    }
    else
    {
        state->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
    }
  }

bool ElevationGridVisualization::isHeightColorCycled() const
{
    return cycleHeightColor;
}

void ElevationGridVisualization::setCycleHeightColor(bool enabled)
{
    cycleHeightColor = enabled;
    emit propertyChanged("cycle_height_color");
}
  
}
