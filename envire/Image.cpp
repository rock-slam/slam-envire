#include "Core.hpp"
#include "Image.hpp"

#include <iostream>
#include <fstream>
#include <stdint.h>

#include "cpl_string.h"
#include "gdal_priv.h"
#include "ogr_spatialref.h"

using namespace envire;
using namespace Eigen;

const std::string Image::CLASS_NAME = "envire::Image";
const std::string Image::GRAY = "gray";
const std::string Image::RED = "red";
const std::string Image::BLUE = "blue";
const std::string Image::GREEN = "green";

Image::Image(size_t width, size_t height, double scalex, double scaley) :
     Grid<double>(width,height,scalex, scaley)
{
 
}

Image::~Image()
{
    
}

Image::Image(Serialization& so)
    : Grid<double>(so)
{
    so.setClassName(CLASS_NAME);
}

void Image::serialize(Serialization& so)
{
    Grid<double>::serialize(so);
    so.setClassName(CLASS_NAME);

    writeMap( getMapFileName(so.getMapPath()) ); 
}

Image* Image::clone() 
{
    return new Image(*this);
}

void Image::writeMap(const std::string& path)
{
   
}

//so far only 8 and 16 bit gray images are supported
void Image::readMap(const std::string& path)
{
   GDALDataset  *poDataset;
   GDALAllRegister();
   poDataset = (GDALDataset *) GDALOpen(path.c_str(), GA_ReadOnly );
   if( poDataset == NULL )
      throw std::runtime_error("can not open file " + path);
   
   if(poDataset->GetRasterCount() != 1)
     throw std::runtime_error("only images with one band are supported: " + path);
   
   GDALRasterBand  *poBand;
   poBand = poDataset->GetRasterBand(1);
   width =  poDataset->GetRasterXSize();
   height =  poDataset->GetRasterYSize();
   
   //get coordinates
   //z is always zero --> the image is projected in the plane of the parent FrameNode
   double adfGeoTransform[6];
   poDataset->GetGeoTransform(adfGeoTransform);
   cord_left_top.x()= adfGeoTransform[0];
   cord_left_top.y()= adfGeoTransform[3];
   cord_right_top.x() =  adfGeoTransform[0]+width*adfGeoTransform[1];
   cord_right_top.y() =  adfGeoTransform[3];
   cord_left_bottom.x()= adfGeoTransform[0]+height*adfGeoTransform[2];
   cord_left_bottom.y()= adfGeoTransform[3]+height*adfGeoTransform[5];
   cord_right_bottom.x() =  adfGeoTransform[0]+width*adfGeoTransform[1]+height*adfGeoTransform[2];
   cord_right_bottom.y() =  adfGeoTransform[3]+width*adfGeoTransform[4]+height*adfGeoTransform[5];
   
   switch(poBand->GetColorInterpretation()) 
   {
     case GCI_GrayIndex: 
        switch(poBand->GetRasterDataType())
	{
	  case  GDT_Byte:  
	  {
	    //writing data into the Image object
	    boost::multi_array<double,2> &data(getGridData(GRAY));
	    poBand->RasterIO(GF_Read,0,0,width,height,data.data(),width,height,GDT_Byte,0,0);
	    break;
	  }
	  case GDT_Int16:
	  case GDT_UInt16:
	  {
	    //writing data into the Image object
	    boost::multi_array<double,2> &data(getGridData(GRAY));
	    poBand->RasterIO(GF_Read,0,0,width,height,data.data(),width,height,GDT_Byte,0,0);
	    break;
	  }
	  default:
	     throw std::runtime_error("color depth is not supported: " + path); 
	}
        break;  
	
     default:
	throw std::runtime_error("color mode is not supported: " + path);
   }
   GDALClose(poDataset);
}

