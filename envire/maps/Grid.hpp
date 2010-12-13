#ifndef __ENVIRE_GRID_HPP__
#define __ENVIRE_GRID_HPP__

#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>

#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <stdint.h>

//#include "cpl_string.h"
#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include "boost/multi_array.hpp"

#include <vector>
#include <stdexcept>

namespace envire 
{
    template <typename T>
    class Grid : public GridBase
    {
    public:
	typedef boost::multi_array<T,2> ArrayType; 
	static const std::string className;
	static const std::string GRID_DATA;
    private:
	const static std::vector<std::string> &bands;
    private:
	Grid(){};
    protected:	
	Grid(Serialization& so,const std::string &class_name);
    public:
	Grid(size_t width, size_t height, double scalex, double scaley);
	~Grid();
	Grid(Serialization& so);
	void serialize(Serialization& so);
	void unserialize(Serialization& so);

	virtual void writeMap(const std::string& path);
	virtual void readMap(const std::string& path);
	
	//be careful !!!
	//because of the data layout of images the ArrayType
	//as the following indices  
	//data[height][width]
	//the intern memory layout is p = (data.data())[width*row + col]
	ArrayType& getGridData(){return getGridData(getBands().front());};
	const ArrayType& getGridData() const {return getGridData(getBands().front());};
	ArrayType& getGridData( const std::string& key )
	{
	    ArrayType& data( getData<ArrayType>(key) );
	    data.resize( boost::extents[height][width] );
	    return data;
	};

	const ArrayType& getGridData( const std::string& key ) const
	{
	    return getData<ArrayType>(key);
	};
	
	virtual const std::string& getClassName() const {return className;};
	virtual const std::vector<std::string>& getBands() const {return bands;};
	
	Grid* clone() const;
	void set( EnvironmentItem* other );

	bool toGrid( double x, double y, size_t& m, size_t& n ) const;

        T getFromRaster(std::string const& band, size_t x, size_t y) const
        {
            return getGridData(band)[y][x];
        } 

        T get(std::string const& band, double x, double y) const
        {
            size_t raster_x, raster_y;
            if (!toGrid(x, y, raster_x, raster_y))
                throw std::runtime_error("provided coordinates are out of the grid");
            return getFromRaster(band, raster_x, raster_y);
        } 

	bool inGrid( double x, double y) const
	{
	    return (x >= 0) && (x < width) && (y >= 0) && (y < height); 
	};
	
	size_t getWidth() const { return width; };
	size_t getHeight() const { return height; };

	double getScaleX() const { return scalex; };
	double getScaleY() const { return scaley; };

	unsigned int getGridDepth(){return sizeof(T);};	//returns the depth per grid element
      protected:
	//saves the GridData with a specific key to a GTiff 
	void writeGridData(const std::string &key,const std::string& path);
        void writeGridData(const std::vector<std::string> &keys,const std::string& path);
	
	//reads the GridData with a specific key 
	void readGridData(const std::string &key,const std::string& path);
	void readGridData(const std::vector<std::string> &keys,const std::string& path);
	
	//this function is called after a band is safed to the file
	//overwrite this function if you want to add specific meta data
	virtual void preCallWriteBand(std::string key,GDALRasterBand  *poBand){};
	
	//this function is called after a band is loaded
	//overwrite this function if you want to load specific meta data
	virtual void preCallReadBand(std::string key,GDALRasterBand  *poBand){};
	
	//returns the path of the GTiff image
	std::string getFullPath(const std::string &path,const std::string &key)
	{return path+"_"+key+".tiff";};
	
	//checks if poBand can be loaded into this
	void checkBandType(GDALRasterBand  *poBand);
	GDALDataType getGDALDataTypeOfArray();

    };
    
    //set unique class name for each template type
    #define GRID_DATA_VALUE "grid_data"
    template <class T> const std::string envire::Grid<T>::className = "envire::Grid_"+ std::string(typeid(T).name());
    template <class T> const std::string envire::Grid<T>::GRID_DATA = GRID_DATA_VALUE;
    template <class T> static const std::vector<std::string> & initbands()
    {
      static std::vector<std::string> bands;
      if(bands.empty())
	 bands.push_back(GRID_DATA_VALUE);
      return bands;
    };
    template <class T> const std::vector<std::string> & Grid<T>::bands = initbands<T>();
 
    
    template<class T>Grid<T>::Grid(size_t width, size_t height, double scalex, double scaley) :
	GridBase( width, height, scalex, scaley )
    {
      static bool initialized = false;
      if(!initialized)
      {
	initialized = true;
	//check if typeid is working properly
	//if not you can delete this but you will get incompatible save files
	if(std::string(typeid(int).name()) != "i"||
	   std::string(typeid(float).name()) != "f"||
	   std::string(typeid(double).name()) != "d"||
	   std::string(typeid(uint8_t).name()) != "h")
	   throw std::runtime_error("envire::Grid<T> typeid is not working like expected. ClassName can not be set to an unique string value.");
      }
    }
    
    //this is for initializing CartesianMap from a child class without loading GridData
    template<class T>Grid<T>::Grid(Serialization& so,const std::string &class_name)
      : GridBase(so)
    {
    }
    template<class T>Grid<T>::Grid(Serialization& so)
      : GridBase(so)
    {
	unserialize(so);
    }
    template<class T>Grid<T>::~Grid()
    {
      
    }
    template<class T>void Grid<T>::unserialize(Serialization& so)
    {
	so.setClassName(getClassName());
	readMap( getMapFileName(so.getMapPath())); 
    }
    template<class T>void Grid<T>::serialize(Serialization& so)
    {
	CartesianMap::serialize(so);
	so.setClassName(getClassName());
	writeMap( getMapFileName(so.getMapPath())); 
    }

    template<class T>void Grid<T>::writeMap(const std::string& path)
    {
	std::cout << "saving all GridData for " << getClassName() << std::endl;
	const std::vector<std::string> &bands_ = getBands();
	std::vector<std::string>::const_iterator iter = bands_.begin();
	for(;iter != bands_.end();iter++)
	  writeGridData(*iter,getFullPath(path,*iter));
    }
    
    template<class T>void Grid<T>::readMap(const std::string& path)
    {
	std::cout << "loading all GridData for " << getClassName() << std::endl;
	const std::vector<std::string> &bands_ = getBands();
	std::vector<std::string>::const_iterator iter = bands_.begin();
	for(;iter != bands_.end();iter++)
	  readGridData(*iter,getFullPath(path,*iter));
    }
    
    template<class T>Grid<T>* Grid<T>::clone() const
    {
	return new Grid<T>(*this);
    }
   
    template<class T>void Grid<T>::set(EnvironmentItem* other)
    {
	Grid<T>* gp = dynamic_cast<Grid<T>*>( other );
	if( gp ) operator=( *gp );
    }

    template<class T>void Grid<T>::writeGridData(const std::string &key,const std::string& path)
    {
	std::vector<std::string> string_vector;
	string_vector.push_back(key);
	writeGridData(string_vector,path);
    }
   
    template<class T> void Grid<T>::writeGridData(const std::vector<std::string> &keys,const std::string& path)
    {
	std::cout << "writing file "<< path << std::endl;
        GDALAllRegister();
	const char *pszFormat = "GTiff";
	GDALDriver *poDriver;
	GDALDataset *poDstDS;       
	char **papszOptions = NULL;

	poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if( poDriver == NULL )
	    throw std::runtime_error("GDALDriver not found.");
	
        GDALDataType data_type = getGDALDataTypeOfArray();
	poDstDS = poDriver->Create( path.c_str(), width, height, keys.size(), data_type, 
		papszOptions );

	if(getEnvironment())
	{
	  envire::FrameNode::TransformType t = getEnvironment()->relativeTransform(
		  getFrameNode(),
		  getEnvironment()->getRootNode() );

	  //calc GeoTransform	
	  Eigen::Matrix4d m = (t * Eigen::Scaling3d(scalex, scaley,0)).matrix();
	  double adfGeoTransform[6] = { m(0,3), m(0,0), m(0,1), m(1,3), m(1,0), m(1,1) };
	  
	  OGRSpatialReference oSRS;
	  char *pszSRS_WKT = NULL;
	  poDstDS->SetGeoTransform( adfGeoTransform );

	  oSRS.SetUTM( 32, TRUE );
	  oSRS.SetWellKnownGeogCS( "WGS84" );
	  oSRS.exportToWkt( &pszSRS_WKT );
	  poDstDS->SetProjection( pszSRS_WKT );
	  CPLFree( pszSRS_WKT );
	}
	else
	{
	  std::cout << className << " has no environment!!!" << std::endl;
	}

	GDALRasterBand *poBand;
	std::vector<std::string>::const_iterator iter = keys.begin();
	for(int i=1;iter != keys.end(); iter++,i++)
	{
	  poBand =  poDstDS->GetRasterBand(i);
	  if(!poBand)
	  {
	    std::stringstream strstr;
	    strstr << "Can not write file: " << path << ". Raster band " << i 
		  << " could not be written.";
	    throw std::runtime_error(strstr.str());
	  }
	  ArrayType &data = getGridData(*iter);
	  poBand->RasterIO(GF_Write ,0,0,width,height,data.data(),width,height,poBand->GetRasterDataType(),0,0);
	  preCallWriteBand(*iter,poBand);
	}
	GDALClose( (GDALDatasetH) poDstDS );
    }
      
    template<class T>void Grid<T>::readGridData(const std::vector<std::string> &keys,const std::string& path)
    {
      std::cout << "reading file "<< path << std::endl;
      GDALDataset  *poDataset;
      GDALAllRegister();
      poDataset = (GDALDataset *) GDALOpen(path.c_str(), GA_ReadOnly );
      if( poDataset == NULL )
	  throw std::runtime_error("can not open file " + path);
      
      width =  poDataset->GetRasterXSize();
      height =  poDataset->GetRasterYSize();  
      
      //calc scaling
      double adfGeoTransform[6];
      poDataset->GetGeoTransform(adfGeoTransform);  
      scalex = sqrt(adfGeoTransform[1]*adfGeoTransform[1]+adfGeoTransform[4]*adfGeoTransform[4]);
      scaley = sqrt(adfGeoTransform[2]*adfGeoTransform[2]+adfGeoTransform[5]*adfGeoTransform[5]);

      GDALRasterBand  *poBand;
      if((unsigned int )poDataset->GetRasterCount() != keys.size())
      {
	std::stringstream strstr;
	strstr << "Can not read file: " << path << ". File has " << poDataset->GetRasterCount() 
	      << " raster bands but " << keys.size() << " are expected.";
	throw std::runtime_error(strstr.str());
      }
      
      //loaded all bands 
      std::vector<std::string>::const_iterator iter = keys.begin();
      for(int i=1;iter != keys.end(); iter++,i++)
      {
	poBand = poDataset->GetRasterBand(i);
	if(!poBand)
	{
	  std::stringstream strstr;
	  strstr << "Can not read file: " << path << ". Raster band " << i 
		 << " could not be opened.";
	  throw std::runtime_error(strstr.str());
	}
	checkBandType(poBand);
	//writing data into the grid object
	boost::multi_array<T,2> &data(getGridData(*iter));
	poBand->RasterIO(GF_Read,0,0,width,height,data.data(),width,height,poBand->GetRasterDataType(),0,0);
	preCallReadBand(*iter,poBand);
      }
      GDALClose(poDataset);
    }
    
    template<class T>void Grid<T>::readGridData(const std::string &key,const std::string& path)
    {
	std::vector<std::string> string_vector;
	string_vector.push_back(key);
	readGridData(string_vector,path);
    }

    //returns the gird indices if the coordinates are on the grid
    template<class T>bool Grid<T>::toGrid( double x, double y, size_t& m, size_t& n ) const
    {
	int am = floor(x/scalex);
	int an = floor(y/scaley);
	if( 0 <= am && am < static_cast<int>(width) && 0 <= an && an < static_cast<int>(height) )
	{
	    m = am;
	    n = an;
	    return true;
	}
	else {
	    return false;
	}
    }
        
    template<class T> GDALDataType Grid<T>::getGDALDataTypeOfArray()
    {
      if(typeid(T) == typeid(unsigned char))
	return GDT_Byte;
      else if(typeid(T) == typeid(int16_t))
	return GDT_Int16;
      else if(typeid(T) == typeid(uint16_t))
	return GDT_UInt16;
      else if(typeid(T) == typeid(int32_t))
	return GDT_Int32;
      else if(typeid(T) == typeid(uint32_t))
	return GDT_UInt32;
      else if(typeid(T) == typeid(float) && sizeof(float)==4)
	return GDT_Float32;
      else if(typeid(T) == typeid(double)&& sizeof(double)==8)
	return GDT_Float64;
      
      throw std::runtime_error(std::string("enview::Grid<T>:") + std::string("type ") + typeid(T).name() + " is not supported by " + getClassName());
    }
    
    //will throw a runtime_error if the grid array has another type than poBand
    template<class T> void Grid<T>::checkBandType(GDALRasterBand  *poBand)
    {
        std::string type_name;
	int size = 0;
        switch(poBand->GetRasterDataType())
	{
	  case  GDT_Byte:
	    if(typeid(T)== typeid(unsigned char))
	      return;
	    type_name = typeid(unsigned char).name();
	    size = sizeof(unsigned char);
	    break;
	  case GDT_Int16:
	    if(typeid(T)== typeid(int16_t))
	      return;
	    type_name = typeid(int16_t).name();
	    size = sizeof(int16_t);
	    break;
	  case GDT_UInt16:
	   if(typeid(T)== typeid(uint16_t))
	      return;
	    type_name = typeid(uint16_t).name();
	    size = sizeof(uint16_t);
	    break;
	  case GDT_Int32 :
	   if(typeid(T)== typeid(int32_t))
	      return;
	    type_name = typeid(int32_t).name();
	    size = sizeof(int32_t);
	    break;
	  case GDT_UInt32 :
	    if(typeid(T)== typeid(uint32_t))
	      return;
	    type_name = typeid(uint32_t).name();
	    size = sizeof(uint32_t);
	    break;
	  case GDT_Float32:
	   if(typeid(T)== typeid(float)&&sizeof(T)==4)
	      return;
	    type_name = typeid(float).name();
	    size = 4;
	    break;
	  case GDT_Float64:
	    if(typeid(T)== typeid(double)&&sizeof(T)==8)
	      return;
	    type_name = typeid(double).name();
	    size = 8;
	    break;
	  default:
	    throw std::runtime_error("enview::Grid<T>: GDT type is not supported.");  
	}
	std::stringstream strstr;
	strstr << "enview::Grid<T>: type missmatch: the array is of type "<< typeid(T).name()
	       << " and length: " << sizeof(T) << ". But the Band data are of type "
	       << type_name << " and length: " << size;
	throw std::runtime_error(strstr.str());
    }
}
#endif
