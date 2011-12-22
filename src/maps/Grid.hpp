#ifndef __ENVIRE_GRID_HPP__
#define __ENVIRE_GRID_HPP__

#include <envire/Core.hpp>
#include <base/samples/frame.h>
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
    /** Generic handling of a multi-layer grid
     *
     * The data is stored as a boost multiarray. The indices in the boost
     * multiarray is
     *
     * <code>
     * getGridData()[y][x]
     * </code>
     *
     * so that the internal memory layout is the classical
     *
     * <code>
     * getGridData().data()[y * width + x]
     * </code>
     */
    template <typename T>
    class Grid : public GridBase
    {
    public:
	typedef boost::multi_array<T,2> ArrayType; 
	static const std::string className;
	static const std::string GRID_DATA;

    private:
	const static std::vector<std::string> &bands;
        std::map<std::string, T> nodata;

    protected:	
	Grid(Serialization& so,const std::string &class_name);

        /** @deprecated
         *
         * This is used to re-read maps that were serialized before the Grid
         * serialization strategy changed. Do not use in new code
         */
        void readMap(const std::string& path);

    public:
        typedef boost::intrusive_ptr< Grid<T> > Ptr;

	Grid() {}
	Grid(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0);
	~Grid();
	Grid(Serialization& so);
	void serialize(Serialization& so);
	void unserialize(Serialization& so);

        /** Returns true if there is band information for the given key */
        bool hasBand(std::string const& key) const
        {
            return hasData<ArrayType>(key);
        }

        /** Sets the nodata value for the given band */
        void setNoData(std::string const& key, T value)
        {
            nodata[key] = value;
        }

        /** Returns the nodata value for the given band, if one has been
         * defined.
         *
         * The boolean flag is true if there is one, and false otherwise
         */
        std::pair<T, bool> getNoData(std::string const& key) const
        {
            typename std::map<std::string, T>::const_iterator it =
                nodata.find(key);
            if (it == nodata.end())
                return std::make_pair(T(), false);
            else
                return std::make_pair(it->second, true);
        }

        /** Sets the nodata value for all the bands */
        void setNoData(T value)
        {
            for (DataMap::iterator it = data_map.begin(); it != data_map.end(); ++it)
            {
                if (it->second->isOfType<ArrayType>())
                    nodata[it->first] = value;
            }
        }
        /** Returns the nodata value of the first band */
        std::pair<T, bool> getNoData() const
        { return getNoData(getBands().front()); }
	
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

        //converts the grid to base/samples/frame/Frame 
        void convertToFrame(const std::string &key,base::samples::frame::Frame &frame);

	//saves the GridData with a specific key to a GTiff 
	void writeGridData(const std::string &key,const std::string& path);
        void writeGridData(const std::vector<std::string> &keys,const std::string& path);
	
	//reads the GridData with a specific key 
	void readGridData(const std::string &key,const std::string& path,int base_band = 1);
	void readGridData(const std::vector<std::string> &keys,const std::string& path,int base_band = 1);

        void copyBandFrom(GridBase const& _source, std::string const& source_band, std::string const& _target_band = "")
        {
            Grid<T> const& source = dynamic_cast< Grid<T> const& >(_source);
            std::string target_band = _target_band;
            if (_target_band.empty())
                target_band = source_band;

            getGridData(target_band) = source.getGridData(source_band);
            std::pair<T, bool> no_data = source.getNoData(source_band);
            if (no_data.second)
                setNoData(target_band, no_data.first);
        }

      protected:
	
	//this function is called after a band is safed to the file
	//overwrite this function if you want to add specific meta data
	virtual void preCallWriteBand(std::string key,GDALRasterBand  *poBand){};
	
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
 
    
    template<class T>Grid<T>::Grid(size_t width, size_t height, double scalex, double scaley, double offsetx, double offsety ) :
	GridBase( width, height, scalex, scaley, offsetx, offsety )
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
        // The serialization strategy changed to save the band names in the
        // scene file instead of relying on having them defined in a static
        // array
        //
        // In the new strategy, the number of bands is saved in a map_count
        // field. In the old one, that key does not exist. Discriminate, and
        // load old maps the old way
        if (so.hasKey("map_count"))
        {
            int count = so.read<int>("map_count");
            std::string base_path = getMapFileName(so.getMapPath());
            for (int i = 0; i < count; ++i)
            {
                std::string layer_name = so.read<std::string>(boost::lexical_cast<std::string>(i));
                readGridData(layer_name,getFullPath(base_path,layer_name));
            }
        }
        else
        {
            readMap( getMapFileName(so.getMapPath())); 
        }
    }
    template<class T>void Grid<T>::serialize(Serialization& so)
    {
	CartesianMap::serialize(so);
        std::string base_path = getMapFileName(so.getMapPath());
        int map_index = 0;
        for (DataMap::const_iterator it = data_map.begin(); it != data_map.end(); ++it)
        {
            if (it->second->isOfType<ArrayType>())
            {
                so.write(boost::lexical_cast<std::string>(map_index), it->first);
                map_index++;
                writeGridData(it->first,getFullPath(base_path,it->first));
            }
        }
        so.write("map_count", map_index);
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

    template<class T>
    void Grid<T>::convertToFrame(const std::string &key,base::samples::frame::Frame &frame)
    {
        ArrayType& data_ = getGridData(key);
        frame.init(width,height,sizeof(T)*8,base::samples::frame::MODE_GRAYSCALE);
        memcpy(frame.image.data(),data_.data(),frame.image.size());
        frame.frame_status = base::samples::frame::STATUS_VALID;
        frame.setAttribute<std::string>("key",key);
        frame.time = base::Time::now();
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
	  Eigen::Matrix4d m = (t * Eigen::DiagonalMatrix<double,3>(scalex, scaley,0)).matrix();
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
          std::pair<T, bool> no_data = getNoData(*iter);
          if (no_data.second)
              poBand->SetNoDataValue(no_data.first);
	  poBand->RasterIO(GF_Write ,0,0,width,height,data.data(),width,height,poBand->GetRasterDataType(),0,0);
	  preCallWriteBand(*iter,poBand);
	}
	GDALClose( (GDALDatasetH) poDstDS );
    }
      
    template<class T>void Grid<T>::readGridData(const std::vector<std::string> &keys,const std::string& path, int base_band)
    {
      std::cout << "reading file "<< path << std::endl;
      GDALDataset  *poDataset;
      GDALAllRegister();
      poDataset = (GDALDataset *) GDALOpen(path.c_str(), GA_ReadOnly );
      if( poDataset == NULL )
	  throw std::runtime_error("can not open file " + path);
      
      double file_width =  poDataset->GetRasterXSize();
      double file_height =  poDataset->GetRasterYSize();  
      if (width != 0 && file_width != width)
          throw std::runtime_error("file width and map width differ");
      if (height != 0 && file_height != height)
          throw std::runtime_error("file height and map height differ");
      width  = file_width;
      height = file_height;
      
      // If the map does not yet have a scale, allow reading it from file
      //
      // It is made optional as sometime one wants to load existing bitmap
      // data that is not georeferenced in a map that is. It might get more
      // strict in the future if it becomes too fragile
      if (getScaleX() == 0 || getScaleY() == 0)
      {
          double adfGeoTransform[6];
          poDataset->GetGeoTransform(adfGeoTransform);  
          scalex = sqrt(adfGeoTransform[1]*adfGeoTransform[1]+adfGeoTransform[4]*adfGeoTransform[4]);
          scaley = sqrt(adfGeoTransform[2]*adfGeoTransform[2]+adfGeoTransform[5]*adfGeoTransform[5]);
      }

      GDALRasterBand  *poBand;
      if((unsigned int )(poDataset->GetRasterCount()-base_band+1) < keys.size())
      {
	std::stringstream strstr;
	strstr << "Can not read file: " << path << ". File has " << poDataset->GetRasterCount() 
	      << " raster bands but " << keys.size() << " are expected.";
	throw std::runtime_error(strstr.str());
      }
      
      //loaded all bands 
      std::vector<std::string>::const_iterator iter = keys.begin();
      for(int i=0;iter != keys.end(); iter++,i++)
      {
	poBand = poDataset->GetRasterBand(base_band + i);
	if(!poBand)
	{
	  std::stringstream strstr;
	  strstr << "Can not read file: " << path << ". Raster band " << base_band+i 
		 << " could not be opened.";
	  throw std::runtime_error(strstr.str());
	}
	checkBandType(poBand);
	//writing data into the grid object
	boost::multi_array<T,2> &data(getGridData(*iter));

        int has_nodata = 0;
        double nodata = poBand->GetNoDataValue(&has_nodata);
        if (has_nodata)
            setNoData(*iter, nodata);
	poBand->RasterIO(GF_Read,0,0,width,height,data.data(),width,height,poBand->GetRasterDataType(),0,0);
      }
      GDALClose(poDataset);
    }
    
    template<class T>void Grid<T>::readGridData(const std::string &key,const std::string& path, int base_band)
    {
	std::vector<std::string> string_vector;
	string_vector.push_back(key);
	readGridData(string_vector,path,base_band);
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
