#ifndef __ENVIRE_GRID_HPP__
#define __ENVIRE_GRID_HPP__

#include <envire/Core.hpp>
#include <envire/core/Serialization.hpp>
#include <base/samples/Frame.hpp>
#include <envire/maps/GridBase.hpp>

#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <stdint.h>

//#include "cpl_string.h"
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>

#include <boost/multi_array.hpp>

#include <vector>
#include <stdexcept>
#include <base-logging/Logging.hpp>

namespace envire
{
    /** Tests if a path points to an existing file or not
     *
     * It uses directly boost::filesystem, but avoids including
     * boost::filesystem in headers. This is meant to avoid "leaking"
     * boost::filesystem to client libraries, since it pushes quite a bit of
     * requirements and is incompatible with older versions of gccxml
     */
    bool fileExists(std::string const& path);

    /**
     * @brief helper class, which all the templated Grid<> types derive from.
     *
     * Contains functionality that is common to these classes, but should
     * not necessarily go into GridBase.
     */
    class BandedGrid : public GridBase
    {
    public:
        explicit BandedGrid(std::string const& id = Environment::ITEM_NOT_ATTACHED)
	    : GridBase( id ) {}
	BandedGrid(size_t cellSizeX, size_t cellSizeY,
                double scalex, double scaley,
                double offsetx = 0.0, double offsety = 0.0,
                std::string const& id = Environment::ITEM_NOT_ATTACHED)
	    : GridBase( cellSizeX, cellSizeY, scalex, scaley, offsetx, offsety, id ) {}
	virtual void createBand( const std::string& key ) = 0;
        virtual ~BandedGrid(){};
    };

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
     * getGridData().data()[y * cellSizeX + x]
     * </code>
     */
    template <typename T>
    class Grid : public BandedGrid
    {
    public:
	typedef boost::multi_array<T,2> ArrayType;
        typedef T DataType;
	static const std::string className;
	static const std::string GRID_DATA;

    private:
	static const std::vector<std::string> bands;
        std::map<std::string, T> nodata;

    protected:
        /** @deprecated
         *
         * This is used to re-read maps that were serialized before the Grid
         * serialization strategy changed. Do not use in new code
         */
        void readMap(const std::string& path);

	/**
	 * override this method and return true if the bands in the
	 * grid should be written to a single file instead
	 * of multiple files.
	 */
	virtual bool singleFile() const { return false; }

    public:
        typedef boost::intrusive_ptr< Grid<T> > Ptr;

	Grid(std::string const& id = Environment::ITEM_NOT_ATTACHED)
            : BandedGrid(id) {}
	Grid(size_t cellSizeX, size_t cellSizeY,
                double scalex, double scaley,
                double offsetx = 0.0, double offsety = 0.0,
                std::string const& id = Environment::ITEM_NOT_ATTACHED);
	virtual ~Grid();
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

        /**
         * @overload because this class has its own declaration of className
         */
        virtual const std::string& getClassName() const {return className;};

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

        /** Returns the boost::multiarray that stores the data of the first band
         */
	ArrayType& getGridData(){return getGridData(getBands().front());};
        /** Returns the boost::multiarray that stores the data of the first band
         */
	const ArrayType& getGridData() const {return getGridData(getBands().front());};

	/** @brief create a band with the given name
	 *
	 * this method allows the generation of named bands from the base class,
	 * without actually knowing the grid type
	 */
	void createBand( const std::string& key )
	{
	    getGridData( key );
	}

        /**
         * @return the minimum and maximum values in the grid
         */
        void getMinMaxValues( const std::string& key, T& min, T& max )
        {
            min = std::numeric_limits<T>::max();
            max = std::numeric_limits<T>::min();

            ArrayType& a = getGridData( key );
            for( T* i = a.data(); i < (a.data() + a.num_elements()); i++ )
            {
                const T val = *i;
                if( val < min )
                    min = val;
                if( val > max )
                    max = val;
            }
        }

        /** Returns the boost::multiarray that stores the data of the specified band
         */
	ArrayType& getGridData( const std::string& key )
	{
	    ArrayType& data( getData<ArrayType>(key) );
	    data.resize( boost::extents[cellSizeY][cellSizeX] );
	    return data;
	};
        /** Returns the boost::multiarray that stores the data of the specified band
         */
	const ArrayType& getGridData( const std::string& key ) const
	{
	    return getData<ArrayType>(key);
	};

        /** Returns the list of bands defined on this grid
         */
	virtual const std::vector<std::string>& getBands() const {return bands;};

	Grid* clone() const;
	void set( EnvironmentItem* other );

        /** Returns the value of the cell (xi, yi) in band \c band
         */
        T getFromRaster(std::string const& band, size_t xi, size_t yi) const
        {
            return getGridData(band)[yi][xi];
        }

        /** Returns the value of the cell (xi, yi) in band \c band
         */
        T& getFromRaster(std::string const& band, size_t xi, size_t yi)
        {
            return getGridData(band)[yi][xi];
        }

        /** Returns the value of the cell in band \c band that is at the world
         * position (x, y), given relative to the (0, 0) cell
         */
        T get(std::string const& band, double x, double y) const
        {
            size_t raster_x, raster_y;
            if (!toGrid(x, y, raster_x, raster_y))
                throw std::runtime_error("provided coordinates are out of the grid");
            return getFromRaster(band, raster_x, raster_y);
        }

        T& get(std::string const& band, double x, double y)
	{
            size_t raster_x, raster_y;
            if (!toGrid(x, y, raster_x, raster_y))
                throw std::runtime_error("provided coordinates are out of the grid");
            return getFromRaster(band, raster_x, raster_y);
	}

	bool inGrid( double x, double y) const
	{
	    return (x >= 0) && (x < cellSizeX) && (y >= 0) && (y < cellSizeY);
	}

	unsigned int getGridDepth(){return sizeof(T);};	//returns the depth per grid element

        //converts the grid to base/samples/frame/Frame
        void convertToFrame(const std::string &key,base::samples::frame::Frame &frame);

	/** Helper method for serialization
         *
         * Saves the data contained in the band \c key in a GeoTiff file in the
         * path \c path
         */
	void writeGridData(const std::string& band,const std::string& path);

	/** Helper method for serialization
         *
         * Saves the data contained in the provided bands in a GeoTiff file in the
         * path \c path
         */
        void writeGridData(const std::vector<std::string>& bands,const std::string& path);
	/** Helper method for serialization
         *
         * Saves the data contained in the provided band in raw form in the
         * provided stream
         */
        void writeGridData(const std::string &key, std::ostream& os);

	/** Helper method for deserialization
         *
         * Reads the data contained in the provided band of the GDAL-readable
         * file at \c path into the \c key band of this map
         */
	void readGridData(const std::string &band,const std::string& path,int base_band = 1, boost::enable_if< boost::is_fundamental<T> >* enabler = 0);
	/** Helper method for deserialization
         *
         * Reads the data contained in the bands
         *      [base_band, base_band + bands.size[
         * of the GDAL-readable file at \c path into the listed bands of this map
         */
	void readGridData(const std::vector<std::string> &bands,const std::string& path,int base_band = 1, boost::enable_if< boost::is_fundamental<T> >* enabler = 0);
	/** Helper method for deserialization
         *
         * Reads the data contained in the given stream into the provided band
         * of this map
         */
        void readGridData(const std::string &band, std::istream& is, boost::enable_if< boost::is_fundamental<T> >* enabler = 0);

        /** Copy the data from the band \c source_band of \c _source into the \c
         * _target_band of this map
         *
         * @throw std::bad_cast if the source grid is not of the same cell type
         * than this grid
         */
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

    /* Explicit instanciations in Grids.cpp for the purpose of serialization */
    extern template class Grid<double>;
    extern template class Grid<float>;
    extern template class Grid<uint8_t>;
    extern template class Grid<int16_t>;
    extern template class Grid<uint16_t>;
    extern template class Grid<int32_t>;
    extern template class Grid<uint32_t>;

    //set unique class name for each template type
    #define GRID_DATA_VALUE "grid_data"
    template <class T> const std::string envire::Grid<T>::className = "envire::Grid_"+ std::string(typeid(T).name());
    template <class T> const std::string envire::Grid<T>::GRID_DATA = GRID_DATA_VALUE;
    template <class T> const std::vector<std::string> Grid<T>::bands = { GRID_DATA_VALUE };


    template<class T>Grid<T>::Grid(size_t cellSizeX, size_t cellSizeY,
            double scalex, double scaley, double offsetx, double offsety,
            std::string const& id) :
	BandedGrid( cellSizeX, cellSizeY, scalex, scaley, offsetx, offsety, id )
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

    template<class T>Grid<T>::~Grid()
    {

    }
    template<class T>void Grid<T>::unserialize(Serialization& so)
    {
        GridBase::unserialize(so);
        // The serialization strategy changed to save the band names in the
        // scene file instead of relying on having them defined in a static
        // array
        //
        // In the new strategy, the number of bands is saved in a map_count
        // field. In the old one, that key does not exist. Discriminate, and
        // load old maps the old way
        FileSerialization* fso = dynamic_cast<FileSerialization*>(&so);

	if (so.hasKey("map_count"))
	{
	    // read in the layer names
	    int count = so.read<int>("map_count");
	    std::vector<std::string> layers;
	    for (int i = 0; i < count; ++i)
		layers.push_back( so.read<std::string>(boost::lexical_cast<std::string>(i)) );

	    // there are three cases to differentiate here
	    // single file access, multi-file access and memory access
	    if( fso )
	    {
		std::string single_file = getFullPath(getMapFileName( fso->getMapPath(), getClassName() ),"" );
		if( singleFile() && fileExists(single_file) )
		    readGridData(layers, single_file);
		else
		    for (int i = 0; i < count; ++i)
			readGridData(layers[i], getFullPath(getMapFileName( fso->getMapPath(), getClassName() ), layers[i]));
	    }
	    else
	    {
		for (int i = 0; i < count; ++i)
		    readGridData(layers[i], so.getBinaryInputStream(getFullPath(getMapFileName( getClassName() ), layers[i])));
	    }
	}
	else
	{
	    // old way of using bands
	    if(fso)
	    {
		readMap(fso->getMapPath());
	    }
	    else
	    {
		throw std::runtime_error("can't unserialize " + className + ": missing key 'map_count' in yaml data.");
	    }
	}
    }
    template<class T>void Grid<T>::serialize(Serialization& so)
    {
	GridBase::serialize(so);

        FileSerialization* fso = dynamic_cast<FileSerialization*>(&so);

	// get layers vector first
	// base it on the bands and see if there additional layers available
	std::vector<std::string> layers;
        for (DataMap::const_iterator it = data_map.begin(); it != data_map.end(); ++it)
            if (it->second->isOfType<ArrayType>() && find( layers.begin(), layers.end(), it->first ) == layers.end() )
		layers.push_back( it->first );

	// write layer configuration to properties
	for( size_t i=0; i<layers.size(); i++ )
	    so.write(boost::lexical_cast<std::string>(i), layers[i]);

	// differentiate between single file, multi-file and memory serialization
	if( fso && singleFile() )
	    writeGridData( layers, getFullPath(getMapFileName( fso->getMapPath(), getClassName() ), "") );
	else
	    if( fso )
		for( size_t i=0; i<layers.size(); i++ )
		    writeGridData(layers[i], getFullPath(getMapFileName( fso->getMapPath(), getClassName() ), layers[i]));
	    else
		for( size_t i=0; i<layers.size(); i++ )
		    writeGridData(layers[i], so.getBinaryOutputStream(getFullPath(getMapFileName(), layers[i])));

        so.write("map_count", layers.size());
    }

    template<class T>void Grid<T>::readMap(const std::string& path)
    {
	LOG_DEBUG_S << "loading all GridData for " << getClassName();
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
        frame.init(cellSizeX,cellSizeY,sizeof(T)*8,base::samples::frame::MODE_GRAYSCALE);
        memcpy(frame.image.data(),data_.data(),frame.image.size());
        frame.frame_status = base::samples::frame::STATUS_VALID;
        frame.setAttribute<std::string>("key",key);
        frame.time = base::Time::now();
    }

    template<class T>void Grid<T>::writeGridData(const std::string &key, std::ostream& os)
    {
        ArrayType &data = getGridData(key);
        os.write(reinterpret_cast<const char*>(data.data()), sizeof(T) * data.num_elements());
    }

    template<class T>void Grid<T>::readGridData(const std::string &key, std::istream& is, boost::enable_if< boost::is_fundamental<T> >* enabler)
    {
        ArrayType &data = getGridData(key);
        is.read(reinterpret_cast<char*>(data.data()), sizeof(T) * data.num_elements());
    }

    template<class T>void Grid<T>::writeGridData(const std::string &key,const std::string& path)
    {
	std::vector<std::string> string_vector;
	string_vector.push_back(key);
	writeGridData(string_vector,path);
    }

    template<class T> void Grid<T>::writeGridData(const std::vector<std::string> &keys,const std::string& path)
    {
	LOG_DEBUG_S << "writing file "<< path;

        GDALAllRegister();
	const char *pszFormat = "GTiff";
	GDALDriver *poDriver;
	GDALDataset *poDstDS;
	char **papszOptions = NULL;

	poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if( poDriver == NULL )
	    throw std::runtime_error("GDALDriver not found.");

        GDALDataType data_type = getGDALDataTypeOfArray();
	poDstDS = poDriver->Create( path.c_str(), cellSizeX, cellSizeY,
                keys.size(), data_type,
		papszOptions );

        if (!poDstDS)
            throw std::runtime_error("failed to create file " + path);

	// get the scale and transform value so that they can be
	// added as a geotransform
	Eigen::Affine2d gridTransform = Eigen::Affine2d::Identity();
	gridTransform.translate( Eigen::Vector2d( offsetx, offsety ) );
	gridTransform.scale( Eigen::Vector2d( scalex, scaley ) );

	if(getEnvironment())
	{
	    // get the frame transformation to the root frame
	    Transform t = getEnvironment()->relativeTransform(
		    getFrameNode(),
		    getEnvironment()->getRootNode() );

	    // and flatten it to 2d
	    Eigen::Affine2d frameTransform = Eigen::Affine2d::Identity();
	    frameTransform.matrix().topLeftCorner<2,2>() = t.matrix().topLeftCorner<2,2>();
	    frameTransform.matrix().topRightCorner<2,1>() = t.matrix().topRightCorner<2,1>();

	    // apply transform to the grid transform
	    gridTransform = frameTransform * gridTransform;
	}
	else
	{
	    LOG_DEBUG_S << className << " has no environment!!!";
	}

	//calc GeoTransform
	Eigen::Matrix3d m( gridTransform.matrix() );
	double adfGeoTransform[6] = { m(0,2), m(0,0), m(0,1), m(1,2), m(1,0), m(1,1) };

	OGRSpatialReference oSRS;
	char *pszSRS_WKT = NULL;
	poDstDS->SetGeoTransform( adfGeoTransform );

	oSRS.SetUTM( 32, TRUE );
	oSRS.SetWellKnownGeogCS( "WGS84" );
	oSRS.exportToWkt( &pszSRS_WKT );
	poDstDS->SetProjection( pszSRS_WKT );
	CPLFree( pszSRS_WKT );

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
	  poBand->RasterIO(GF_Write ,0,0,cellSizeX,cellSizeY,data.data(),cellSizeX,cellSizeY,poBand->GetRasterDataType(),0,0);
	  preCallWriteBand(*iter,poBand);
	}
	GDALClose( (GDALDatasetH) poDstDS );
    }

    template<class T>void Grid<T>::readGridData(const std::vector<std::string> &keys,const std::string& path, int base_band, boost::enable_if< boost::is_fundamental<T> >* enabler)
    {
      LOG_DEBUG_S << "reading file "<< path;
      GDALDataset  *poDataset;
      GDALAllRegister();
      poDataset = (GDALDataset *) GDALOpen(path.c_str(), GA_ReadOnly );
      if( poDataset == NULL )
	  throw std::runtime_error("can not open file " + path);

      double file_cellSizeX =  poDataset->GetRasterXSize();
      double file_cellSizeY =  poDataset->GetRasterYSize();
      if (cellSizeX != 0 && file_cellSizeX != cellSizeX)
          throw std::runtime_error("file and map sizes differ along the X direction");
      if (cellSizeY != 0 && file_cellSizeY != cellSizeY)
          throw std::runtime_error("file and map sizes differ along the Y direction");
      cellSizeX = file_cellSizeX;
      cellSizeY = file_cellSizeY;

      // If the map does not yet have a scale, allow reading it from file
      //
      // It is made optional as sometime one wants to load existing bitmap
      // data that is not georeferenced in a map that is. It might get more
      // strict in the future if it becomes too fragile
      //
      // GDAL data can be arbitrarily ordered w.r.t. the direction of the X or Y
      // axis. xDir and yDir are the actual directions, which gets updated from
      // the data in the file
      int xDir = 0, yDir = 0;
      if (getScaleX() == 0 || getScaleY() == 0)
      {
          double adfGeoTransform[6];
          if( poDataset->GetGeoTransform(adfGeoTransform) == CE_Failure )
	      throw std::runtime_error("file has no geotransform information");

          scalex = fabs(adfGeoTransform[1]);
          scaley = fabs(adfGeoTransform[5]);
          if (fabs(adfGeoTransform[4] * cellSizeY) > scaley * 1e-2  || fabs(adfGeoTransform[2]) > scalex * 1e-2)
              throw std::runtime_error("cannot load rotated raster files");

          if (adfGeoTransform[1] < 0)
              xDir = -1;
          if (adfGeoTransform[5] < 0)
              yDir = -1;
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

        T* data_ptr = &data[0][0];
        // I did not manage to make GDAL invert the data for us, so I do it
        // manually. To avoid swapping values around, I allocate an additional
        // line and move the data line-by-line
        if (xDir == -1)
        {
            data.resize( boost::extents[cellSizeY + 1][cellSizeX] );
            data_ptr = &data[1][0];
        }
        if (yDir == -1)
            data_ptr += cellSizeX * (cellSizeY - 1);

        int has_nodata = 0;
        double nodata = poBand->GetNoDataValue(&has_nodata);
        if (has_nodata)
            setNoData(*iter, T(nodata));
	poBand->RasterIO(GF_Read,
                0,0,cellSizeX,cellSizeY,
                data_ptr, cellSizeX, cellSizeY,
                getGDALDataTypeOfArray(),
                0, yDir * sizeof(T) * cellSizeX);

        // See comment above as to why we do this manually
        if (xDir == -1)
        {
            for (size_t yi = 0; yi < cellSizeY; ++yi)
                for (size_t xi = 0; xi < cellSizeX; ++xi)
                    data[yi][xi] = data[yi + 1][cellSizeX - 1 - xi];
            data.resize( boost::extents[cellSizeY][cellSizeX] );
        }
      }
      GDALClose(poDataset);
    }

    template<class T>void Grid<T>::readGridData(const std::string &key,const std::string& path, int base_band, boost::enable_if< boost::is_fundamental<T> >* enabler)
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
