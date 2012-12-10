#ifndef __ENVIRE_GRIDS_HPP__
#define __ENVIRE_GRIDS_HPP__

#include <envire/maps/Grid.hpp>
#include <base/samples/distance_image.h>

namespace envire
{  
  class TraversabilityGrid : public Grid<uint8_t>
  {
      ENVIRONMENT_ITEM( TraversabilityGrid )
    public:
      static const std::string TRAVERSABILITY;
    private:
      const static std::vector<std::string> &bands;
    public:
      TraversabilityGrid() : Grid<uint8_t>() {};
      TraversabilityGrid(size_t width, size_t height, double scalex, double scaley):Grid<uint8_t>::Grid(width,height,scalex,scaley){};
      ~TraversabilityGrid(){};
      virtual const std::vector<std::string>& getBands() const {return bands;};
  };
  
  class ConfidenceGrid : public Grid<uint8_t>
  {
      ENVIRONMENT_ITEM( ConfidenceGrid )
    public:
      static const std::string CONFIDENCE;
    private:
      const static std::vector<std::string> &bands;
    public:
      ConfidenceGrid() : Grid<uint8_t>() {};
      ConfidenceGrid(size_t width, size_t height, double scalex, double scaley):Grid<uint8_t>::Grid(width,height,scalex,scaley){};
      ~ConfidenceGrid(){};
      virtual const std::vector<std::string>& getBands() const {return bands;};
  };

  class DistanceGrid : public Grid<float>
  {
      ENVIRONMENT_ITEM( DistanceGrid )
  public: 
      static const std::string DISTANCE;
      static const std::string CONFIDENCE;
  private:
      const static std::vector<std::string> &bands;
  public:
      DistanceGrid() : Grid<float>() {};
      DistanceGrid( const base::samples::DistanceImage& dimage )
	  : Grid<float>::Grid(dimage.width,dimage.height,dimage.scale_x,dimage.scale_y,dimage.center_x,dimage.center_y) {}
      DistanceGrid(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0 )
	  : Grid<float>::Grid(width,height,scalex,scaley,offsetx,offsety) {}
      virtual const std::vector<std::string>& getBands() const {return bands;}

      void copyFromDistanceImage( const base::samples::DistanceImage& dimage );
  };
  
  class ElevationGrid : public Grid<double>
  {
      ENVIRONMENT_ITEM( ElevationGrid )
    public:
      static const std::string ELEVATION;
      static const std::string ELEVATION_MIN;
      static const std::string ELEVATION_MAX;
      static const std::string ILLUMINATION;
      static const std::string VISIBILITY;
    private:
      const static std::vector<std::string> &bands;

    public:
      ElevationGrid() : Grid<double>() {};
      ElevationGrid(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0):Grid<double>::Grid(width,height,scalex,scaley,offsetx,offsety){};
      ~ElevationGrid(){};
      virtual const std::vector<std::string>& getBands() const {return bands;};

      void convertToFrame(base::samples::frame::Frame &frame)
      {
        Grid<double>::convertToFrame(ElevationGrid::ELEVATION,frame);
      }

      double& get(double x, double y)
      { return Grid<double>::get(ELEVATION, x, y); }

      double get(double x, double y) const
      { return Grid<double>::get(ELEVATION, x, y); }

      double& get( const Position& pos )
      { return Grid<double>::get(ELEVATION, pos.x, pos.y); }

      double get( const Position& pos ) const
      { return Grid<double>::get(ELEVATION, pos.x, pos.y); }

      Eigen::Vector3d getNormal( const Position& pos ) const
      {
	  size_t m = pos.x, n = pos.y;

	  const ArrayType &grid( getGridData( ELEVATION ) );
	  double slope_x = (grid[n][m-1] - grid[n][m+1]) / (getScaleX()*2.0);
	  double slope_y = (grid[n-1][m] - grid[n+1][m]) / (getScaleY()*2.0);

	  return Eigen::Vector3d( slope_x, slope_y, 1.0 ).normalized();
      }
      /** @brief get the normal vector at the given position
       */
      Eigen::Vector3d getNormal( const Point2D& pos ) const
      {
	  size_t m, n;
	  if (!toGrid(pos.x(), pos.y(), m, n ))
	      throw std::runtime_error("provided coordinates are out of the grid");

	  return getNormal( Position( m, n ) );
      }

      /** @brief get the elevation at the given point 
       *
       * The underlying model assumes the height value to be at
       * the center of the cell, and a surface is approximated
       * using the getNormal. The Height value is the value of the
       * plane at that point.
       */
      double getElevation( const Point2D& pos ) const
      {
	  size_t m, n;
	  if (!toGrid(pos.x(), pos.y(), m, n ))
	      throw std::runtime_error("provided coordinates are out of the grid");

	  double x, y;
	  fromGrid( m, n, x, y );

	  x -= pos.x();
	  y -= pos.y();

	  Eigen::Vector3d normal = getNormal( pos );
	  double slope_x = normal.x() / normal.z();
	  double slope_y = normal.y() / normal.z();

	  double height = 
	      getFromRaster( ELEVATION, m, n ) +
	      x * slope_x +
	      y * slope_y;

	  return height;
      }
  };
  
  class OccupancyGrid : public Grid<unsigned char>
  {
      ENVIRONMENT_ITEM( OccupancyGrid )
    public:
      static const std::string OCCUPANCY;
    private:
      const static std::vector<std::string> &bands;  
    public:
      OccupancyGrid() : Grid<unsigned char>() {};
      OccupancyGrid(size_t width, size_t height, double scalex, double scaley):Grid<unsigned char>::Grid(width,height,scalex,scaley){};
      ~OccupancyGrid(){};
      virtual const std::vector<std::string>& getBands() const {return bands;};
  };
  
  class ImageRGB24 : public Grid<unsigned char>
  {
      ENVIRONMENT_ITEM( ImageRGB24 )
    public:
      static const std::string R;
      static const std::string G;
      static const std::string B;
    private:
      const static std::vector<std::string> &bands;     
    public:
      ImageRGB24() : Grid<unsigned char>() {};
      ImageRGB24(size_t width, size_t height, double scalex, double scaley, double offsetx = 0.0, double offsety = 0.0 ) 
	  : Grid<unsigned char>::Grid(width,height,scalex,scaley,offsetx,offsety){};
      ~ImageRGB24(){};
      virtual const std::vector<std::string>& getBands() const {return bands;};
      
      // write all bands into a single tiff file
      bool singleFile() const { return true; }

      void copyFromFrame( const base::samples::frame::Frame& frame )
      {
	  // only support RGB frames for now
	  assert( frame.getFrameMode() == base::samples::frame::MODE_RGB );

	  unsigned char* r = getGridData( R ).data();
	  unsigned char* g = getGridData( G ).data();
	  unsigned char* b = getGridData( B ).data();

	  // copy the data
	  const unsigned char* start = frame.getImageConstPtr();
	  const unsigned char* end = frame.getImageConstPtr() + frame.getNumberOfBytes();
	  while( start < end )
	  {
	      *(r++) = *(start++);
	      *(g++) = *(start++);
	      *(b++) = *(start++);
	  }

	  itemModified();
      }
  };
  
  template<class T1, class T2>
  void copyGridToGrid(Grid<T1> &grid1, Grid<T2> &grid2,float scale)
  {
     typename Grid<T1>::ArrayType &data1 = grid1.getGridData();
     typename Grid<T2>::ArrayType &data2 = grid2.getGridData("elevation");
     for (unsigned int i1 = 0; i1< min(grid1.getHeight(),grid2.getHeight());i1++)
     {
	for (unsigned int i2 = 0; i2< min(grid1.getWidth(),grid2.getWidth());i2++)
	{
	  data2[i1][i2] = ((T2)data1[i1][i2])*scale; 
	}
     }
  };
}

#endif
