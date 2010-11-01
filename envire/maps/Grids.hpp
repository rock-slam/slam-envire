#ifndef __ENVIRE_GRIDS_HPP__
#define __ENVIRE_GRIDS_HPP__

#include <envire/maps/Grid.hpp>

namespace envire
{  
  class TraversabilityGrid : public Grid<uint8_t>
  {
    public:
      static const std::string className;
      static const std::string TRAVERSABILITY;
    private:
      const static std::vector<std::string> &bands;
    public:
      TraversabilityGrid(size_t width, size_t height, double scalex, double scaley):Grid<uint8_t>::Grid(width,height,scalex,scaley){};
      TraversabilityGrid(Serialization& so):Grid<uint8_t>(so,className){unserialize(so);};
      ~TraversabilityGrid(){};
      virtual const std::string& getClassName() const {return className;};
      virtual const std::vector<std::string>& getBands() const {return bands;};
  };
  
  class ConfidenceGrid : public Grid<uint8_t>
  {
    public:
      static const std::string className;
      static const std::string CONFIDENCE;
    private:
      const static std::vector<std::string> &bands;
    public:
      ConfidenceGrid(size_t width, size_t height, double scalex, double scaley):Grid<uint8_t>::Grid(width,height,scalex,scaley){};
      ConfidenceGrid(Serialization& so):Grid<uint8_t>(so,className){unserialize(so);};
      ~ConfidenceGrid(){};
      virtual const std::string& getClassName() const {return className;};
      virtual const std::vector<std::string>& getBands() const {return bands;};
  };
  
  class ElevationGrid : public Grid<double>
  {
    public:
      static const std::string className;
      static const std::string ELEVATION;
      static const std::string ELEVATION_MIN;
      static const std::string ELEVATION_MAX;
    private:
      const static std::vector<std::string> &bands;
    public:
      ElevationGrid(size_t width, size_t height, double scalex, double scaley):Grid<double>::Grid(width,height,scalex,scaley){};
      ElevationGrid(Serialization& so):Grid<double>(so,className){unserialize(so);};
      ~ElevationGrid(){};
      virtual const std::string& getClassName() const {return className;};
      virtual const std::vector<std::string>& getBands() const {return bands;};

      double get(double x, double y) const
      { return Grid<double>::get(ELEVATION, x, y); }
      
      //read all bands from one file
    /*  virtual void readMap(const std::string& path)
      {
	std::cout << "loading all GridData for " << getClassName() << " disabled" <<std::endl;
	std::cout << "create empty grid " << getClassName() <<std::endl;
	width = 120;
	height = 850;
	scalex = 0.125;
	scaley = 0.125;
      };*/
  };
  
  class OccupancyGrid : public Grid<unsigned char>
  {
    public:
      static const std::string className;
      static const std::string OCCUPANCY;
    private:
      const static std::vector<std::string> &bands;  
    public:
      OccupancyGrid(size_t width, size_t height, double scalex, double scaley):Grid<unsigned char>::Grid(width,height,scalex,scaley){};
      OccupancyGrid(Serialization& so):Grid<unsigned char>(so,className){unserialize(so);};
      ~OccupancyGrid(){};
      virtual const std::string& getClassName() const {return className;};
      virtual const std::vector<std::string>& getBands() const {return bands;};
  };
  
  class ImageRGB24 : public Grid<unsigned char>
  {
    public:
      static const std::string className;
      static const std::string R;
      static const std::string G;
      static const std::string B;
    private:
      const static std::vector<std::string> &bands;     
    public:
      ImageRGB24(size_t width, size_t height, double scalex, double scaley):Grid<unsigned char>::Grid(width,height,scalex,scaley){};
      ImageRGB24(Serialization& so):Grid<unsigned char>(so,className){unserialize(so);};
      ~ImageRGB24(){};
      virtual const std::string& getClassName() const {return className;};
      virtual const std::vector<std::string>& getBands() const {return bands;};
      
      //save all bands in one file
      virtual void writeMap(const std::string& path)
      {
	std::cout << "saving all GridData for " << getClassName() << std::endl;
	writeGridData(bands,getFullPath(path,""));
      };
	
      //read all bands from one file
      virtual void readMap(const std::string& path)
      {
	std::cout << "loading all GridData for " << getClassName() << std::endl;
	readGridData(bands,getFullPath(path,""));
      };
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
