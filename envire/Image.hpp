#ifndef __ENVIRE_IMAGE_HPP__
#define __ENVIRE_IMAGE_HPP__

#include "Core.hpp"
#include "Grid.hpp"
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "boost/multi_array.hpp"

#include <vector>
#include <stdexcept>
#include <stdint.h>

namespace envire {
    class Image : public Grid<double>
    {
    public:
	static const std::string CLASS_NAME;
	
	static const std::string GRAY;
	static const std::string RED;
	static const std::string BLUE;
	static const std::string GREEN;
	
	//coordinates of the image edges in respect to
	//the parent framenode 
	//the image is projected in the plane of the framenode
	//--> z is always zero
	Eigen::Vector3d cord_left_top;
	Eigen::Vector3d cord_right_top;
	Eigen::Vector3d cord_left_bottom;
	Eigen::Vector3d cord_right_bottom;
		
     private:
 
     public:
	Image(size_t width, size_t height, double scalex, double scaley);
	~Image();

	Image(Serialization& so);
	void serialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);
	
	const std::string& getClassName() const {return CLASS_NAME;};
	Image* clone();
	
//	bool setBand(GDALRasterBand  *poBand);
    };
}

#endif
