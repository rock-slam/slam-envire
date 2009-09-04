#ifndef __TRIMESH_HPP__
#define __TRIMESH_HPP__

#include "Core.hpp"
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace envire {

    class TriMesh : public CartesianMap 
    {
    public:
	typedef boost::tuple<int, int, int> triangle_t;

	std::vector<Eigen::Vector3f> points;
	std::vector<triangle_t> faces;

	static const std::string className;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	TriMesh();

	TriMesh(Serialization& so);
	void serialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	const std::string& getClassName() const {return className;};

	TriMesh* clone();
    };
}

#endif
