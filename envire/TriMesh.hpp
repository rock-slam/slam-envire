#ifndef __TRIMESH_HPP__
#define __TRIMESH_HPP__

#include "Core.hpp"
#include "Pointcloud.hpp"
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <stdexcept>

namespace envire {

    class TriMesh : public Pointcloud 
    {
    public:
	typedef boost::tuple<int, int, int> triangle_t;

    public:
	/** vector of triangle_t, which are indeces into the points vector the
	 * number of trimeshes is independent of the number of points in the
	 * map
	 */
	std::vector<triangle_t> faces;

    public:
	static const std::string className;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TriMesh();
	~TriMesh();

	TriMesh(Serialization& so);
	void serialize(Serialization& so);

	void writeMap(const std::string& path);
	void readMap(const std::string& path);

	const std::string& getClassName() const {return className;};

	TriMesh* clone();

	void calcVertexNormals( void );
    };
}

#endif
