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

	/** per point colors. Size and order needs to match the points array
	 */
	std::vector<Eigen::Vector3f> colors;
	
	/** per point normals. Size and order of this vector needs to match the
	 * points array or can be empty
	 */
	std::vector<Eigen::Vector3f> normals;
	
	/** definition of 3d points of the trimesh
	 */
	std::vector<Eigen::Vector3f> points;

	/** vector of triangle_t, which are indeces into the points vector the
	 * number of trimeshes is independent of the number of points in the
	 * map
	 */
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
