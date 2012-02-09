#ifndef __TRIMESH_HPP__
#define __TRIMESH_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/core/Serialization.hpp>

#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <stdexcept>

namespace envire {

    class TriMesh : public Pointcloud 
    {
	ENVIRONMENT_ITEM( TriMesh )

    public:
	typedef boost::tuple<int, int, int> triangle_t;

    public:
	/** vector of triangle_t, which are indeces into the points vector the
	 * number of trimeshes is independent of the number of points in the
	 * map
	 */
	std::vector<triangle_t> faces;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TriMesh();

	void serialize(Serialization& so);
    void unserialize(Serialization& so);

	void calcVertexNormals( void );
    };
}

#endif
