#ifndef __TRIMESH_HPP__
#define __TRIMESH_HPP__

#include "Core.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace envire {
   
    class TriMesh;
    typedef boost::shared_ptr<TriMesh> TriMesh_Ptr;

    class TriMesh : public CartesianMap 
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            TriMesh(FrameNode_Ptr node, std::string const& id = "");

            Layer_Ptr clone(std::string const& id = "");

            typedef boost::tuple<int, int, int> triangle_t;

            std::vector<Eigen::Vector3f> points;
            std::vector<triangle_t> faces;
    };
}

#endif
