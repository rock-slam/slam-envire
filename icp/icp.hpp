#ifndef __ICP_H__
#define __ICP_H__

#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/StdVector>
#include<Eigen/QR>

#include<kdtree++/kdtree.hpp>

struct EigenAccessMethod
{
    typedef float result_type;
    inline float operator()(Eigen::Vector3f const& v, size_t i) const
    {
        return v[i];
    }
};


typedef KDTree::KDTree<3, Eigen::Vector3f, EigenAccessMethod> tree_type;

class ICP {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ICP( const std::vector<Eigen::Vector3f>& model, 
                const std::vector<Eigen::Vector3f>& measurement, 
                const Eigen::Transform3f alignment,
                float threshold, int density );

        float iterate();

        Eigen::Transform3f& getTransform() { return t; };
        std::vector<Eigen::Vector3f>& getX() { return x; };
        std::vector<Eigen::Vector3f>& getP() { return p; };
        
    private:
        const std::vector<Eigen::Vector3f>& model, measurement;
        std::vector<Eigen::Vector3f> x, p;

        tree_type kdtree;

        Eigen::Transform3f t;

        int density;
        
        float threshold;
        float mean_error;
};

#endif
