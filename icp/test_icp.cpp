#include "icp.hpp"
#include <Eigen/Geometry>

USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *[]) {
    std::vector<Vector3f> model, measurement;
   
    // Perform an initial transform
    Eigen::Transform3f t;
    t = Eigen::Translation3f( 0,0,.2 );
    t *= Eigen::AngleAxisf(0.2, Vector3f::UnitX());

    std::cout << t.matrix() << std::endl << std::endl;

    // generate a cube of voxels
    for(int i=0;i<8;i++) {
        Vector3f a( (i&1?1:-1), (i&2?1:-1), (i&4?1:-1) );
        model.push_back( a );
        Vector3f b = t*a;
        measurement.push_back( b );
    }

    Eigen::Transform3f alignment;
    alignment.setIdentity();
    ICP icp(model, measurement, alignment, 2, 1);

    std::cout << icp.getTransform().matrix() << std::endl << std::endl;
    icp.iterate();
    std::cout << (icp.getTransform()*t).matrix() << std::endl << std::endl;
}
