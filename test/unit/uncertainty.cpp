#include <envire/Core.hpp>

#define BOOST_TEST_MODULE UncertaintyTest 
#include <boost/test/included/unit_test.hpp>

using namespace envire;

BOOST_AUTO_TEST_CASE( test_rigid_body_state ) 
{
    base::samples::RigidBodyState rbs;
    TransformWithUncertainty t( rbs );
}


BOOST_AUTO_TEST_CASE( test_relative_uncertainty ) 
{
    // test if the relative transform also 
    // takes the uncertainty into account
    Eigen::Matrix<double,6,6> lt1; 
    lt1 <<
	0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, -1.0;

    TransformWithUncertainty t1(
	    Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(1,0,0)) * Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitX()) ),
	    lt1 );

    Eigen::Matrix<double,6,6> lt2; 
    lt2 <<
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 3.0;

    TransformWithUncertainty t2(
	    Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0,1,2)) * Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY()) ),
	    lt2 );

    // chain a transform with uncertainty
    TransformWithUncertainty tr = t2 * t1;

    // and recover the second transform
    TransformWithUncertainty t2r = tr.compositionInv( t1 );
    TransformWithUncertainty t1r = tr.preCompositionInv( t2 );

    const double sigma = 1e-12;

    BOOST_CHECK( t2.getTransform().matrix().isApprox( t2r.getTransform().matrix(), sigma ) );
    BOOST_CHECK( t2.getCovariance().isApprox( t2r.getCovariance(), sigma ) );

    BOOST_CHECK( t1.getTransform().matrix().isApprox( t1r.getTransform().matrix(), sigma ) );
    BOOST_CHECK( t1.getCovariance().isApprox( t1r.getCovariance(), sigma ) );

//    std::cout << "t2 transforms: " << std::endl;
//    std::cout << t2.getTransform().matrix() << std::endl;
//    std::cout << t2r.getTransform().matrix() << std::endl;
//
//    std::cout << "t2 covariances: " << std::endl;
//    std::cout << t2.getCovariance() << std::endl;
//    std::cout << t2r.getCovariance() << std::endl;
//
//    std::cout << "t1 transforms: " << std::endl;
//    std::cout << t1.getTransform().matrix() << std::endl;
//    std::cout << t1r.getTransform().matrix() << std::endl;
//
//    std::cout << "t1 covariances: " << std::endl;
//    std::cout << t1.getCovariance() << std::endl;
//    std::cout << t1r.getCovariance() << std::endl;
    
}
