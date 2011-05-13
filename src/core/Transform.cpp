#include "Core.hpp"
#include <Eigen/LU>
#include <Eigen/Geometry>

using namespace envire;

PointWithUncertainty::PointWithUncertainty() {}
PointWithUncertainty::PointWithUncertainty( const Point& point, const Covariance& cov ) 
    : point( point ), cov( cov ) {}

TransformWithUncertainty::TransformWithUncertainty() {}
TransformWithUncertainty::TransformWithUncertainty( const Transform& trans )
    : trans( trans ), cov( Covariance::Zero() ), uncertain(false) {}
TransformWithUncertainty::TransformWithUncertainty( const Transform& trans, const Covariance& cov )
    : trans( trans ), cov( cov ), uncertain(true) {}

// The uncertainty transformations are implemented according to: 
// Pennec X, Thirion JP. A framework for uncertainty and validation of 3-D
// registration methods based on points and frames. International Journal of
// Computer Vision. 1997;25(3):203–229. Available at:
// http://www.springerlink.com/index/JJ25N2Q23T402682.pdf.

Eigen::Quaterniond r_to_q( const Eigen::Vector3d& r )
{
    double theta = r.norm();
    if( fabs(theta) > 1e-5 )
	return Eigen::Quaterniond( Eigen::AngleAxisd( theta, r/theta ) );
    else
	return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d q_to_r( const Eigen::Quaterniond& q )
{
    Eigen::AngleAxisd aa( q );
    return aa.axis() * aa.angle();
}

inline double sign( double v )
{
    return v > 0 ? 1.0 : -1.0;
}

Eigen::Matrix<double,3,3> skew_symmetric( const Eigen::Vector3d& r )
{
    Eigen::Matrix3d res;
    res << 0, -r.z(), r.y(),
	r.z(), 0, -r.x(),
	-r.y(), r.x(), 0;
    return res;
}

Eigen::Matrix<double,4,3> dq_by_dr( const Eigen::Quaterniond& q )
{
    const Eigen::Vector3d r( q_to_r( q ) );

    const double theta = r.norm();
    const double kappa = 0.5 - theta*theta / 48.0; // approx. see Paper 
    const double lambda = 1.0/24.0*(1.0-theta*theta/40.0); // approx.
    Eigen::Matrix<double,4,3> res;
    res << - q.vec().transpose()/2.0,
       kappa * Eigen::Matrix3d::Identity() - lambda * r * r.transpose(); 	

    return res;
}

Eigen::Matrix<double,3,4> dr_by_dq( const Eigen::Quaterniond& q )
{
    const Eigen::Vector3d r( q_to_r( q ) );
    const double mu = q.vec().norm();
    const double tau = 2.0 * sign( q.w() ) * ( 1.0 + mu*mu/6.0 ); // approx
    const double nu = -2.0 * sign( q.w() ) * ( 2.0/3.0 + mu*mu/5.0 ); // approx

    Eigen::Matrix<double,3,4> res;
    res << -2*q.vec(), tau * Eigen::Matrix3d::Identity() + nu * q.vec() * q.vec().transpose();

    return res;
}

Eigen::Matrix<double,4,4> dq2q1_by_dq1( const Eigen::Quaterniond& q2 )
{
    Eigen::Matrix<double,4,4> res;
    res << 0, q2.vec().transpose(),
	q2.vec(), skew_symmetric( q2.vec() );
    return Eigen::Matrix<double,4,4>::Identity() * q2.w() + res;
}

Eigen::Matrix<double,4,4> dq2q1_by_dq2( const Eigen::Quaterniond& q1 )
{
    Eigen::Matrix<double,4,4> res;
    res << 0, q1.vec().transpose(),
	q1.vec(), -skew_symmetric( q1.vec() );
    return Eigen::Matrix<double,4,4>::Identity() * q1.w() + res;
}

Eigen::Matrix<double,3,3> dr2r1_by_r1( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 )
{
    return Eigen::Matrix3d(
	    dr_by_dq( q )
	    * dq2q1_by_dq1( q2 )
	    * dq_by_dr( q1 ) );
}

Eigen::Matrix<double,3,3> dr2r1_by_r2( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 )
{
    return Eigen::Matrix3d(
	    dr_by_dq( q )
	    * dq2q1_by_dq2( q1 )
	    * dq_by_dr( q2 ) );
}

Eigen::Matrix<double,3,3> drx_by_dr( const Eigen::Quaterniond& q, const Eigen::Vector3d& x )
{
    const Eigen::Vector3d r( q_to_r( q ) );
    const double theta = r.norm();
    const double alpha = 1.0 - theta*theta/6.0;
    const double beta = 0.5 - theta*theta/24.0;
    const double gamma = 1.0 / 3.0 - theta*theta/30.0;
    const double delta = -1.0 / 12.0 + theta*theta/180.0;

    return Eigen::Matrix3d(
	    -skew_symmetric(x)*(gamma*r*r.transpose()
		- beta*skew_symmetric(r)+alpha*Eigen::Matrix3d::Identity())
	    -skew_symmetric(r)*skew_symmetric(x)*(delta*r*r.transpose() 
		+ 2.0*beta*Eigen::Matrix3d::Identity()) );
}

TransformWithUncertainty TransformWithUncertainty::operator*( const TransformWithUncertainty& t1 ) const
{
    const TransformWithUncertainty &t2(*this);
    // short path if there is no uncertainty 
    if( !t1.hasUncertainty() && !t2.hasUncertainty() )
	return TransformWithUncertainty( t2.getTransform() * t1.getTransform() );

    // convert the rotations of the respective transforms into quaternions
    Eigen::Quaterniond 
	q1( t1.getTransform().linear() ),
	q2( t2.getTransform().linear() );
    Eigen::Quaterniond q( q2 * q1 );

    // calculate the Jacobians (this is what all the above functions are for)
    Eigen::Matrix<double,6,6> J1, J2;
    J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
       Eigen::Matrix3d::Zero(), t2.getTransform().linear();

    J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
       drx_by_dr(q, t1.getTransform().translation()), t2.getTransform().linear();

    // and calculate the resulting uncertainty transform
    return TransformWithUncertainty( 
	    t2.getTransform() * t1.getTransform(),
	    J1*t1.getCovariance()*J1.transpose() + J2*t2.getCovariance()*J2.transpose() );
}

PointWithUncertainty TransformWithUncertainty::operator*( const PointWithUncertainty& point ) const
{
    Eigen::Matrix3d R( getTransform().linear() );
    Eigen::Quaterniond q( R );
    Eigen::Matrix<double,3,6> J;
    J << drx_by_dr( q, point.getPoint() ), Eigen::Matrix3d::Identity();
    
    return PointWithUncertainty( 
	    getTransform() * point.getPoint(),
	    J*getCovariance()*J.transpose() + R*point.getCovariance()*R.transpose() );
}

TransformWithUncertainty TransformWithUncertainty::inverse( Eigen::TransformTraits traits ) const
{
    // short path if there is no uncertainty 
    if( !hasUncertainty() )
	return TransformWithUncertainty( Transform( getTransform().inverse( traits ) ) );

    Eigen::Quaterniond q( getTransform().linear() );
    Eigen::Vector3d t( getTransform().translation() );
    Eigen::Matrix<double,6,6> J;
    J << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
	drx_by_dr( q.inverse(), t ), q.toRotationMatrix().transpose();

    return TransformWithUncertainty(
	    Eigen::Affine3d( getTransform().inverse( traits ) ),
	    J*getCovariance()*J.transpose() );
}
