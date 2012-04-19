#ifndef __ENVIRE_CORE_TRANSFORM_HPP__
#define __ENVIRE_CORE_TRANSFORM_HPP__

#include <Eigen/Core>
#include <base/samples/rigid_body_state.h>

namespace envire
{
    typedef Eigen::Vector3d Point;
    
    /** 
     * Represents a point with associated uncertainty.
     *
     * The uncertainty is represented as a 3x3 covariance matrix.
     */
    class PointWithUncertainty
    {
    public:
	typedef Eigen::Matrix<double,3,3> Covariance;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PointWithUncertainty();
	PointWithUncertainty( const Point& point );
	PointWithUncertainty( const Point& point, const Covariance& cov );

	const Covariance& getCovariance() const { return cov; }
	void setCovariance( const Covariance& cov ) { this->cov = cov; uncertain = true; }
	const Point& getPoint() const { return point; }
	void setPoint( const Point& point ) { this->point = point; }

	bool hasUncertainty() const { return uncertain; }

    protected:
	Point point;
	Covariance cov;
	bool uncertain;
    };

    /** 
     * Class which is used to represent a 3D Transform.
     *
     * The transformation is represented as a 4x4 homogenous matrix. Both
     * rotation and translation in 3D are represented.
     */
    typedef Eigen::Affine3d Transform;

    /** 
     * Class which represents a 3D Transform with associated uncertainty information.
     *
     * The uncertainty is represented as a 6x6 matrix, which is the covariance
     * matrix of the [r t] representation of the error. Here r is the rotational
     * part expressed as a scaled axis of rotation, and t the translational
     * component.
     *
     * The uncertainty information is optional. The hasUncertainty() method can
     * be used to see if uncertainty information is associated with the class.
     */
    class TransformWithUncertainty
    {
    public:
	typedef Eigen::Matrix<double,6,6> Covariance;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	TransformWithUncertainty();
	explicit TransformWithUncertainty( const base::samples::RigidBodyState& rbs );
	explicit TransformWithUncertainty( const Transform& trans );
	TransformWithUncertainty( const Transform& trans, const Covariance& cov );

	static TransformWithUncertainty Identity();

	/** performs a composition of this transform with the transform given.
	 * The result is another transform with result = this * trans
	 */
	TransformWithUncertainty composition( const TransformWithUncertainty& trans ) const;

	/** performs an inverse composition of two transformations.
	 * The result is such that result * trans = this. Note that this is different from
	 * calling result = this * inv(trans), in the way the uncertainties are handled.
	 */
	TransformWithUncertainty compositionInv( const TransformWithUncertainty& trans ) const;

	/** Same as compositionInv, just that the trans * result = this.
	 */
	TransformWithUncertainty preCompositionInv( const TransformWithUncertainty& t2 ) const;

	/** alias for the composition of two transforms
	 */
	TransformWithUncertainty operator*( const TransformWithUncertainty& trans ) const;
	PointWithUncertainty operator*( const PointWithUncertainty& point ) const;
	TransformWithUncertainty inverse() const;

	TransformWithUncertainty& operator=( const base::samples::RigidBodyState& rbs );
	void copyToRigidBodyState( base::samples::RigidBodyState& rbs ) const;

	const Covariance& getCovariance() const { return cov; }
	void setCovariance( const Covariance& cov ) { this->cov = cov; uncertain = true; }
	const Transform& getTransform() const { return trans; }
	void setTransform( const Transform& trans ) { this->trans = trans; }

	bool hasUncertainty() const { return uncertain; }

    protected:
	Transform trans;
	Covariance cov;
	bool uncertain;
    };
}

#endif
