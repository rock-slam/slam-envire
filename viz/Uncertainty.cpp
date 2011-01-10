#include "Uncertainty.hpp"
#include <boost/random/normal_distribution.hpp>

#include <osg/Group>
#include <osg/Geode>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/Drawable>
#include <osg/ShapeDrawable>

#include <Eigen/Cholesky>

using namespace vizkit;

Uncertainty::Uncertainty()
    : rand_gen(42u), m_showSamples( true ), num_samples( 500 )
{
    geode = new osg::Geode();
    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    addChild( geode );
}

void Uncertainty::setMean( const Eigen::Vector3d& mean )
{
    setPosition( osg::Vec3( mean.x(), mean.y(), mean.z() ) );
}

void Uncertainty::setCovariance( const Eigen::Matrix3d& cov )
{
    //remove old drawables
    while(geode->removeDrawables(0));

    if( m_showSamples )
    {
	const size_t max_samples = num_samples;
	// create a new geometry object with the distribution give in the covariance
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1,1,1,1));
	geom->setColorArray(color.get());
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > 
	    rand(rand_gen, boost::normal_distribution<>(0,1.0) );

	Eigen::Matrix3d covLT = cov.llt().matrixL();

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	for(size_t i=0; i<max_samples; i++) 
	{
	    Eigen::Vector3d n, p;
	    for(int i=0;i<3;i++)
		n[i] = rand();

	    p = covLT * n;
	    vertices->push_back(osg::Vec3(p.x(), p.y(), p.z()));
	}
	geom->setVertexArray(vertices);

	osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, vertices->size() );
	geom->addPrimitiveSet(drawArrays.get());

	geode->addDrawable(geom.get());    
    }
}
