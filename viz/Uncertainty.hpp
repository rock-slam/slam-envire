#ifndef __ENVIRE_VIZ_UNCERTAINTY__
#define __ENVIRE_VIZ_UNCERTAINTY__

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <osg/PositionAttitudeTransform>
#include <Eigen/Core>

namespace vizkit
{

class Uncertainty : public osg::PositionAttitudeTransform
{
public:
    Uncertainty();

    void setMean( const Eigen::Vector3d& mean );
    void setCovariance( const Eigen::Matrix3d& cov );

    void showSamples() { m_showSamples = true; }
    void hideSamples() { m_showSamples = false; }

    void setNumSamples(size_t samples) { num_samples = samples; }

private:
    /** random number generator */
    boost::minstd_rand rand_gen;

    bool m_showSamples;
    osg::ref_ptr<osg::Geode> geode;

    size_t num_samples;
};

}

#endif
