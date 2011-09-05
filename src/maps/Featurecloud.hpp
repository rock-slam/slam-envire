#ifndef __ENVIRE_FEATURECLOUD_HPP__
#define __ENVIRE_FEATURECLOUD_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/core/Features.hpp>

namespace envire
{
    class Featurecloud : public Pointcloud 
    {
	ENVIRONMENT_ITEM( Featurecloud )

	typedef float Scalar;
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::DontAlign> Descriptor;

    public:
	Featurecloud();
	Featurecloud(Serialization& so);
	void serialize(Serialization& so);

	std::vector<KeyPoint> keypoints;
	std::vector<Descriptor> descriptors;

	void clear();
    };
}

#endif
