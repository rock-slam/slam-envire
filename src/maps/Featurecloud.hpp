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

    public:
	Featurecloud();
	Featurecloud(Serialization& so);
	void serialize(Serialization& so);

	std::vector<KeyPoint> keypoints;
	std::vector<Scalar> descriptors;

	DESCRIPTOR descriptorType; 
	int descriptorSize;

	size_t size() const { return vertices.size(); }

	/** copy the content of the source feature cloud
	 * and transform to new frame if necessary
	 */
	void copyFrom( Featurecloud* source, bool transform = true );

	void clear();
    };
}

#endif
