#ifndef __ENVIRE_FEATURES_HPP__
#define __ENVIRE_FEATURES_HPP__

#include <base/eigen.h>

namespace envire
{

enum DESCRIPTOR
{
    DESCRIPTOR_SURF = 1,
    DESCRIPTOR_PSURF = 2,
};

struct KeyPoint
{
    typedef float Scalar;

    /** the original image point */
    base::Vector2d point;

    /** scaled size of the feature */
    Scalar size;

    Scalar angle;
    Scalar response;
};

}

#endif
