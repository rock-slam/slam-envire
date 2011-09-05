#ifndef __ENVIRE_FEATURES_HPP__
#define __ENVIRE_FEATURES_HPP__

namespace envire
{

enum DESCRIPTOR
{
    DESCRIPTOR_SURF = 1,
};

struct KeyPoint
{
    typedef float Scalar;

    Scalar size;
    Scalar angle;
    Scalar response;
};

}

#endif
