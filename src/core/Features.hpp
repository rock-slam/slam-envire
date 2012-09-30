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

class KeyPoint
{
public:
    typedef float Scalar;

    /** the original image point */
    base::Vector2d point;

    /** scaled size of the feature */
    Scalar size;

    Scalar angle;
    Scalar response;
    void store(std::ostream& os) const
    {
      os << point[0] << " " << point[1] << " ";
      os << size << " ";
      os << angle << " ";
      os << response << " ";
    }
    void load(std::istream& is)
    {
      double d1, d2;
      is >> d1;
      is >> d2;
      point = base::Vector2d(d1, d2);
      is >> size;
      is >> angle;
      is >> response;
    }

};

}

#endif
