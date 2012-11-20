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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef float Scalar;

    /** the original image point */
    base::Vector2d point;

    /** scaled size of the feature */
    Scalar size;

    Scalar angle;
    Scalar response;
    void store(std::ostream& os) const
    {
      os << point[0] << " " << point[1] << "\n";
      os << size << "\n";
      os << angle << "\n";
      os << response << "\n";
    }
    void load(std::istream& is)
    {
      double d1, d2;
      is >> d1;
      is >> d2;
      is.ignore(10, '\n');
      point = base::Vector2d(d1, d2);
      is >> size;
      is.ignore(10, '\n');
      is >> angle;
      is.ignore(10, '\n');
      is >> response;
      is.ignore(10, '\n');
    }

};

}

#endif
