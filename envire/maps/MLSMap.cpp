#include "MLSMap.hpp"

using namespace envire;

Eigen::AlignedBox<double, 2> MLSMap::getExtents() const 
{ 
    throw std::runtime_error("not implemented"); 
}

MLSMap::MLSMap(const MLSMap& other)
{
    // TODO implement
}

MLSMap& MLSMap::operator=(const MLSMap& other)
{
    // TODO implement
    return *this;
}
