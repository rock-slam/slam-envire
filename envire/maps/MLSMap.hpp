#ifndef __ENVIRE_MAPS_MLSMAP_HPP__
#define __ENVIRE_MAPS_MLSMAP_HPP__

#include <envire/maps/MultiLevelSurfaceGrid.hpp>

namespace envire
{

class MLSMap : Map<2>
{
    ENVIRONMENT_ITEM( MLSMap )

public:
    typedef envire::MultiLevelSurfaceGrid::SurfacePatch SurfacePatch;

    MLSMap();
    MLSMap(const MLSMap& other);
    MLSMap(Serialization& so);

    MLSMap& operator=(const MLSMap& other);

    Eigen::AlignedBox<double, 2> getExtents() const;

public:
    SurfacePatch* get( const Point& p, const SurfacePatch& patch, double sigma_threshold = 3.0 );
    void updateCell( size_t m, size_t n, const SurfacePatch& patch );

protected:
    std::vector<MultiLevelSurfaceGrid::Ptr> grids;

};

}

#endif
