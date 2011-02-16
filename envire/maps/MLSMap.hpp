#ifndef __ENVIRE_MAPS_MLSMAP_HPP__
#define __ENVIRE_MAPS_MLSMAP_HPP__

#include <envire/maps/MultiLevelSurfaceGrid.hpp>

namespace envire
{

class MLSMap : public Map<2>
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
    /** get a patch from the stored grids. @param p is in the coordinate from of this map. */
    SurfacePatch* getPatch( const Point& p, const SurfacePatch& patch, double sigma_threshold = 3.0 );

    /** 
     * add new grid and make it active. The grid is assumed to be 
     * attached to the environment. 
     * @param grid - to be added to the map
     */
    void addGrid( MultiLevelSurfaceGrid::Ptr grid );

    /** add a new grid taking the current active grid as a template
     * and placing it relative to the active grid 
     * @param trans - the transform between the active grid and the newly
     *                created grid
     */
    void createGrid( const Transform& trans );

    /** @return the currently active grid
     */
    MultiLevelSurfaceGrid::Ptr getActiveGrid() const { return active; }

    /** @return a deep clone of the object,
     * which wil also clone the references to the children.
     */
    MLSMap* cloneDeep();

public:
    // TODO we only store pointer to the grids here,
    // there is a little problem here, since we actually should
    // also add a framenode per grid
    std::vector<MultiLevelSurfaceGrid::Ptr> grids;
    MultiLevelSurfaceGrid::Ptr active;
};

}

#endif
