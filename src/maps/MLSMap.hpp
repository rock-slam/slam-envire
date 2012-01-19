#ifndef __ENVIRE_MAPS_MLSMAP_HPP__
#define __ENVIRE_MAPS_MLSMAP_HPP__

#include <envire/maps/MLSGrid.hpp>
#include <envire/core/Serialization.hpp>

namespace envire
{

class MLSMap : public Map<2>
{
    ENVIRONMENT_ITEM( MLSMap )

public:
    typedef envire::MLSGrid::SurfacePatch SurfacePatch;

    MLSMap();
    MLSMap(const MLSMap& other);
    MLSMap(Serialization& so);

    MLSMap& operator=(const MLSMap& other);

    Eigen::AlignedBox<double, 2> getExtents() const;

public:
    /** get a patch from the stored grids. @param p is in the coordinate from of this map. */
    bool getPatch( const Point& p, SurfacePatch& patch, double sigma_threshold = 3.0 );

    /** 
     * add new grid and make it active. The grid is assumed to be 
     * attached to the environment. 
     * @param grid - to be added to the map
     */
    void addGrid( MLSGrid::Ptr grid );

    /** add a new grid taking the current active grid as a template
     * and placing it relative to the active grid 
     * @param trans - the transform between the active grid and the newly
     *                created grid
     */
    void createGrid( const Transform& trans );

    /** @return the currently active grid
     */
    MLSGrid::Ptr getActiveGrid() const { return active; }

    std::vector<MLSGrid::Ptr>& getGrids() { return grids; }

    /** @return a deep clone of the object,
     * which wil also clone the references to the children.
     */
    MLSMap* cloneDeep();

public:
    // TODO we only store pointer to the grids here,
    // there is a little problem here, since we actually should
    // also add a framenode per grid
    std::vector<MLSGrid::Ptr> grids;
    MLSGrid::Ptr active;

protected:
    struct Cache
    {
	Cache() : grid( NULL ) {}
	MLSGrid* grid;
	Transform trans;
    };

    Cache cache;
};

}

#endif
