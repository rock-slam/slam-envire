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
     * and placing it relative to the active grid or relative to the mlsmap
     * @param trans - the transform between the active grid and the newly
     *                created grid
     * @param relative - if true, the transform is considered relative to 
     *	                 the previous grid
     * @param aligned - if true, the grid is aligned to to the cells of the 
     *                  currently active grid
     */
    void createGrid( const Transform& trans, bool relative = true, bool aligned = true );

    /** 
     * based on the transform this function will set the active grid to a
     * previous grid, which is within range, or create a new grid. In both
     * cases, the resulting grid is made active.
     *
     * @param fn - center of this framenode is used for distance measurement 
     * @param double threshold - if t is within threshold of grid center, that
     *        grid is made active
     */
    void selectActiveGrid( const FrameNode* fn, double threshold, bool aligned = true );

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
     public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Cache() : grid( NULL ) {}
	MLSGrid* grid;
	Transform trans;
    };

    Cache cache;
};

}

#endif
