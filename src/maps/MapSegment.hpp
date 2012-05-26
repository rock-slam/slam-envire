#ifndef ENVIRE_MAPSEGMENT__
#define ENVIRE_MAPSEGMENT__

#include <envire/Core.hpp>
#include <envire/tools/GaussianMixture.hpp>

namespace envire
{

/** @brief Representation of a map segment, which holds multiple map hypothesis
 */
class MapSegment : public Map<3>
{
    ENVIRONMENT_ITEM( MapSegment )
    friend class MapSegmentVisualization;

public:
    envire::Map<3>::Extents getExtents() const;

public:
    MapSegment();

    /** @brief Add a map and its pose to the list of hypothesis stored by this map segment
     */
    void addPart( const base::Affine3d& pose, CartesianMap* map, double weight, double zVar = 0 );
        
    /** @brief Update the gaussian mixture representation based on the poses in the
     * map segment
     */
    void update();

    /** @brief The transform with covariance for this map segment 
     * 
     * Only valid after a call to update. The transform is from the end of the
     * segment to the origin of the segment.
     *
     * @todo currently we just take the gaussian with the highest weight.
     * Later, we should consider different gaussians, end embed them in the
     * optimization process.
     */
    TransformWithUncertainty getTransform() const;

    /** @brief Provide the stored map, for which the end pose is closest to the
     * provided pose.
     */
    CartesianMap* getMapForPose( const base::Affine3d& pose ) const;

    /** @brief get the best map without any knowledge on a relative transformation
     *
     * @return the map with the highest weight
     */
    CartesianMap* getBestMap() const;

protected:
    struct Part
    {
	/** pointer to map structure holding
	 * a single map hypothesis for this segment
	 */
	CartesianMap::Ptr map;
	/** transform for the final pose of the hypothesis
	 * to the origin of the segment.
	 */
	base::Pose pose;

	/** relative weight of the pose
	 */
	double weight;
    };
    /** 
     * HACK variance of z value of pose
    */
    double zVar;

    typedef GaussianMixture<double, 6> GMM;

    /** gaussian mixture representation of the pose distribution the individual
     * particles provide.
     * The representation is as 3d coordinates and rotation around z axis.
     */
    GMM gmm;

    /** vector of map hypothesis of this segment
     */
    std::vector<Part> parts;

public:
    /** vector of trajectories, which may record the position of parts over time
     */
    std::vector<std::vector<base::Vector3d> > trajectories;
};
}

#endif
