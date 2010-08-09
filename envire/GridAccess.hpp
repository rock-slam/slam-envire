#ifndef __ENVIRE_GRIDACCESS__
#define __ENVIRE_GRIDACCESS__

#include "Core.hpp"
#include <boost/shared_ptr.hpp>


namespace envire
{
    class GridAccess
    {
    public:
	GridAccess(Environment* env);

	/** augment the position vector with the elevation part of an elevation
	 * map found at the relevant coordinates.  This method does some
	 * caching of the envire structure. The current implementation and
	 * hence will not work well with dynamically changing environment.
	 */
	bool getElevation(Eigen::Vector3d& position);

    private:
	struct GridAccessImpl;
	boost::shared_ptr<GridAccessImpl> impl;
    };

    class PointcloudAccess
    {
    public:
	PointcloudAccess(Environment* env);

	bool getElevation(Eigen::Vector3d& position, double threshold = 0.05);

    private:
	struct PointcloudAccessImpl;
	boost::shared_ptr<PointcloudAccessImpl> impl;
    };
}

#endif
