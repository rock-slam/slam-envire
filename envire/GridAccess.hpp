#ifndef __ENVIRE_GRIDACCESS__
#define __ENVIRE_GRIDACCESS__

#include "Core.hpp"

class GridAccessImpl;

namespace envire
{
    class GridAccess
    {
    public:
	GridAccess(Environment* env);
	~GridAccess();

	/** augment the position vector with the elevation part of an elevation
	 * map found at the relevant coordinates.  This method does some
	 * caching of the envire structure. The current implementation and
	 * hence will not work well with dynamically changing environment.
	 */
	bool getElevation(Eigen::Vector3d& position);

    private:
	Environment *env;
	GridAccessImpl* impl;
    };
}

#endif
