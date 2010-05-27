#ifndef __ENVIRE_GRIDACCESS__
#define __ENVIRE_GRIDACCESS__

#include "Core.hpp"

namespace envire
{
    class GridAccess
    {
    public:
	GridAccess(Environment* env);

	/** augment the position vector with the elevation part of an elevation map found at the relevant coordinates.  
	 */
	bool getElevation(Eigen::Vector3d& position);

    private:
	Environment *env;
    };
}

#endif
