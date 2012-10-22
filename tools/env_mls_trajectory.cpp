#include <Eigen/Geometry>

#include "envire/Core.hpp"
#include "envire/maps/TriMesh.hpp"
#include "envire/maps/MLSGrid.hpp"

#include "boost/scoped_ptr.hpp"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace envire;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 2 ) 
    {
	std::cout << "usage: env_mls input trajectory" << std::endl;
	exit(0);
    }
    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));

    // get all mls in environment
    std::vector<MLSGrid*> items = env->getItems<MLSGrid>();
    
    // get trajectory
    std::ifstream file( argv[2] );
    
    while( !file.eof() )
    {
	double x,y,z;
	file >> x >> y >> z; 
	Eigen::Vector3d p( x, y, z );

	for( std::vector<MLSGrid*>::iterator it=items.begin(); it != items.end(); it++ )
	{
	    MLSGrid *mls = *it;
	    double top = -1e9;

	    // point in map
	    Eigen::Vector3d mp = mls->toMap( p );
	    MLSGrid::Position pos;
	    if( mls->toGrid( mp.head<2>(), pos ) )
	    {
		MLSGrid::iterator gi = mls->beginCell( pos.x, pos.y );
		while( gi != mls->endCell() )
		{
		    if( gi->mean > top )
		    {
			top = gi->mean;
		    }
		    gi++;
		}
	    }

	    std::cout << top << " ";
	}
	std::cout << std::endl;
    }
}
