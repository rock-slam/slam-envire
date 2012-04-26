#include "envire/Core.hpp"
#include "envire/maps/Pointcloud.hpp"

#include "boost/scoped_ptr.hpp"
#include <boost/version.hpp>

#include <fstream>
#include <iostream>

using namespace envire;
using namespace std;
using boost::lexical_cast;
     
void usage()
{
    std::cout << "usage: asc2ply asc_file ply_file [sampling]" << std::endl;
}

int main( int argc, char* argv[] )
{
    if( argc < 3 ) 
    {
	usage();
	exit(0);
    }

    string asc_file = argv[1];
    string ply_file = argv[2];
    int sampling = 1;
    if( argc >= 4 )
	sampling = lexical_cast<int>( argv[3] );

    Pointcloud::Ptr pc = new Pointcloud();
    ifstream asc( asc_file.c_str() );
    pc->readText( asc, sampling, Pointcloud::XYZR );

    ofstream ply( ply_file.c_str() );
    pc->writePly( ply_file, ply );
}

