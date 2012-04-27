#include "envire/Core.hpp"
#include "envire/maps/Pointcloud.hpp"
#ifdef ENVIRE_USE_CGAL
#include "envire/operators/SimplifyPointcloud.hpp"
#endif

#include "boost/scoped_ptr.hpp"
#include <boost/version.hpp>

#include <fstream>
#include <iostream>

using namespace envire;
using namespace std;
using boost::lexical_cast;
     
void usage()
{
    std::cout << "usage: asc2ply asc_file ply_file [sampling] [cell_size]" << std::endl;
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
    float cell_size = 0;
    if( argc >= 4 )
	sampling = lexical_cast<int>( argv[3] );
    if( argc >= 5 )
	cell_size = lexical_cast<float>( argv[4] );

    Environment env;

    Pointcloud::Ptr pc = new Pointcloud();
    env.attachItem( pc.get() );
    env.setFrameNode( pc.get(), env.getRootNode() );
    ifstream asc( asc_file.c_str() );
    pc->readText( asc, sampling, Pointcloud::XYZR );

    if( cell_size > 0 )
    {
#ifdef ENVIRE_USE_CGAL
	Pointcloud::Ptr pc_simp = new Pointcloud();
	env.attachItem( pc_simp.get() );
	env.setFrameNode( pc_simp.get(), env.getRootNode() );

	SimplifyPointcloud::Ptr spc = new SimplifyPointcloud();
	env.attachItem( spc.get() );
	spc->addInput( pc.get() );
	spc->addOutput( pc_simp.get() );
	spc->setSimplifyCellSize( cell_size );

	spc->updateAll();

	pc = pc_simp;
#else
	std::cout << "ignoring cell_size parameter since CGAL is not compiled in." << std::endl;
#endif
    }

    ofstream ply( ply_file.c_str() );
    pc->writePly( ply_file, ply );
}

