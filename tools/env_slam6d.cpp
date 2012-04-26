#include "envire/Core.hpp"
#include "envire/maps/Pointcloud.hpp"

#include "boost/scoped_ptr.hpp"
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace envire;
using namespace std;
using namespace boost::filesystem;
using boost::format;
using boost::str;
using boost::regex;
using boost::regex_match;
using boost::smatch;
using boost::lexical_cast;
     
void usage()
{
    std::cout << "usage: env_slam6d <generate|update> env_dir slam6d_dat" << std::endl;
}

bool comp( const Pointcloud *pc1, const Pointcloud *pc2 )
{
    return pc1->getUniqueIdNumericalSuffix() < pc2->getUniqueIdNumericalSuffix();
}

int main( int argc, char* argv[] )
{
    if( argc < 4 ) 
    {
	usage();
	exit(0);
    }

    string cmd = argv[1];
    string env_dat = argv[2];
    string slam6d_dat = argv[3];

    // Transform from envire to slam6d coordinate system
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    S(0,1) = S(1,2) = S(2,0) = 1.0;

    if( cmd == "generate" )
    {
	// this will generate a slam6d directory based on an environment
	boost::scoped_ptr<Environment> env( Environment::unserialize( env_dat ) );

	path env_dir = env_dat;
	path slam6d_dir = slam6d_dat;

	// create directories if necessary
	if( !is_directory( slam6d_dir ) )
	    create_directories( slam6d_dir );

	vector<Pointcloud*> pcs = env->getItems<Pointcloud>();
	try
	{
	    sort( pcs.begin(), pcs.end(), comp );
	}
	catch( exception e )
	{
	    cout << "Could not sort by numerical id." << endl;
	}

	size_t map_idx = 0;
	for( vector<Pointcloud*>::iterator it = pcs.begin(); it != pcs.end(); it++ )
	{
	    Pointcloud *pc = *it;
	    FrameNode *fn = pc->getFrameNode();

	    // create symbolic link
	    string scan_prefix = str(format("scan%03d") % map_idx);
#if BOOST_VERSION >= 104600
	    create_symlink( absolute(env_dir) / (pc->getMapFileName() + ".ply"), slam6d_dir / (scan_prefix + ".ply") );
#else
	    create_symlink( env_dir / (pc->getMapFileName() + ".ply"), slam6d_dir / (scan_prefix + ".ply") );
#endif

	    // create pose file
	    ofstream pose_stream( (slam6d_dir / (scan_prefix + ".pose")).string().c_str() );
	    pose_stream << (S*fn->getTransform().translation()).transpose() << endl;
	    pose_stream << ((S*fn->getTransform().linear()*S.transpose()).eulerAngles(0,1,2)).transpose() << endl;

	    // create id file
	    ofstream id_stream( (slam6d_dir / (scan_prefix + ".id")).string().c_str() );
	    id_stream << pc->getUniqueId() << endl;

	    map_idx++;
	}
    }
    else if( cmd == "update" )
    {
	// this will generate a slam6d directory based on an environment
	boost::scoped_ptr<Environment> env( Environment::unserialize( env_dat ) );

	path slam6d_dir = slam6d_dat;
	regex rx("scan(\\d{3})\\.frames");
	smatch r;
	for( directory_iterator it( slam6d_dir ); it != directory_iterator(); it++ )
	{
	    string file_name = it->path().string();
	    if( regex_search( file_name, r, rx ) )
	    {
		string idx_str( r[1].first, r[1].second );
		size_t map_idx = lexical_cast<size_t>( idx_str );
		string scan_prefix = str(format("scan%03d") % map_idx);

		// load transforms
		ifstream frame_stream( (slam6d_dir / (scan_prefix + ".frames")).string().c_str() );
		Eigen::Affine3d trans;
		string line;
		while( getline( frame_stream, line ) )
		{
		    stringstream ss( line );
		    for( int i=0; i<16; i++)
			ss >> trans.matrix()(i);
		}

		// convert coordinate system from slam6d
		trans.translation() = S.transpose() * trans.translation();
		trans.linear() = S.transpose() * trans.linear() * S;

		// load id file 
		ifstream id_stream( (slam6d_dir / (scan_prefix + ".id")).string().c_str() );
		string unique_id;
		id_stream >> unique_id;

		// write transformation to framenode
		Pointcloud* pc = env->getItem<Pointcloud>( unique_id ).get();
		pc->getFrameNode()->setTransform( trans );
		
		cout << "updated pointcloud " << unique_id << endl;
	    }

	};

	// write environment back
	env->serialize( env_dat );
    }
    else
	usage();
}

