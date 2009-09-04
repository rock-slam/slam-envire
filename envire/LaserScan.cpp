#include "LaserScan.hpp"

#include <string>
#include <fstream>

#include <stdexcept>

using namespace envire;
using namespace std;

const std::string LaserScan::className = "envire::LaserScan";

LaserScan::LaserScan()
{
}

LaserScan::LaserScan(Serialization& so) :
    CartesianMap(so)
{
    so.setClassName(className);
    readScan( getMapFileName(so.getMapPath()) );
}

void LaserScan::serialize(Serialization& so)
{
    CartesianMap::serialize(so);

    so.setClassName(className);
    writeScan( getMapFileName(so.getMapPath()) );
}

LaserScan* LaserScan::importScanFile(const std::string& file, FrameNode* node)
{
    if( !node->isAttached() )
	throw std::runtime_error("node needs to be attached to environment");

    Environment* env = node->getEnvironment();
    LaserScan* scan = new LaserScan();

    FrameNode::TransformType transform( Eigen::Matrix4f::Identity() );

    try {
	scan->parseScan( file, transform );
    }
    catch( ... )
    {
	delete scan;
	throw;
    }

    env->attachItem( scan );

    if( transform.matrix() != Eigen::Matrix4f::Identity() )
    {
	FrameNode* fm = new FrameNode();
	fm->setTransform( transform );
	env->addChild( node, fm );
	scan->setFrameNode( fm );
    }
    else 
    {
	scan->setFrameNode( node );
    }

    return scan;
}


bool LaserScan::parseScan( const std::string& file, FrameNode::TransformType& transform ) {
    std::ifstream data(file.c_str());
    if( data.fail() )  
    {
        throw std::runtime_error("Could not open file '" + file + "'.");
    }

    std::string line;
    while( !data.eof() ) {
        getline( data, line );
        std::istringstream iline( line );

        std::string key;
        iline >> key;

        if( key == "orientation" ) {
            float x, u, v, w;
            iline >> x >> u >> v >> w;
            transform.rotate( Eigen::Quaternionf(x, u, v, w) );
        }

        if( key == "origin" ) {
            float x,y,z;
            iline >> x >> y >> z;
            transform.pretranslate( Eigen::Vector3f(x,y,z) );
        }

        if( key == "delta_psi" ) {
            iline >> delta_psi;
        }

        if( key == "origin_psi" ) {
            iline >> origin_psi;
        }

        if( key == "origin_phi" ) {
            iline >> origin_phi;
        }

        if( key == "center_offset" ) {
            iline >> center_offset.x() >> center_offset.y() >> center_offset.z();
        }

        if( key == "line" ) {
            scanline_t scanline;
            iline >> scanline.first;

            while( !iline.eof() ) {
                int dist;
                iline >> dist;
                scanline.second.push_back( dist );
            }

            points_per_line = scanline.second.size();

            lines.push_back( scanline );
        }
    }

    data.close();

    return true;
}


bool LaserScan::readScan( const std::string& file ) 
{
    FrameNode::TransformType t;
    return parseScan( file, t );
}

bool LaserScan::writeScan( const std::string& file )
{
    std::ofstream data(file.c_str());
    if( data.fail() )  
    {
        throw std::runtime_error("Could not open file '" + file + "' for writing.");
    }

    data << "delta_psi " << delta_psi << endl;
    data << "origin_psi " << origin_psi << endl;
    data << "origin_phi " << origin_phi << endl;
    data << "center_offset " 
	<< center_offset.x() << " " 
	<< center_offset.y() << " " 
	<< center_offset.z() << endl;

    for( vector<scanline_t>::iterator it=lines.begin();it!=lines.end();it++)
    {
	data << "line " << (*it).first;
	for( vector<int>::iterator pi=(*it).second.begin();pi!=(*it).second.end();pi++)
	{
	    data << " " << (*pi);
	}
	data << endl;
    }	

    data.close();

    return true;
}

LaserScan* LaserScan::clone() 
{
    return new LaserScan(*this);
}

