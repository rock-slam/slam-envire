#include "LaserScan.hpp"

#include <string>
#include <fstream>

#include <stdexcept>
#include <iostream>

using namespace envire;
using namespace std;

ENVIRONMENT_ITEM_DEF( LaserScan )

LaserScan::LaserScan()
    : origin_phi(0), center_offset( Eigen::Vector3d::Zero() )
{
}

LaserScan::LaserScan(Serialization& so) :
    Map<3>(so)
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

void LaserScan::addScanLine( double tilt_angle, const base::samples::LaserScan& scan )
{
    if( lines.empty() )
    {
	// set the scan properties
	origin_psi = scan.start_angle;
	delta_psi = scan.angular_resolution;	
	points_per_line = scan.ranges.size();
    }
    else
    {
	// check scanline properties
	if( origin_psi != scan.start_angle )
	    std::cerr << "[envire] WARNING: laser start angle mismatch" << std::endl;
	if( delta_psi != scan.angular_resolution )
	    std::cerr << "[envire] WARNING: laser angular resolution mismatch" << std::endl;
	if( static_cast<unsigned int>(points_per_line) != scan.ranges.size() )
	    std::cerr << "[envire] WARNING: laser points per line mismatch" << std::endl; 
    }

    // copy the scanline
    scanline_t line;
    line.delta_phi = tilt_angle;
    std::copy( scan.ranges.begin(), scan.ranges.end(), std::back_inserter( line.ranges ) );
    std::copy( scan.remission.begin(), scan.remission.end(), std::back_inserter( line.remissions ) );

    lines.push_back( line );
}

LaserScan* LaserScan::importScanFile(const std::string& file, FrameNode* node)
{
    if( !node->isAttached() )
	throw std::runtime_error("node needs to be attached to environment");

    Environment* env = node->getEnvironment();
    LaserScan* scan = new LaserScan();

    FrameNode::TransformType transform( Eigen::Matrix4d::Identity() );

    try {
	scan->parseScan( file, transform );
    }
    catch( ... )
    {
	delete scan;
	throw;
    }

    env->attachItem( scan );
    
    FrameNode* fm = new FrameNode();
    env->addChild( node, fm );
    scan->setFrameNode( fm );
    
    fm->setTransform( transform );

    return scan;
}

const std::string LaserScan::getMapFileName(const std::string& path) const
{
    return Layer::getMapFileName(path) + ".scan";
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
            transform.rotate( Eigen::Quaterniond(x, u, v, w) );
        }

        if( key == "origin" ) {
            float x,y,z;
            iline >> x >> y >> z;
            transform.pretranslate( Eigen::Vector3d(x,y,z) );
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
            iline >> scanline.delta_phi;

            while( !iline.eof() ) {
                int dist;
                iline >> dist;
                scanline.ranges.push_back( dist );
            }

            points_per_line = scanline.ranges.size();

            lines.push_back( scanline );
        }

        if( key == "remission" ) {
	    if( lines.size() == 0 )
	    {
		std::cerr << "remission line without prior line statement." << std::endl;
		return false;
	    }

            scanline_t& scanline(lines.back());
	    float dphi;
            iline >> dphi;

	    if( dphi != scanline.delta_phi )
	    {
		std::cerr << "remission line does not have the same delta_phi value as prior line statement." << std::endl;
		return false;
	    }

            while( !iline.eof() ) {
                float rem;
                iline >> rem;
                scanline.remissions.push_back( rem );
            }

            points_per_line = scanline.remissions.size();
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
	data << "line " << (*it).delta_phi;
	for( vector<unsigned int>::iterator pi=(*it).ranges.begin();pi!=(*it).ranges.end();pi++)
	{
	    data << " " << (*pi);
	}
	data << endl;

	if( (*it).ranges.size() == (*it).remissions.size() )
	{
	    data << "remission " << (*it).delta_phi;
	    for( vector<unsigned int>::iterator pi=(*it).remissions.begin();pi!=(*it).remissions.end();pi++)
	    {
		data << " " << (*pi);
	    }
	    data << endl;
	}
    }	

    data.close();

    return true;
}

LaserScan::Extents LaserScan::getExtents() const
{
    // TODO provide proper extents
    return Extents(); 
}
