#include "LaserScan.hpp"

#include <string>
#include <fstream>

#include <stdexcept>
#include <iostream>
#include <base-logging/Logging.hpp>

using namespace envire;
using namespace std;

ENVIRONMENT_ITEM_DEF( LaserScan )

LaserScan::LaserScan()
    : origin_phi(0), center_offset( Eigen::Vector3d::Zero() ), x_forward( false )
{
}

void LaserScan::serialize(Serialization& so)
{
    CartesianMap::serialize(so);

    writeScan( so.getBinaryOutputStream(getMapFileName()) );
}

void LaserScan::setXForward()
{
    x_forward = true;
}

void LaserScan::setYForward()
{
    x_forward = false;
}

void LaserScan::unserialize(Serialization& so)
{
    CartesianMap::unserialize(so);
    
    readScan( so.getBinaryInputStream(getMapFileName()) );
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
        { LOG_WARN_S << "laser start angle mismatch"; }
	if( delta_psi != scan.angular_resolution )
        { LOG_WARN_S << "laser angular resolution mismatch"; }
	if( static_cast<unsigned int>(points_per_line) != scan.ranges.size() )
        { LOG_WARN_S << "laser points per line mismatch"; }
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

    Transform transform( Eigen::Matrix4d::Identity() );

    std::ifstream data(file.c_str());
    if( data.fail() )  
    {
        throw std::runtime_error("Could not open file '" + file + "'.");
    }
    
    try {
	scan->parseScan( data, transform );
    }
    catch( ... )
    {
	delete scan;
	throw;
    }
    
    data.close();

    env->attachItem( scan );
    
    FrameNode* fm = new FrameNode();
    env->addChild( node, fm );
    scan->setFrameNode( fm );
    
    fm->setTransform( transform );

    return scan;
}

const std::string LaserScan::getMapFileName() const
{
    return Layer::getMapFileName() + ".scan";
}

bool LaserScan::parseScan( std::istream& is, Transform& transform ) {
    std::string line;
    while( !is.eof() ) {
        getline( is, line );
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
		throw std::runtime_error("remission line without prior line statement.");

            scanline_t& scanline(lines.back());
	    float dphi;
            iline >> dphi;

	    if( dphi != scanline.delta_phi )
		throw std::runtime_error("remission line does not have the same delta_phi value as prior line statement.");

            while( !iline.eof() ) {
                float rem;
                iline >> rem;
                scanline.remissions.push_back( rem );
            }

            points_per_line = scanline.remissions.size();
        }
    }

    return true;
}


bool LaserScan::readScan( std::istream& is ) 
{
    Transform t;
    return parseScan( is, t );
}

bool LaserScan::writeScan( std::ostream& os )
{
    os << "delta_psi " << delta_psi << endl;
    os << "origin_psi " << origin_psi << endl;
    os << "origin_phi " << origin_phi << endl;
    os << "center_offset " 
	<< center_offset.x() << " " 
	<< center_offset.y() << " " 
	<< center_offset.z() << endl;

    for( vector<scanline_t>::iterator it=lines.begin();it!=lines.end();it++)
    {
	os << "line " << (*it).delta_phi;
	for( vector<unsigned int>::iterator pi=(*it).ranges.begin();pi!=(*it).ranges.end();pi++)
	{
	    os << " " << (*pi);
	}
	os << endl;

	if( (*it).ranges.size() == (*it).remissions.size() )
	{
	    os << "remission " << (*it).delta_phi;
	    for( vector<unsigned int>::iterator pi=(*it).remissions.begin();pi!=(*it).remissions.end();pi++)
	    {
		os << " " << (*pi);
	    }
	    os << endl;
	}
    }	

    return true;
}

LaserScan::Extents LaserScan::getExtents() const
{
    // TODO provide proper extents
    return Extents(); 
}
