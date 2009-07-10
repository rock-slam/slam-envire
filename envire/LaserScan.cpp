#include "LaserScan.hpp"

#include <string>
#include <fstream>

#include <stdexcept>

using namespace envire;

LaserScan::LaserScan(FrameNode_Ptr node, std::string const& id) :
    CartesianMap(node, id)
{
}

LaserScan_Ptr LaserScan::createFromScanFile(const std::string& file, FrameNode_Ptr node)
{
    LaserScan_Ptr scan(new LaserScan(node, file));
    std::ifstream ifstream(file.c_str());
    if( ifstream.fail() )  
    {
        throw std::runtime_error("Could not open file '" + file + "'.");
    }
    else
    {
        try {
            scan->parseScan( ifstream );
        }
        catch( ... )
        {
            ifstream.close();
            throw;
        }
        ifstream.close();
    }
    return scan;
}

bool LaserScan::parseScan( std::istream& data ) {
    std::string line;
    while( !data.eof() ) {
        getline( data, line );
        std::istringstream iline( line );

        std::string key;
        iline >> key;

        // construct a new FrameNode if either origin or rotation are
        // specified in the scan file, uses the current FrameNode as parent
        if( key == "rotation" || key == "origin" ) {
            if( !frame )
                frame = FrameNode_Ptr( new FrameNode(frame) );
        }

        if( key == "rotation" ) {
            float x, u, v, w;
            iline >> x >> u >> v >> w;
            frame->getTransform().rotation = Eigen::Quaternionf(x, u, v, w);
        }

        if( key == "origin" ) {
            float x,y,z;
            iline >> x >> y >> z;
            frame->getTransform().translation = Eigen::Vector3f(x,y,z);
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

    return true;
}

Layer_Ptr LaserScan::clone(const std::string& id) 
{
    Layer_Ptr clone(new LaserScan(*this));
    return clone;
}
