#include "LaserScan.hpp"

using namespace envire;

static LaserScan_Ptr LaserScan::createFromScanFile(const std::string& file, FrameNode_Ptr node)
{
    LaserScan_Ptr scan(new LaserScan());
    std::ifstream ifstream(file);
    if( ifstream.fail() )  
    {
        throw runtime_error("Could not open file '" + file + "'.");
    }
    else
    {
        try {
            scan->parseScan( ifstream );
        }
        catch( ... )
        {
            file.close();
            throw;
        }
        file.close();
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

        if( key == "rotation" ) {
            float x, u, v, w;
            iline >> x >> u >> v >> w;
            rotation = Eigen::Quaternionf(x, u, v, w);
        }

        if( key == "origin" ) {
            float x,y,z;
            iline >> x >> y >> z;
            translation = Eigen::Vector3f(x,y,z);
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
