#include "ScanMeshing.hpp"
#include <stdexcept>

using namespace envire;

void ScanMeshing::addInput( LaserScan_Ptr scan ) 
{
    Operator::addInput(scan);
}

void ScanMeshing::addOutput( TriMesh_Ptr mesh )
{
    if( outputs.size() > 0 )
        throw std::runtime_error("ScanMeshing can only have one output.");

    Operator::addOutput(mesh);
}

void ScanMeshing::setMaxEdgeLength( float value ) 
{
    maxEdgeLength = value;
}

bool ScanMeshing::updateAll() 
{
    // this implementation can handle only one input at the moment
    if( inputs.size() != 1 || outputs.size() != 1 )
        throw std::runtime_error("Scanmeshing needs to have exactly 1 input and 1 output for now.");
    
    TriMesh_Ptr meshPtr = boost::static_pointer_cast<envire::TriMesh>(*outputs.begin());
    LaserScan_Ptr scanPtr = boost::static_pointer_cast<envire::LaserScan>(*inputs.begin());

    const double PI = 3.141592;

    std::vector<Eigen::Vector3f>& points(meshPtr->points);
    typedef TriMesh::triangle_t triangle_t;
    std::vector< triangle_t > faces(meshPtr->faces);

    envire::LaserScan& scan(*scanPtr);

    int line1[scan.points_per_line], line2[scan.points_per_line];
    int *idx_line=line1, *prev_idx_line=line2;

    for(int line_num=0;line_num<scan.lines.size();line_num++) {
        const LaserScan::scanline_t& line(scan.lines[line_num]);

        float phi = scan.origin_phi + line.first;
        float psi = scan.origin_psi;

        for(int point_num=0;point_num<line.second.size();point_num++) {
            // TODO check what actually is a valid range
            if( line.second[point_num] > 100 ) {
                float range = line.second[point_num] / 1000.0;
                float xx = std::cos( psi ) * range;

                // TODO needed to mirror the image... don't know why 
                // could be located in scan file generation
                Eigen::Vector3f point( 
                    -std::sin( psi ) * range,
                    std::cos( phi ) * xx,
                    std::sin( phi ) * xx );

                // perform center offset compensation
                Eigen::Vector3f offset = Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX()) * scan.center_offset;

                points.push_back( point - offset ); 
                idx_line[point_num] = points.size() - 1;
            } else {
                idx_line[point_num] = -1;
            }
           
            if( line_num > 0 ) {
                // prev_line is valid now, we can start looking for polygons
                int poly[4] = { prev_idx_line[point_num-1], 
                    prev_idx_line[point_num],
                    idx_line[point_num],
                    idx_line[point_num-1] };

                int first_poly = -1;

                for(int n=0;n<4;n++) {
                    int idx[] = {0+(n<=0),1+(n<=1),2+(n<=2)};
                    
                    // check polygon for existance and then see if all
                    // three edges are within the threshold limit
                    if( (poly[idx[0]] > 0) && (poly[idx[1]] > 0) && (poly[idx[2]] > 0) && 
                            (first_poly==-1 || (first_poly+n)%2==0 ) ) {

                        float dist_a = (points[poly[idx[0]]] - points[poly[idx[1]]]).norm();
                        float dist_b = (points[poly[idx[0]]] - points[poly[idx[2]]]).norm();
                        float dist_c = (points[poly[idx[1]]] - points[poly[idx[2]]]).norm();

                        if( dist_a < maxEdgeLength && dist_b < maxEdgeLength && dist_c < maxEdgeLength ) {
                            // found a polygon save idx to prevent the
                            // overlapping part to be selected, but still look
                            // for a second one
                            first_poly = n;

                            // observe the triangle order for faces on the
                            // other half of the sphere
                            if( fabs(psi) > (PI/2) ) {
                                triangle_t tri( poly[idx[0]], poly[idx[2]], poly[idx[1]] );
                                faces.push_back( tri );
                            } else {
                                triangle_t tri( poly[idx[0]], poly[idx[1]], poly[idx[2]] );
                                faces.push_back( tri );
                            }
                        }
                    }
                }
            }
                
            psi -= scan.delta_psi;
        }

        // swap line and previous line
        int *tmp = idx_line;
        idx_line = prev_idx_line;
        prev_idx_line = tmp;
    }
}
