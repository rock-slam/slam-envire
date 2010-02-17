#include "ScanMeshing.hpp"
#include <stdexcept>

using namespace envire;
using namespace std;

const std::string ScanMeshing::className = "ScanMeshing";

ScanMeshing::ScanMeshing()
    : maxEdgeLength(0.5), remissionScaleFactor(10000)
{
}

ScanMeshing::ScanMeshing(Serialization& so)
    : Operator(so)
{
    so.setClassName(className);
    so.read("maxEdgeLength", maxEdgeLength ); 
    so.read("remissionScaleFactor", maxEdgeLength ); 
}

void ScanMeshing::serialize(Serialization& so)
{
    Operator::serialize(so);
    so.setClassName(className);
    so.write("maxEdgeLength", maxEdgeLength ); 
    so.write("remissionScaleFactor", maxEdgeLength ); 
}


void ScanMeshing::addInput( LaserScan* scan ) 
{
    Operator::addInput(scan);
}

void ScanMeshing::addOutput( TriMesh* mesh )
{
    
    if( env->getOutputs(this).size() > 0 )
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
    if( env->getInputs(this).size() != 1 || env->getOutputs(this).size() != 1 )
        throw std::runtime_error("Scanmeshing needs to have exactly 1 input and 1 output for now.");
    
    TriMesh* meshPtr = static_cast<envire::TriMesh*>(*env->getOutputs(this).begin());
    LaserScan* scanPtr = static_cast<envire::LaserScan*>(*env->getInputs(this).begin());

    std::vector<Eigen::Vector3f>& points(meshPtr->points);
    std::vector<Eigen::Vector3f>& colors(meshPtr->colors);
    typedef TriMesh::triangle_t triangle_t;
    std::vector< triangle_t >& faces(meshPtr->faces);

    envire::LaserScan& scan(*scanPtr);

    int line1[scan.points_per_line], line2[scan.points_per_line];
    int *idx_line=line1, *prev_idx_line=line2;

    for(int line_num=0;line_num<scan.lines.size();line_num++) {
        const LaserScan::scanline_t& line(scan.lines[line_num]);

        float phi = scan.origin_phi + line.delta_phi;
        float psi = scan.origin_psi;

	bool has_rem = (line.ranges.size() == line.remissions.size());

        for(int point_num=0;point_num<line.ranges.size();point_num++) {
            // TODO check what actually is a valid range
            if( line.ranges[point_num] > 100 ) {
                float range = line.ranges[point_num] / 1000.0;
                float xx = std::cos( psi ) * range;

                Eigen::Vector3f point( 
                    std::sin( psi ) * range,
                    std::cos( phi ) * xx,
                    std::sin( phi ) * xx );

                // perform center offset compensation
                Eigen::Vector3f offset = Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX()) * scan.center_offset;

                points.push_back( point - offset ); 
		if( has_rem )
		{
		    // convert the remission value into a color value
		    // for now we do that with a simple linear conversion and a cutoff
		    float cval = std::min( 1.0, std::max( 0.0, line.remissions[point_num] / (double)remissionScaleFactor ) );

		    colors.push_back( Eigen::Vector3f::Ones() * cval );
		}

                idx_line[point_num] = points.size() - 1;
            } else {
                idx_line[point_num] = -1;
            }
           
            if( line_num > 0 && point_num > 0) {
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
                            if( fabs(psi) > (M_PI/2) ) {
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
