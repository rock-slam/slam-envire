#include "ScanMeshing.hpp"
#include <stdexcept>
#include <iostream>

using namespace envire;
using namespace std;

ENVIRONMENT_ITEM_DEF( ScanMeshing )

ScanMeshing::ScanMeshing()
{
    setDefaultConfiguration();
}

void ScanMeshing::setDefaultConfiguration()
{
    maxEdgeAngle = 85.0/180.0*M_PI;
    maxEdgeLength = 0.5; 
    remissionScaleFactor = 10000; 
    remissionMarkerThreshold = 16000;
    minRange = 0.1;
    maxRange = 1e9;
    extractMarkers = false;
    _useRemission = false;
}

void ScanMeshing::serialize(Serialization& so)
{
    Operator::serialize(so);
    so.write("maxEdgeLength", maxEdgeLength ); 
    so.write("remissionScaleFactor", maxEdgeLength ); 
    so.write("minRange", minRange ); 
}

void ScanMeshing::unserialize(Serialization& so)
{
    Operator::unserialize(so);
    so.read("maxEdgeLength", maxEdgeLength ); 
    so.read("remissionScaleFactor", maxEdgeLength ); 
    so.read("minRange", minRange ); 
}

void ScanMeshing::useRemission( bool value )
{
    _useRemission = value;
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

void ScanMeshing::setMaxEdgeLength( double value ) 
{
    maxEdgeLength = value;
}

void ScanMeshing::setMinRange( double value ) 
{
    minRange = value;
}

void ScanMeshing::setMaxRange( double value ) 
{
    maxRange = value;
}

void ScanMeshing::setRemissionScaleFactor( double value ) 
{
    remissionScaleFactor = value;
}

void ScanMeshing::setRemissionMarkerThreshold( long value ) 
{
    remissionMarkerThreshold = value;
}

struct Marker
{
    Eigen::Vector3d center;
    std::vector<Eigen::Vector3d> points;
    void calcCenter()
    {
	center = Eigen::Vector3d::Zero();
	for(size_t i=0;i<points.size();i++)
	{
	    center += points[i];
	}
	center /= points.size();
    };

    void addPoint(const Eigen::Vector3d &point)
    {
	points.push_back( point );
	calcCenter();
    }

    double dist(const Eigen::Vector3d &point)
    {
	return (point - center).norm();
    };
};

bool ScanMeshing::updateAll() 
{
    // this implementation can handle only one input at the moment
    if( env->getInputs(this).size() != 1 || env->getOutputs(this).size() != 1 )
        throw std::runtime_error("Scanmeshing needs to have exactly 1 input and 1 output for now.");
    
    TriMesh* meshPtr = static_cast<envire::TriMesh*>(*env->getOutputs(this).begin());
    LaserScan* scanPtr = static_cast<envire::LaserScan*>(*env->getInputs(this).begin());

    std::vector<Eigen::Vector3d>& points(meshPtr->vertices);
    std::vector<Eigen::Vector3d>& colors(meshPtr->getVertexData<Eigen::Vector3d>(TriMesh::VERTEX_COLOR));
    std::vector<TriMesh::vertex_attr>& point_attrs(meshPtr->getVertexData<TriMesh::vertex_attr>(TriMesh::VERTEX_ATTRIBUTES));
    std::vector<double>& uncertainty(meshPtr->getVertexData<double>(Pointcloud::VERTEX_VARIANCE));

    typedef TriMesh::triangle_t triangle_t;
    std::vector< triangle_t >& faces(meshPtr->faces);
    
    std::vector<Marker> markers;

    // our "update" strategy is to clear everything and redo
    points.clear();
    colors.clear();
    point_attrs.clear();
    faces.clear();
    uncertainty.clear();

    envire::LaserScan& scan(*scanPtr);

    int line1[scan.points_per_line], line2[scan.points_per_line];
    int *idx_line=line1, *prev_idx_line=line2;

    for(size_t line_num=0;line_num<scan.lines.size();line_num++) {
        const LaserScan::scanline_t& line(scan.lines[line_num]);

        float phi = scan.origin_phi + line.delta_phi;
        float psi = scan.origin_psi;

	bool has_rem = (line.ranges.size() == line.remissions.size());

	float prev_edgeAngle = M_PI*0.5;
        for(size_t point_num=0;point_num<line.ranges.size();point_num++) {
	    // check distance derivative over angle to filter ghost
	    // readings on edges
	    float range = line.ranges[point_num] / 1000.0;
	    float edgeAngle = M_PI*0.5;
	    if( point_num < (line.ranges.size() - 1) )
	    {
		float next_range = line.ranges[point_num+1] / 1000.0;
		float c = sqrt( pow(range,2) + pow(next_range,2) - 2*range*next_range*cos( 2*scan.delta_psi ) );
		edgeAngle = acos( (pow(range,2) - pow(next_range,2) + pow(c,2) ) / ( 2*range*c ) );
	    }
	    bool pass = fabs(edgeAngle-M_PI*0.5) < maxEdgeAngle &&
		fabs(prev_edgeAngle-M_PI*0.5) < maxEdgeAngle;
	    prev_edgeAngle = edgeAngle;

            if( range > minRange && range < maxRange && pass ) 
	    {
                float xx = std::cos( psi ) * range;

                Eigen::Vector3d point;
		if( scan.x_forward )
		{
		    // x-forward
		    point = Eigen::Vector3d( 
			    std::cos( phi ) * xx,
			    std::sin( psi ) * range,
			    std::sin( phi ) * xx );
		}
		else
		{
		    // y-forward
		    point = Eigen::Vector3d( 
			    -std::sin( psi ) * range,
			    std::cos( phi ) * xx,
			    std::sin( phi ) * xx );
		}

                // perform center offset compensation
                Eigen::Vector3d offset = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) * scan.center_offset;
                Eigen::Vector3d opoint = point + offset;

                points.push_back(opoint); 

		// calculate uncertainty of the point
		// TODO remove numbers and put into parameters
		double sigma = std::min(0.01 * range, 0.02);
		uncertainty.push_back( sigma * sigma );

		if( _useRemission && has_rem )
		{
		    // convert the remission value into a color value
		    // for now we do that with a simple linear conversion and a cutoff
		    // TODO really we should store the reflectivity and not convert into a color
		    // this is up to the visualisation
		    float cval = std::min( 1.0, std::max( 0.0, line.remissions[point_num] / (double)remissionScaleFactor ) );

		    colors.push_back( Eigen::Vector3d::Ones() * cval );

		    if( extractMarkers )
		    {
			// TODO maybe move into separate function
			// see if remission value is above threshold for marker
			if( line.remissions[point_num] > remissionMarkerThreshold )
			{
			    bool newMarker = true;
			    for(size_t i=0;i<markers.size();i++)
			    {
				// TODO make max distance configurable
				if( markers[i].dist( opoint ) < 0.05 )
				{
				    newMarker = false;
				    markers[i].addPoint( opoint );
				}
			    }
			    if( newMarker )
			    {
				Marker m;
				m.addPoint( opoint );
				markers.push_back( m );
			    }
			}
		    }
		}

		// see if the the scanpoint is actually on the edge of the scan
		bool edge = 
		    point_num == 0 
		    || point_num == (line.ranges.size()-1) 
		    || line_num == 0
		    || line_num == (scan.lines.size()-1);

		point_attrs.push_back( edge << TriMesh::SCAN_EDGE );

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
			    //
			    // TODO, this is not the right check for a reverse
			    // triangle, because of the offset
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
                
            psi += scan.delta_psi;
        }

        // swap line and previous line
        int *tmp = idx_line;
        idx_line = prev_idx_line;
        prev_idx_line = tmp;
    }

    for(size_t i=0;i<markers.size();i++)
    {
	std::cout << "marker " << i << " center: " << markers[i].center.transpose() << " pixel: " << markers[i].points.size() << std::endl;
    }

    // calculate vertex normals
    meshPtr->calcVertexNormals();

    // remove colors if empty 
    assert( colors.empty() || colors.size() == points.size() );
    if( colors.empty() )
	meshPtr->removeData( TriMesh::VERTEX_COLOR );

    // mark item as modified
    env->itemModified( meshPtr );

    return true;
}

