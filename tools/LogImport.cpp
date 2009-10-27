#include <iostream>
#include <fstream>
#include <sstream>
#include <envire/Core.hpp>
#include <boost/lexical_cast.hpp>
#include <ogr_spatialref.h>

using namespace std;

/**
 * This tool will parse three log files, gps.log, imu.log and scan.log and import
 * them into an environemnt, which is then saved (to be viewed with enview).
 */

class GPSCoordinateTransform
{
    const static int UTM_ZONE = 32;
    const static bool UTM_NORTH = true;

    OGRSpatialReference oSourceSRS, oTargetSRS;
    OGRCoordinateTransformation *poCT;

public:
    GPSCoordinateTransform()
    {
	oSourceSRS.SetWellKnownGeogCS( "WGS84" );
	oTargetSRS.SetWellKnownGeogCS( "WGS84" );
	oTargetSRS.SetUTM( UTM_ZONE, UTM_NORTH );

	poCT = OGRCreateCoordinateTransformation( &oSourceSRS,
		&oTargetSRS );
    };

    ~GPSCoordinateTransform()
    {
	if( poCT )
	    OCTDestroyCoordinateTransformation( poCT );
    };

    void WGS84toUTM( double *p )
    {
	poCT->Transform(1, p+0, p+1, p+2);
    };
};

int main(int argc, char* argv[])
{
    if( argc < 5 ) 
    {
	cout << "usage: logimport gps.log imu.log scan.log out_dir" << endl;
	exit(0);
    }

    ifstream gps_log( argv[1] );
    ifstream imu_log( argv[2] );
    ifstream scan_log( argv[3] );

    GPSCoordinateTransform gpsTrans;
    bool first = true;
    string lineStr;

    while(getline(gps_log, lineStr)) {
	stringstream line( lineStr );
	long ts;
	double point[3];
	double origin[3];

	line >> ts;
	line >> point[0];
	line >> point[1];
	line >> point[2];

	gpsTrans.WGS84toUTM(point);

	// base origin on first point in gps
	if( first ) {
	    first = false;
	    for(int i=0;i<3;i++)
		origin[i] = point[i];
	}
	
	for(int i=0;i<2;i++)
	    point[i] -= origin[i];

	cout << point[0] << " " << point[1] << " " << point[2] << endl;
    }

}
