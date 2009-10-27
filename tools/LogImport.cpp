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

#define UTM_ZONE 32
#define UTM_NORTH TRUE

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

    string lineStr;

    bool first = true;
    while(getline(gps_log, lineStr)) {
	stringstream line( lineStr );
	long ts;
	double point[3];
	double origin[3];

	line >> ts;
	line >> point[0];
	line >> point[1];
	line >> point[2];

	OGRSpatialReference oSourceSRS;
	oSourceSRS.SetWellKnownGeogCS( "WGS84" );

	OGRSpatialReference oTargetSRS;
	oTargetSRS.SetWellKnownGeogCS( "WGS84" );
	oTargetSRS.SetUTM( UTM_ZONE, UTM_NORTH );

	OGRCoordinateTransformation *poCT;
	poCT = OGRCreateCoordinateTransformation( &oSourceSRS,
		&oTargetSRS );

	poCT->Transform(1, point, point+1, point+2);

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
