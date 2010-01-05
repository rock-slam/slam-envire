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

    string gps_lineStr, imu_lineStr, scan_lineStr;

    GPSCoordinateTransform gpsTrans;
    bool first = true;
    double origin[3];

    while(getline(gps_log, gps_lineStr)) {
	stringstream gps_line( gps_lineStr );
	long gps_ts=0, imu_ts=0, scan_ts=0;
	double point[3];
	Eigen::Vector3d pos(point);

	string str_ts;
	//gps_line >> gps_ts;
	gps_line >> str_ts;
	gps_line >> point[0];
	gps_line >> point[1];
	gps_line >> point[2];
	gps_ts += 60*60*2+1; // empirical value for one particular dataset

	cout << str_ts << endl;
//	parseTS(str_ts);

	gpsTrans.WGS84toUTM(point);

	// base origin on first point in gps
	if( first ) {
	    first = false;
	    for(int i=0;i<3;i++)
		origin[i] = point[i];
	}
	
	for(int i=0;i<2;i++)
	    point[i] -= origin[i];

	// find corresponding records in other logs
	// corresponding meaning having a similar timestamp (next to second is ok)
	Eigen::Quaterniond orientation;	
	int skip = 0;
	while(getline(imu_log, imu_lineStr) && (gps_ts > imu_ts)) 
	{
	    stringstream imu_line( imu_lineStr );
	    imu_line >> imu_ts;
	    imu_line >> orientation.x();
	    imu_line >> orientation.y();
	    imu_line >> orientation.z();
	    imu_line >> orientation.w();

	    cout << "gps: " << gps_ts << " imu: " << imu_ts << endl;
	    skip++;
	}
	cout << "skipping " << skip << endl;

	cout << pos << endl;
	cout << orientation.x() << endl;
    }

}
