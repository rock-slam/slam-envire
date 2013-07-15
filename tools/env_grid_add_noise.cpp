#include <envire/Core.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <libnoise/noise.h>
#include <iostream>

using namespace envire;
using namespace std;
namespace po = boost::program_options;
using boost::lexical_cast;

struct fractal
{
    fractal()
	: octaves(8), frequency(1.0), persistence(0.5) {}
    int octaves;
    float frequency;
    float persistence;
};

struct position
{
    position() : x(0), y(0) {}
    position( float x, float y )
	: x(x), y(y) {}
    float x, y;
};

void validate(boost::any& v, 
              const std::vector<std::string>& values,
	      fractal*, int)
{
    po::validators::check_first_occurrence(v);
    const string& s = po::validators::get_single_string(values);

    vector<std::string> strs;
    boost::split(strs, s, boost::is_any_of(","));

    if( strs.size() != 3 )
        throw po::validation_error(po::validation_error::invalid_option_value);

    fractal f;
    f.octaves = lexical_cast<int>( strs[0] );
    f.frequency = lexical_cast<float>( strs[1] );
    f.persistence = lexical_cast<float>( strs[2] );

    v = boost::any( f );
}

void validate(boost::any& v, 
              const std::vector<std::string>& values,
	      position*, int)
{
    po::validators::check_first_occurrence(v);
    const string& s = po::validators::get_single_string(values);

    vector<std::string> strs;
    boost::split(strs, s, boost::is_any_of(","));

    if( strs.size() != 2 )
        throw po::validation_error(po::validation_error::invalid_option_value);

    v = boost::any( position( 
		lexical_cast<float>( strs[0] ),
		lexical_cast<float>( strs[1] ) ) );
}

void validate(boost::any& v, 
              const std::vector<std::string>& values,
	      float*, int)
{
    po::validators::check_first_occurrence(v);
    const string& s = po::validators::get_single_string(values);

    v = boost::any( lexical_cast<float>( s ) );
}

int main(int argc, char* argv[])
{
    string input_env, output_env;
    position center, win;
    fractal frac;
    float scale = 1.0, angle, sigma = 1.0, radius;

    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("input-env", po::value<string>(&input_env)->required(), "input env path")
	("output-env", po::value<string>(&output_env)->required(), "output env path")
	("scale", po::value<float>(&scale)->default_value(1.0), "<scaling> of the noise")
	("slope", po::value<float>(&angle), "<orientation of slope> vs x-axis in degrees")
	("normal", po::value<float>(&sigma), "normal distribution with <sigma>")
	("sphere", po::value<float>(&radius), "sphere with <radius>")
	("fractal", po::value<fractal>(&frac), "fractal with noise <octaves,frequency,persistance>")
	("const", "constant offset of 1.0 (use scaling to change)")
	("noise", "add gaussian noise with sigma 1.0")
	("center", po::value<position>(&center), "<x,y> center of origin")
	("window", po::value<position>(&win), "<width,height> window parameters")
	;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (argc <= 1 || vm.count("help")) {
	cout << desc << "\n";
	return 1;
    }
    po::notify(vm);    

    // load environment
    boost::scoped_ptr<Environment> env(Environment::unserialize(input_env));

    ElevationGrid *grid;
    // get first elevationgrid in file
    vector<ElevationGrid*> grids = env->getItems<ElevationGrid>();
    if( grids.empty() )
    {
	cerr << "no elevation grid found in environment." << endl;
	exit(1);
    }
    grid = grids.front();
    cerr << "using ElevationGrid with id: " << grid->getUniqueId() << endl;

    if( !vm.count("window") )
    {
	win.x = grid->getSizeX();
	win.y = grid->getSizeY();
    }

    bool useSlope = vm.count("slope"),
	useConst = vm.count("const"),
	useNormal = vm.count("normal"),
	useNoise = vm.count("noise"),
	useSphere = vm.count("sphere"),
	useFractal = vm.count("fractal");

    // do some more parsing
    float slopeAngle = angle / 180.0 * M_PI;
    boost::math::normal normal( 0, sigma );

    // generate random number generator
    boost::mt19937 eng;
    boost::variate_generator<boost::mt19937,boost::normal_distribution<float> > 
	gen( eng, boost::normal_distribution<float>(0,1) );

    // Initialize fractal noise
    noise::module::Perlin perlin;
    perlin.SetOctaveCount( frac.octaves );
    perlin.SetFrequency( frac.frequency );
    perlin.SetPersistence( frac.persistence );

    // calculate window on which to operate
    float 
	minx = std::max( grid->getOffsetX(), center.x - win.x/2.0 ),
	maxx = std::min( grid->getOffsetX() + grid->getSizeX(), center.x + win.x/2.0 ),
	miny = std::max( grid->getOffsetY(), center.y - win.y/2.0 ),
	maxy = std::min( grid->getOffsetY() + grid->getSizeY(), center.y + win.y/2.0 );

    const float sx = grid->getScaleX(), sy = grid->getScaleY();
    const float normalScale = 1.0 / pdf( normal, 0 );
    ElevationGrid::ArrayType &raster = grid->getGridData( ElevationGrid::ELEVATION );
    for( float x = minx + grid->getScaleX()*.5; x < maxx; x+= sx )
    {
	for( float y = miny + grid->getScaleY()*.5; y < maxy; y+= sy )
	{
	    // get reference to height value
            size_t raster_x, raster_y;
            grid->toGrid(x, y, raster_x, raster_y);
	    double &height( raster[raster_y][raster_x] );

	    const float r = sqrt( pow(x-center.x,2) + pow(y-center.y,2) );

	    if( useConst )
		height += scale;

	    if( useSlope )
		height += (cos( slopeAngle ) * x + sin( slopeAngle ) * y) * scale;
	    
	    if( useNormal )
		height += pdf( normal, r ) * scale * normalScale; 

	    if( useSphere && r < radius )
		height += sqrt( pow(radius,2) - pow(r,2) ) * scale;

	    if( useNoise )
		height += gen() * scale;

	    if( useFractal )
		height += perlin.GetValue( x, y, 0 ) * scale;
	}
    }	

    // write resulting env back
    env->serialize( output_env );
}

