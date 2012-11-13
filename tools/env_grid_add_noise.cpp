#include <envire/Core.hpp>
#include <envire/maps/Grids.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/distributions/normal.hpp>

#include <iostream>

using namespace envire;
using namespace std;
namespace po = boost::program_options;
using boost::lexical_cast;

struct window
{
    window() {}
    window( float x, float y, float width, float height ) 
	: x(x), y(y), width(width), height(height) {}
    float x, y, width, height;
};

void validate(boost::any& v, 
              const std::vector<std::string>& values,
	      window*, int)
{
    po::validators::check_first_occurrence(v);
    const string& s = po::validators::get_single_string(values);

    vector<std::string> strs;
    boost::split(strs, s, boost::is_any_of(","));

    if( strs.size() != 4 )
        throw po::validation_error(po::validation_error::invalid_option_value);

    v = boost::any( window( 
		lexical_cast<float>( strs[0] ),
		lexical_cast<float>( strs[1] ),
		lexical_cast<float>( strs[2] ),
		lexical_cast<float>( strs[3] ) ) );
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
    window win;
    float scale, angle, sigma, radius;

    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("input-env", po::value<string>(&input_env)->required(), "input env path")
	("output-env", po::value<string>(&output_env)->required(), "output env path")
	("scale", po::value<float>(&scale)->default_value(1.0), "<scaling> of the noise")
	("slope", po::value<float>(&angle), "<orientation of slope> vs x-axis in degrees")
	("normal", po::value<float>(&sigma), "normal distribution with <sigma>")
	("sphere", po::value<float>(&radius), "sphere with <radius>")
	("const", "constant offset of 1.0 (use scaling to change)")
	("window", po::value<window>(&win), "<x,y,width,height> window parameters")
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

    // get some default options
    if( !vm.count("window") )
    {
	win.x = grid->getCenterPoint().x(); 
	win.y = grid->getCenterPoint().y();
	win.width = grid->getSizeX();
	win.height = grid->getSizeY();
    }

    bool useSlope = vm.count("slope"),
	useConst = vm.count("const"),
	useNormal = vm.count("normal"),
	useSphere = vm.count("sphere"),
	useFractal = vm.count("fractal");

    // do some more parsing
    float slopeAngle = angle / 180.0 * M_PI;
    boost::math::normal normal( 0, sigma );

    for( float x = win.x-win.width/2 + grid->getScaleX()*.5; x < win.x+win.width/2; x+= grid->getScaleX() )
    {
	for( float y = win.y-win.height/2 + grid->getScaleY()*.5; y < win.y+win.height/2; y+= grid->getScaleY() )
	{
	    const float r = sqrt( pow(x,2) + pow(y,2) );

	    // get reference to height value
	    double &height( grid->get( x, y ) );

	    if( useConst )
		height += scale;

	    if( useSlope )
		height += (cos( slopeAngle ) * x + sin( slopeAngle ) * y) * scale;
	    
	    if( useNormal )
		height += pdf( normal, r ) * scale; 

	    if( useSphere && r < radius )
		height += sqrt( pow(radius,2) - pow(r,2) ) * scale;
	}
    }	

    // write resulting env back
    env->serialize( output_env );
}

