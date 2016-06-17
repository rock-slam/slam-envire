#include <envire/maps/MLSGrid.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <iostream>
#include <fstream>

using namespace envire;
using namespace Eigen;
using namespace std;

struct MLSTest 
{
    MLSGrid::Ptr grid;

    // generate random number generator
    boost::mt19937 eng;
    boost::variate_generator<boost::mt19937,boost::normal_distribution<float> > norm;
    boost::variate_generator<boost::mt19937,boost::uniform_real<float> > uni;

    MLSTest()
	: norm( eng, boost::normal_distribution<float>(0,1) ),
	uni( eng, boost::uniform_real<float>(0,1) ) {};

    virtual ~MLSTest() {};
        
    /** generate a grid on 0x0 to 1x1 */
    void init( size_t cell_count )
    {
	grid = new MLSGrid( cell_count, cell_count, 1.0 / cell_count, 1.0 / cell_count );
	grid->getConfig().thickness = 1.0;
	grid->getConfig().gapSize = 1.5;
    }

    void learnModel( size_t iterations, float var_range = 0.1 )
    {
	for( size_t i=0; i<iterations; i++ )
	{
	    const float x = uni();
	    const float y = uni();
	    const float z = func( x, y );

	    const float stdev = uni() * var_range + 0.01;
	    const float z_meas = z + norm() * stdev;

	    MLSGrid::SurfacePatch patch( z_meas, stdev );
	    grid->update( Vector2d( x, y ), patch );
	}
    }

    void evaluateModel( size_t iterations, ostream& os )
    {
	for( size_t i=0; i<iterations; i++ )
	{
	    const float x = uni();
	    const float y = uni();
	    const float z = func( x, y );
	    MLSGrid::SurfacePatch *patch;
	    double p_z = z, p_stdev = 1.0;
	    if( (patch = grid->get( Vector2d( x, y ), p_z, p_stdev )) )
	    {
		const float diff = (p_z - z);
	        const float ndiff = diff / p_stdev;

		os << diff << " " << ndiff << endl;
	    }
	}
    }

    void saveEnv( const string& name )
    {
	Environment env;
	env.attachItem( grid.get() );
	grid->setFrameNode( env.getRootNode() );

	env.serialize( name );
    }

    /** underlying function that is learned */
    virtual float func( float x, float y )
    {
	return (sin( x ) + cos( y ) - 1.0)*1.0;
    }
};

int main(int argc, char* argv[])
{
    const int grid_size = 5;
    const int learn_steps = 10000;
    const int eval_steps = 10000;
    {
    MLSTest test;
    test.init(grid_size);
    test.learnModel( learn_steps );
    ofstream of("samples.dat");
    test.evaluateModel( eval_steps, of );
    test.saveEnv("/tmp/test.env");
    }

    {
    MLSTest test;
    test.init(grid_size);
    test.grid->getConfig().updateModel = MLSConfiguration::SUM;
    test.learnModel( learn_steps );
    ofstream of("samples2.dat");
    test.evaluateModel( eval_steps, of );
    test.saveEnv("/tmp/test2.env");
    }

    {
    MLSTest test;
    test.init(grid_size);
    test.grid->getConfig().updateModel = MLSConfiguration::SLOPE;
    test.learnModel( learn_steps );
    ofstream of("samples3.dat");
    test.evaluateModel( eval_steps, of );
    test.saveEnv("/tmp/test3.env");
    }
}

