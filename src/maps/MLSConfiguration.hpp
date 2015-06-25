#ifndef ENVIRE_MLSCONFIG_HPP__
#define ENVIRE_MLSCONFIG_HPP__

namespace envire
{

/**
 * Configuration struct which hold information on the different 
 * options and parameters of the MLS. 
 */
struct MLSConfiguration
{
    MLSConfiguration()
	: gapSize( 1.0 ), 
	thickness( 0.05 ),
	useColor( false ),
	updateModel( KALMAN ),
	useNegativeInformation( false ) {}

    enum update_model
    {
	KALMAN,
	SUM,
	SLOPE
    };

    float gapSize;
    float thickness;
    bool useColor;
    update_model updateModel;
    bool useNegativeInformation;
};

}

#endif
