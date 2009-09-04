#ifndef __LASERSCAN_HPP__
#define __LASERSCAN_HPP__

#include <Eigen/Core>
#include "Core.hpp"
#include <vector>

namespace envire {

    class LaserScan : public CartesianMap
    {
        public:
            /** scanline typedef, first in tuple specifies the delta_phi. This is
             * the step size in rads perpendicular to the scan direction of the
             * laser scanner (normally axis of the pan unit */
            typedef std::pair< float, std::vector<int> > scanline_t;

            /** angular stepsize in rad between scan points in scan direction of
             * the laser scan */
            float delta_psi;

            /** starting angle for psi */
            float origin_psi;

            /** starting angle for phi */
            float origin_phi;

            /** offset of the scanner from the rotational center */
            Eigen::Vector3f center_offset;

            /** scan points per scan line */
            int points_per_line;

            /** list of lines in the scan */
            std::vector<scanline_t> lines;

	private:
	    static const std::string className;
	    bool parseScan( const std::string& file, envire::FrameNode::TransformType& transform );

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    LaserScan();
            
            LaserScan(Serialization& so);
	    void serialize(Serialization& so);
	    const std::string& getClassName() const {return className;};
            
	    bool readScan( const std::string& file );
	    bool writeScan( const std::string& file ); 

            LaserScan* clone();

	    static LaserScan* importScanFile( const std::string& file, FrameNode* frame );
    };
                
};
#endif
