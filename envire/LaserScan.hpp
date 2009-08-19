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

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            LaserScan(std::string const& id);
            
	    bool parseScan( std::string& file );
            bool parseScan( std::istream& data );
            Layer* clone( const std::string& id);
    };
                
};
#endif
