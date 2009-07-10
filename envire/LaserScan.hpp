#ifndef __LASERSCAN_HPP__
#define __LASERSCAN_HPP__

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "Core.hpp"
#include <vector>

namespace envire {

    class LaserScan;
    typedef boost::shared_ptr<LaserScan> LaserScan_Ptr;

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
            
            LaserScan(FrameNode_Ptr node, std::string const& id = "");
            LaserScan(FrameNode_Ptr node, Operator_Ptr generator, std::string const& id = "");
            
            bool parseScan( std::istream& data );
            Layer_Ptr clone( const std::string& id = "");

            static LaserScan_Ptr createFromScanFile(const std::string& file, FrameNode_Ptr node);
    };
                
};
#endif
