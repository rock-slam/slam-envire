#ifndef __LASERSCAN_HPP__
#define __LASERSCAN_HPP__

#include <Eigen/Core>
#include <envire/Core.hpp>
#include <vector>
#include <base/samples/laser_scan.h>

namespace envire {

    class LaserScan : public Map<3>
    {
	ENVIRONMENT_ITEM( LaserScan )

        public:
	    /** delta_phi is the step size in rads perpendicular to the scan
	     * direction of the laser scanner (normally axis of the pan unit)
	     * */
	    struct scanline_t
	    {
		float delta_phi;
		std::vector<unsigned int> ranges;
		std::vector<unsigned int> remissions;
	    };

            /** angular stepsize in rad between scan points in scan direction of
             * the laser scan */
            float delta_psi;

            /** starting angle for psi */
            float origin_psi;

            /** starting angle for phi */
            float origin_phi;

            /** offset of the scanner from the rotational center */
            Eigen::Vector3d center_offset;

            /** scan points per scan line */
            int points_per_line;

            /** list of lines in the scan */
            std::vector<scanline_t> lines;

	private:
	    bool parseScan( std::istream& is, envire::FrameNode::TransformType& transform );

        public:
	    void addScanLine( double tilt_angle, const base::samples::LaserScan& scan );

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    LaserScan();
            
            LaserScan(Serialization& so);
	    void serialize(Serialization& so);
            void unserialize(Serialization& so);
            
	    bool readScan( std::istream& is );
	    bool writeScan( std::ostream& os ); 

	    const std::string getMapFileName() const;

	    static LaserScan* importScanFile( const std::string& file, FrameNode* frame );

	    Extents getExtents() const;
    };
                
};
#endif
