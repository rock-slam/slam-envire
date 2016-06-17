#ifndef __LASERSCAN_HPP__
#define __LASERSCAN_HPP__

#include <Eigen/Core>
#include <envire/core/Serialization.hpp>
#include <envire/Core.hpp>
#include <vector>
#include <base/samples/LaserScan.hpp>

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

	    /** 
	     * if true, the center of the scan is on the x-axis,
	     * if set to false, the center is on the y-axis (default)
	     */
	    bool x_forward;

	private:
	    bool parseScan( std::istream& is, Transform& transform );

        public:
	    void addScanLine( double tilt_angle, const base::samples::LaserScan& scan );

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    LaserScan();

	    void setXForward();
	    void setYForward();
            
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
