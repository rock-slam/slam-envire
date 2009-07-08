#ifndef __LASERSCAN_HPP__
#define __LASERSCAN_HPP__

#include <boost/shared_ptr.hpp>
#include "Core.hpp"

namespace envire {

    class LaserScan;
    typedef boost::shared_ptr<LaserScan> LaserScan_Ptr;

    class LaserScan : public CartesianMap
    {
        public:
            static LaserScan_Ptr createFromScanFile(const std::string& file, FrameNode& node);
    };
                
};
#endif
