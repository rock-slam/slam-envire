#ifndef __SCANMESHING_HPP__
#define __SCANMESHING_HPP__

#include <boost/smart_ptr.hpp>

namespace envire {
    class ScanMeshing;
    typedef boost::shared_ptr<ScanMeshing> ScanMeshing_Ptr;

    class ScanMeshing : public Operator
    {
        public:
            ScanMeshing( TriMesh_Ptr mesh );

            void addInput( LaserScan_Ptr scan ); 
            void setMaxEdgeLength( float value );
    };
}

#endif
